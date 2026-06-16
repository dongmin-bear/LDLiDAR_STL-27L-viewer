//! 한 바퀴(rotation)가 끝날 때마다 그 회전의 점들을 "완성 스캔"으로 확정해 그린다.
//!
//! 더블 버퍼: `current`(누적 중) / `display`(마지막 완성). 각도가 크게 되감기면
//! (예: 300° → 10°) 한 바퀴가 끝난 것으로 보고 두 버퍼를 교체한다. 매 회전이 직전
//! 회전을 통째로 교체하므로 잔상/부분호 깜빡임이 없고 별도 decay가 필요 없다.

use crate::LidarPoint;

/// 각도가 이만큼(도) 줄어들면 한 바퀴가 끝난 것(wrap)으로 본다. 측정 노이즈로 인한
/// 소폭 후진과 진짜 회전 경계(≈360°)를 구분하려고 절반 회전을 임계로 둔다.
const WRAP_THRESHOLD_DEG: f32 = 180.0;
/// wrap이 안 잡히는 비정상 스트림에서 메모리 무한 증가를 막는 안전 상한.
const MAX_SCAN_POINTS: usize = 20_000;

/// 한 점의 원시 측정값(직교 변환 전).
struct RawSample {
    angle_degrees: f32,
    distance_m: f32,
    intensity: u8,
}

/// 직교좌표로 변환된 한 점(렌더링용).
#[derive(Debug, Clone, Copy)]
pub struct CartesianPoint {
    pub x: f64,
    pub y: f64,
    pub intensity: u8,
}

/// 회전 단위 더블 버퍼.
pub struct ScanBuffer {
    /// 지금 누적 중인 회전.
    current: Vec<RawSample>,
    /// 마지막으로 완성된 회전(이걸 그린다).
    display: Vec<RawSample>,
    /// 직전 점의 각도(wrap 감지용).
    last_angle: Option<f32>,
}

impl ScanBuffer {
    pub fn new() -> Self {
        Self {
            current: Vec::new(),
            display: Vec::new(),
            last_angle: None,
        }
    }

    /// 측정점 하나를 누적한다. 각도가 크게 되감기면(한 바퀴 끝) 누적분을 완성 스캔으로
    /// 확정하고 새 회전을 시작한다. 거리 0(무효)은 버린다.
    ///
    /// 반환값: 이번 점으로 한 바퀴가 **확정됐으면** 그 완성 스캔의 점 개수(`Some(n)`),
    /// 아니면 `None`. (호출자가 "프레임당 point 개수"를 로그로 찍는 데 쓴다.)
    pub fn ingest(&mut self, point: &LidarPoint) -> Option<usize> {
        if point.distance_mm == 0 {
            return None;
        }
        let angle = point.angle_degrees.rem_euclid(360.0);

        let wrapped = self
            .last_angle
            .is_some_and(|prev| prev - angle > WRAP_THRESHOLD_DEG);
        let mut completed = None;
        if wrapped || self.current.len() >= MAX_SCAN_POINTS {
            // 누적 중이던 회전을 완성본으로 확정하고(swap) 새 회전을 비운다.
            std::mem::swap(&mut self.display, &mut self.current);
            self.current.clear();
            completed = Some(self.display.len());
        }

        self.last_angle = Some(angle);
        self.current.push(RawSample {
            angle_degrees: angle,
            distance_m: point.distance_mm as f32 / 1000.0,
            intensity: point.intensity,
        });
        completed
    }

    /// 마지막으로 완성된 한 바퀴를 직교좌표로 변환해 돌려준다.
    /// LiDAR 각도 0°를 +x축, 반시계 방향을 +각도로 둔다.
    pub fn cartesian_points(&self, max_range_m: f32) -> Vec<CartesianPoint> {
        self.display
            .iter()
            .filter(|sample| sample.distance_m <= max_range_m)
            .map(|sample| {
                let rad = sample.angle_degrees.to_radians();
                CartesianPoint {
                    x: (sample.distance_m * rad.cos()) as f64,
                    y: (sample.distance_m * rad.sin()) as f64,
                    intensity: sample.intensity,
                }
            })
            .collect()
    }
}

impl Default for ScanBuffer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(angle: f32, distance_mm: u16, intensity: u8) -> LidarPoint {
        LidarPoint {
            angle_degrees: angle,
            distance_mm,
            intensity,
        }
    }

    #[test]
    fn nothing_to_draw_before_first_full_rotation() {
        let mut scan = ScanBuffer::new();
        for a in [0.0, 90.0, 180.0, 270.0] {
            scan.ingest(&pt(a, 1000, 10));
        }
        // 아직 wrap이 없었으므로 완성 스캔(display)이 비어 있다.
        assert!(scan.cartesian_points(12.0).is_empty());
    }

    #[test]
    fn wrap_finalizes_previous_rotation() {
        let mut scan = ScanBuffer::new();
        for a in [0.0, 100.0, 200.0, 300.0] {
            scan.ingest(&pt(a, 1000, 10));
        }
        scan.ingest(&pt(10.0, 1000, 10)); // 300→10 : 290°>180 → wrap
        let pts = scan.cartesian_points(12.0);
        assert_eq!(pts.len(), 4); // 직전 회전의 4점이 확정됨
    }

    #[test]
    fn zero_degrees_lands_on_positive_x() {
        let mut scan = ScanBuffer::new();
        scan.ingest(&pt(0.0, 1000, 200));
        scan.ingest(&pt(300.0, 1000, 10));
        scan.ingest(&pt(10.0, 1000, 10)); // wrap → 직전 회전 확정
        let pts = scan.cartesian_points(12.0);
        assert!((pts[0].x - 1.0).abs() < 1e-3);
        assert!(pts[0].y.abs() < 1e-3);
    }

    #[test]
    fn ninety_degrees_lands_on_positive_y() {
        let mut scan = ScanBuffer::new();
        scan.ingest(&pt(90.0, 2000, 200));
        scan.ingest(&pt(300.0, 1000, 10));
        scan.ingest(&pt(10.0, 1000, 10)); // wrap
        let pts = scan.cartesian_points(12.0);
        assert!(pts[0].x.abs() < 1e-3);
        assert!((pts[0].y - 2.0).abs() < 1e-3);
    }

    #[test]
    fn zero_distance_is_discarded() {
        let mut scan = ScanBuffer::new();
        scan.ingest(&pt(45.0, 0, 100));
        scan.ingest(&pt(300.0, 1000, 10));
        scan.ingest(&pt(10.0, 1000, 10)); // wrap → display는 [300]만 (45°,0은 버려짐)
        let pts = scan.cartesian_points(12.0);
        assert_eq!(pts.len(), 1);
    }

    #[test]
    fn out_of_range_points_are_filtered() {
        let mut scan = ScanBuffer::new();
        scan.ingest(&pt(0.0, 20_000, 100));
        scan.ingest(&pt(300.0, 1000, 10));
        scan.ingest(&pt(10.0, 1000, 10)); // wrap
        let pts = scan.cartesian_points(12.0);
        assert_eq!(pts.len(), 1); // 20m 점은 12m 범위 밖이라 제외
    }
}
