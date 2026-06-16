//! 들어오는 측정점을 각도 버킷에 쌓아 "최신 한 바퀴" 스캔을 유지한다.
//!
//! LiDAR는 같은 각도를 계속 다시 지나가므로, 각도를 일정 간격으로 나눈 버킷마다
//! 가장 최근 측정값만 보관하면 회전 경계를 따로 감지하지 않아도 안정적인
//! 실시간 화면을 얻을 수 있다.
//!
//! decay time(잔상 시간)을 켜면, 마지막으로 측정된 뒤 그 시간이 지난(=갱신이 끊긴)
//! 점은 화면에서 제외한다. 데이터가 끊기거나 물체가 지나간 자리는 자동으로 정리된다.

use std::time::{Duration, Instant};

use crate::LidarPoint;

/// 한 각도 버킷의 최신 샘플.
#[derive(Debug, Clone, Copy)]
struct Sample {
    distance_m: f32,
    intensity: u8,
    /// 이 버킷이 마지막으로 갱신된 시각(decay 판단용).
    received: Instant,
}

/// 직교좌표로 변환된 한 점(렌더링용).
#[derive(Debug, Clone, Copy)]
pub struct CartesianPoint {
    pub x: f64,
    pub y: f64,
    pub intensity: u8,
}

/// 각도 버킷 배열. 인덱스 = 각도를 `bins`개로 나눈 칸.
pub struct ScanBuffer {
    buckets: Vec<Option<Sample>>,
}

impl ScanBuffer {
    /// `bins`개의 버킷(한 바퀴 분해능)으로 초기화. 항상 1 이상으로 맞춘다.
    pub fn new(bins: usize) -> Self {
        Self {
            buckets: vec![None; bins.max(1)],
        }
    }

    /// 분해능이 바뀌면 버킷 수를 다시 잡는다. 기존 데이터는 비운다.
    pub fn resize(&mut self, bins: usize) {
        let bins = bins.max(1);
        if bins != self.buckets.len() {
            self.buckets = vec![None; bins];
        }
    }

    /// 측정점 하나를 해당 각도 버킷에 기록(최신값으로 덮어씀, 수신 시각 갱신).
    /// 거리 0(=무효 측정)은 버린다.
    pub fn ingest(&mut self, point: &LidarPoint) {
        self.ingest_at(point, Instant::now());
    }

    /// 수신 시각을 명시해 기록한다(테스트에서 시간을 통제하기 위함).
    fn ingest_at(&mut self, point: &LidarPoint, now: Instant) {
        if point.distance_mm == 0 {
            return;
        }
        let bins = self.buckets.len();
        let normalized = point.angle_degrees.rem_euclid(360.0);
        let index = ((normalized / 360.0 * bins as f32).floor() as usize) % bins;
        self.buckets[index] = Some(Sample {
            distance_m: point.distance_mm as f32 / 1000.0,
            intensity: point.intensity,
            received: now,
        });
    }

    /// `max_range_m` 이내이고 decay로 만료되지 않은 점들을 직교좌표로 변환해 돌려준다.
    ///
    /// `decay`가 `None`이면 시간 제한 없이 모두 유지한다. LiDAR 각도 0°를 +x축,
    /// 반시계 방향을 +각도로 둔다.
    pub fn cartesian_points(&self, max_range_m: f32, decay: Option<Duration>) -> Vec<CartesianPoint> {
        self.cartesian_points_at(max_range_m, decay, Instant::now())
    }

    /// 기준 시각을 명시한 변환(테스트용). `cartesian_points`가 `now`로 호출한다.
    fn cartesian_points_at(
        &self,
        max_range_m: f32,
        decay: Option<Duration>,
        now: Instant,
    ) -> Vec<CartesianPoint> {
        let bins = self.buckets.len();
        self.buckets
            .iter()
            .enumerate()
            .filter_map(|(index, sample)| {
                let sample = (*sample)?;
                if sample.distance_m > max_range_m {
                    return None;
                }
                if is_expired(now.duration_since(sample.received), decay) {
                    return None;
                }
                let angle_rad = (index as f32 / bins as f32) * std::f32::consts::TAU;
                Some(CartesianPoint {
                    x: (sample.distance_m * angle_rad.cos()) as f64,
                    y: (sample.distance_m * angle_rad.sin()) as f64,
                    intensity: sample.intensity,
                })
            })
            .collect()
    }
}

/// 샘플 나이(`age`)가 decay 한계를 넘었는지. `decay`가 `None`이면 절대 만료되지 않는다.
fn is_expired(age: Duration, decay: Option<Duration>) -> bool {
    match decay {
        Some(limit) => age > limit,
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(angle: f32, distance_mm: u16, intensity: u8) -> LidarPoint {
        LidarPoint {
            angle_degrees: angle,
            distance_mm,
            intensity,
        }
    }

    #[test]
    fn ingest_then_convert_zero_degrees_lands_on_positive_x() {
        let mut scan = ScanBuffer::new(360);
        scan.ingest(&point(0.0, 1000, 200));
        let pts = scan.cartesian_points(12.0, None);
        assert_eq!(pts.len(), 1);
        assert!((pts[0].x - 1.0).abs() < 1e-3);
        assert!(pts[0].y.abs() < 1e-3);
    }

    #[test]
    fn ninety_degrees_lands_on_positive_y() {
        let mut scan = ScanBuffer::new(360);
        scan.ingest(&point(90.0, 2000, 200));
        let pts = scan.cartesian_points(12.0, None);
        assert!(pts[0].x.abs() < 1e-3);
        assert!((pts[0].y - 2.0).abs() < 1e-3);
    }

    #[test]
    fn zero_distance_is_discarded() {
        let mut scan = ScanBuffer::new(360);
        scan.ingest(&point(45.0, 0, 100));
        assert!(scan.cartesian_points(12.0, None).is_empty());
    }

    #[test]
    fn out_of_range_points_are_filtered() {
        let mut scan = ScanBuffer::new(360);
        scan.ingest(&point(0.0, 20_000, 100));
        assert!(scan.cartesian_points(12.0, None).is_empty());
    }

    #[test]
    fn latest_sample_per_bucket_wins() {
        let mut scan = ScanBuffer::new(360);
        scan.ingest(&point(0.0, 1000, 100));
        scan.ingest(&point(0.0, 3000, 100));
        let pts = scan.cartesian_points(12.0, None);
        assert_eq!(pts.len(), 1);
        assert!((pts[0].x - 3.0).abs() < 1e-3);
    }

    #[test]
    fn point_survives_within_decay_window() {
        let mut scan = ScanBuffer::new(360);
        let t0 = Instant::now();
        scan.ingest_at(&point(0.0, 1000, 100), t0);
        // 300ms 경과, decay 500ms → 아직 살아있어야 함.
        let pts = scan.cartesian_points_at(12.0, Some(Duration::from_millis(500)), t0 + Duration::from_millis(300));
        assert_eq!(pts.len(), 1);
    }

    #[test]
    fn point_expires_after_decay_window() {
        let mut scan = ScanBuffer::new(360);
        let t0 = Instant::now();
        scan.ingest_at(&point(0.0, 1000, 100), t0);
        // 600ms 경과, decay 500ms → 사라져야 함.
        let pts = scan.cartesian_points_at(12.0, Some(Duration::from_millis(500)), t0 + Duration::from_millis(600));
        assert!(pts.is_empty());
    }

    #[test]
    fn none_decay_never_expires() {
        assert!(!is_expired(Duration::from_secs(3600), None));
    }
}
