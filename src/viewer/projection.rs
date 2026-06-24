//! 극좌표 측정점을 화면에 그리기 위한 직교좌표 변환(표현 계층).
//!
//! 수집기가 넘긴 [`LidarPoint`](각도°, 거리mm)를 (x, y) m로 바꾼다. 이 변환은
//! 렌더링 준비이고 `max_range`는 뷰어 설정이므로, 수집(reader) 쪽이 아니라 뷰어에
//! 둔다. 수집 계층은 표현·설정을 전혀 모른다.

use crate::LidarPoint;

/// 직교좌표로 변환된 한 점(렌더링용).
#[derive(Debug, Clone, Copy)]
pub struct CartesianPoint {
    pub x: f64,
    pub y: f64,
    pub intensity: u8,
}

/// 스캔의 극좌표 점들을 직교좌표로 변환한다. `max_range_m`보다 먼 점은 제외한다.
/// LiDAR 각도 0°를 +x축, 반시계 방향을 +각도로 둔다.
pub fn project(points: &[LidarPoint], max_range_m: f32) -> Vec<CartesianPoint> {
    points
        .iter()
        .filter_map(|point| {
            let distance_m = point.distance_mm as f32 / 1000.0;
            if distance_m > max_range_m {
                return None;
            }
            let rad = point.angle_degrees.to_radians();
            Some(CartesianPoint {
                x: (distance_m * rad.cos()) as f64,
                y: (distance_m * rad.sin()) as f64,
                intensity: point.intensity,
            })
        })
        .collect()
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
    fn zero_degrees_lands_on_positive_x() {
        let out = project(&[pt(0.0, 1000, 200)], 12.0);
        assert!((out[0].x - 1.0).abs() < 1e-3);
        assert!(out[0].y.abs() < 1e-3);
    }

    #[test]
    fn ninety_degrees_lands_on_positive_y() {
        let out = project(&[pt(90.0, 2000, 200)], 12.0);
        assert!(out[0].x.abs() < 1e-3);
        assert!((out[0].y - 2.0).abs() < 1e-3);
    }

    #[test]
    fn out_of_range_points_are_filtered() {
        let out = project(&[pt(0.0, 20_000, 100), pt(0.0, 1000, 100)], 12.0);
        assert_eq!(out.len(), 1); // 20m 점은 12m 범위 밖이라 제외
    }
}
