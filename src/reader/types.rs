//! LiDAR 측정 데이터의 공개 도메인 타입.

/// 측정 점 1개: 각도(도) + 거리(mm) + 신호 세기.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarPoint {
    pub angle_degrees: f32,
    pub distance_mm: u16,
    pub intensity: u8,
}

/// 한 프레임(패킷) 단위 측정 결과.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarBody {
    pub speed_degrees_per_second: u16,
    pub start_angle_degrees: f32,
    pub end_angle_degrees: f32,
    pub timestamp_ms: u16,
    pub points: Vec<LidarPoint>,
}

/// 프레임 파싱 실패 사유.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    InvalidLength { expected: usize, actual: usize },
    InvalidHeader(u8),
    InvalidVerLen(u8),
    CrcMismatch { expected: u8, actual: u8 },
}
