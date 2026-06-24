//! 모든 LiDAR 모델이 공유하는 측정 도메인 타입.
//!
//! 모델(LDROBOT STL-27L, Pacecat LDS-50C-E …)마다 바이트 레이아웃은 다르지만,
//! 파싱이 끝난 결과는 이 공용 타입으로 수렴한다. 덕분에 수집기·뷰어는 모델을
//! 몰라도 되고, 새 모델은 "바이트 → 이 타입"만 구현하면 기존 파이프라인에 붙는다.

/// 측정 점 1개: 각도(도) + 거리(mm) + 신호 세기.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarPoint {
    pub angle_degrees: f32,
    pub distance_mm: u16,
    pub intensity: u8,
}

/// 한 프레임(패킷) 단위 측정 결과. 모델마다 채울 수 있는 메타데이터가 다르면
/// 모르는 값은 0으로 둔다(예: Pacecat 데이터 패킷에는 회전 속도가 없다).
#[derive(Debug, Clone, PartialEq)]
pub struct LidarBody {
    pub speed_degrees_per_second: u16,
    pub start_angle_degrees: f32,
    pub end_angle_degrees: f32,
    pub timestamp_ms: u32,
    pub points: Vec<LidarPoint>,
}

/// 프레임 파싱 실패 사유. 여러 모델을 함께 다루므로 헤더는 `u16`, 체크섬은 `u32`로
/// 넉넉히 잡는다(STL-27L의 1바이트 헤더/CRC8도 그대로 담긴다).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    InvalidLength { expected: usize, actual: usize },
    InvalidHeader(u16),
    InvalidVerLen(u8),
    CrcMismatch { expected: u32, actual: u32 },
}
