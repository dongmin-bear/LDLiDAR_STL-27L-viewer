//! LDROBOT STL-27L — 시리얼 47바이트 프레임 모델.
//!
//! - [`frame`]: 47B 프레임의 바이너리 레이아웃과 zero-copy 파싱 + CRC8.
//! - [`decoder`]: 바이트 스트림에서 프레임을 잘라내는 스트리밍 디코더([`decoder::FrameDecoder`]).

pub mod decoder;
pub mod frame;

/// STL-27L 기본 통신 속도(baud).
pub const BAUD_RATE: u32 = 921_600;
