//! STL-27L LiDAR 데이터 읽기·파싱 모듈.
//!
//! 현대식(2018+) 모듈 레이아웃: 이 `reader.rs`가 모듈 루트이고, 하위 모듈은
//! 같은 이름의 `reader/` 폴더에 둔다.
//!
//! - [`frame`]: 47바이트 프레임의 바이너리 레이아웃과 zero-copy 파싱.
//! - [`decoder`]: 바이트 스트림에서 프레임을 잘라내는 스트리밍 디코더([`FrameDecoder`]).
//! - [`types`]: 공개 도메인 타입([`LidarPoint`]/[`LidarBody`]/[`ParseError`]).

pub mod decoder;
pub mod frame;
pub mod types;

/// STL-27L 기본 통신 속도(baud).
pub const BAUD_RATE: u32 = 921_600;

pub use decoder::FrameDecoder;
pub use frame::{
    crc8, parse, CRC_OFFSET, HEADER, PACKET_LEN, POINT_SIZE, POINTS_PER_PACKET, VER_LEN,
};
pub use types::{LidarBody, LidarPoint, ParseError};
