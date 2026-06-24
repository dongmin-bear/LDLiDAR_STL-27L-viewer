//! LiDAR 데이터 읽기·파싱 모듈 — 여러 제조사/모델을 하나의 파이프라인으로 묶는다.
//!
//! 구조: 공용 계층(모델 독립) + 제조사/모델별 코덱.
//!
//! - [`types`]: 모든 모델이 공유하는 도메인 타입([`LidarPoint`]/[`LidarBody`]/[`ParseError`]).
//! - [`model`]: 모델 추상화 — [`Decoder`] 트레잇과 [`Model`] 팩토리.
//! - [`transport`]: 전송 추상화 — 시리얼/UDP를 [`transport::Transport`]로 가린다.
//! - [`data_collector`]: 전송·코덱 독립 수집기. 읽기·재연결·회전 조립을 끝내고 완성된
//!   [`Scan`]을 publish한다([`ScanFeed`]). 뷰어는 그리기만 한다.
//! - [`ldrobot`] / [`pacecat`]: 제조사별 모델 코덱(데이터 파싱 + 스트리밍 디코더).
//!
//! 새 모델 추가법: `<제조사>/<모델>/` 폴더에 `frame.rs`(파싱)·`decoder.rs`([`Decoder`] 구현)를
//! 두고 [`Model`]에 한 줄 등록하면, 수집기·뷰어는 손대지 않고 그대로 동작한다.

pub mod data_collector;
pub mod model;
pub mod transport;
pub mod types;

// 제조사별 모듈(폴더명에 하이픈이 있어 #[path]로 잇는다).
// 경로는 reader.rs가 있는 `src/` 기준이라 `reader/` 접두사가 붙는다.
#[path = "reader/LDROBOT/mod.rs"]
pub mod ldrobot;
#[path = "reader/Pacecat/mod.rs"]
pub mod pacecat;

// 공용 API 재노출.
pub use data_collector::{ConnectionStatus, Scan, ScanFeed, Source, spawn};
pub use model::{Decoder, Model};
pub use types::{LidarBody, LidarPoint, ParseError};

// STL-27L 기본 통신 속도(back-compat: `ldlidar::BAUD_RATE`).
pub use ldrobot::stl27l::BAUD_RATE;

// STL-27L 코덱 재노출(back-compat: `ldlidar` 콘솔 바이너리가 직접 사용).
pub use ldrobot::stl27l::decoder::FrameDecoder;
pub use ldrobot::stl27l::frame::{
    CRC_OFFSET, HEADER, PACKET_LEN, POINT_SIZE, POINTS_PER_PACKET, VER_LEN, crc8, parse,
};
