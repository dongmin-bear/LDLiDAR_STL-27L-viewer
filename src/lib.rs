//! 멀티모델 LiDAR 크레이트.
//!
//! - [`reader`]: 바이트 스트림(시리얼/UDP)을 프레임으로 디코딩·파싱하고, 점을 한 바퀴
//!   [`reader::Scan`]으로 재구성해 publish하는 수집기([`reader::ScanFeed`])까지 포함.
//!   제조사/모델별 코덱(LDROBOT STL-27L, Pacecat LDS-50C-E)을 [`reader::Model`]로 고른다.
//! - [`viewer`]: 완성된 스캔을 받아 직교좌표로 그리는 egui_plot 2D 뷰어(모델 독립).
//!
//! 데이터 흐름은 `docs/ARCHITECTURE.md` 참고.

pub mod reader;
pub mod viewer;

// 자주 쓰는 공개 API를 크레이트 루트에서 바로 쓸 수 있게 재노출.
pub use reader::{
    BAUD_RATE, ConnectionStatus, FrameDecoder, LidarBody, LidarPoint, Model, ParseError, Scan,
    ScanFeed, Source, crc8, parse,
};
