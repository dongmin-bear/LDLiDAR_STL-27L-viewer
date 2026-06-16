//! STL-27L LiDAR 크레이트.
//!
//! - [`reader`]: 시리얼 바이트 스트림을 프레임으로 디코딩·파싱.
//! - [`viewer`]: egui_plot 기반 2D 포인트 뷰어.
//!
//! 데이터 흐름은 `docs/ARCHITECTURE.md` 참고.

pub mod reader;
pub mod viewer;

// 자주 쓰는 공개 API를 크레이트 루트에서 바로 쓸 수 있게 재노출.
pub use reader::{BAUD_RATE, FrameDecoder, LidarBody, LidarPoint, ParseError, crc8, parse};
