//! Pacecat LDS-50C-E — UDP `0xFAC7` 패킷 모델.
//!
//! STL-27L과 달리 네트워크(UDP) 기반이라, 데이터를 받기 전에 센서에 시작 커맨드를
//! 보내야 한다. 책임을 세 파일로 나눈다.
//!
//! - [`frame`]: `0xFAC7` 패킷 레이아웃(28B 헤더 + dist/ang/strength 배열 + 16-bit sum)과 파싱.
//! - [`decoder`]: 바이트(데이터그램) 스트림에서 패킷을 잘라내는 스트리밍 디코더.
//! - [`command`]: `LVERSH`/`LSTARH` 시작 커맨드 빌드(STM32 CRC32)와 UDP 연결·핸드셰이크.

pub mod command;
pub mod decoder;
pub mod frame;

/// 호스트가 데이터를 받을 기본 UDP 포트(LidarConfig.json `host_port`).
pub const DEFAULT_HOST_PORT: u16 = 6789;
/// 센서로 커맨드를 보내는 기본 포트(LidarConfig.json `command_port`, 고정).
pub const DEFAULT_COMMAND_PORT: u16 = 6543;
