//! LiDAR 포인트 뷰어 실행 바이너리.
//!
//! 사용법:
//!   cargo run --bin viewer                # ./config.toml 사용(없으면 자동 생성)
//!   cargo run --bin viewer -- my.toml     # 설정 파일 경로 지정

use std::path::PathBuf;

const DEFAULT_CONFIG_PATH: &str = "config.toml";

fn main() -> anyhow::Result<()> {
    let config_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_CONFIG_PATH));

    ldlidar::viewer::run(config_path)
}
