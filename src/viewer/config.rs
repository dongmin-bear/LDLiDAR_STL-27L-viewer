//! 뷰어 런타임 설정. `config.toml`을 읽고, 파일이 바뀌면 자동으로 다시 읽어
//! 점 크기·색·표시 옵션을 실시간으로 갱신한다(핫리로드).

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};

use anyhow::Context as _;
use notify::{Event, RecommendedWatcher, RecursiveMode, Watcher};
use serde::Deserialize;

/// `config.toml` 전체 스키마. 누락된 필드는 `#[serde(default)]`로 기본값을 채운다.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct ViewerConfig {
    pub points: PointsConfig,
    pub scan: ScanConfig,
    pub lidar: LidarConfig,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct PointsConfig {
    /// 점 반지름(px).
    pub size: f32,
    /// 점 색(`#RRGGBB` 또는 `#RRGGBBAA`).
    pub color: String,
    /// true면 신호 세기(intensity)에 따라 밝기를 조절한다.
    pub color_by_intensity: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct ScanConfig {
    /// 표시할 최대 거리(m). 이보다 먼 점은 그리지 않는다.
    pub max_range_m: f32,
    /// 극좌표 거리 링(동심원) 표시 여부.
    pub show_range_rings: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct LidarConfig {
    /// 모델명. 전송 방식을 이 값으로 정한다: "STL-27L"=시리얼, "LDS-50C-E"=UDP.
    pub model: String,

    // --- 시리얼 모델(STL-27L 등) ---
    /// 시리얼 포트 경로.
    pub port: String,
    /// 통신 속도(baud). STL-27L 기본은 921600. 바꾸면 뷰어 재시작 필요.
    pub baud: u32,

    // --- UDP 모델(LDS-50C-E 등) ---
    /// 센서(LiDAR)의 IP. 커맨드를 이 주소로 보낸다.
    pub sensor_ip: String,
    /// 센서 커맨드 포트(고정 6543).
    pub command_port: u16,
    /// 호스트(이 PC)가 데이터를 받을 바인드 IP. "0.0.0.0"이면 모든 인터페이스.
    pub host_ip: String,
    /// 호스트가 데이터를 받을 UDP 포트.
    pub host_port: u16,

    /// true면 하드웨어 없이 합성 데이터로 동작(데모 모드).
    pub demo: bool,
}

impl Default for PointsConfig {
    fn default() -> Self {
        Self {
            size: 2.0,
            color: "#34d399".to_string(),
            color_by_intensity: true,
        }
    }
}

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            max_range_m: 12.0,
            show_range_rings: true,
        }
    }
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            model: "STL-27L".to_string(),
            port: "/dev/ttyUSB0".to_string(),
            baud: 921_600,
            sensor_ip: "192.168.158.98".to_string(),
            command_port: 6543,
            host_ip: "0.0.0.0".to_string(),
            host_port: 6789,
            demo: false,
        }
    }
}

impl ViewerConfig {
    /// 점 색을 egui 색으로 변환. 파싱 실패 시 기본색으로 폴백한다.
    pub fn point_color(&self) -> egui::Color32 {
        parse_hex_color(&self.points.color).unwrap_or(egui::Color32::from_rgb(0x34, 0xd3, 0x99))
    }
}

/// 설정 파일을 읽는다. 파일이 없으면 기본값을 써서 새로 만든 뒤 그 기본값을 돌려준다.
pub fn load_or_create(path: &Path) -> anyhow::Result<ViewerConfig> {
    if !path.exists() {
        std::fs::write(path, DEFAULT_CONFIG_TOML)
            .with_context(|| format!("기본 설정 파일 생성 실패: {}", path.display()))?;
        return Ok(ViewerConfig::default());
    }
    load(path)
}

/// 설정 파일을 읽어 파싱한다.
pub fn load(path: &Path) -> anyhow::Result<ViewerConfig> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("설정 파일 읽기 실패: {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("설정 파일 파싱 실패: {}", path.display()))
}

/// 설정 파일을 감시해, 바뀔 때마다 새로 파싱한 `ViewerConfig`를 채널로 보낸다.
///
/// 에디터가 파일을 통째로 교체(rename)하는 경우까지 잡으려고 파일이 아닌
/// 상위 디렉터리를 감시하고 파일명으로 걸러낸다. 워처는 반환된 핸들이
/// 살아있는 동안만 동작하므로 호출자가 보관해야 한다.
pub fn watch(path: &Path) -> anyhow::Result<(Receiver<ViewerConfig>, RecommendedWatcher)> {
    let (tx, rx) = mpsc::channel();

    let watched_path = path.to_path_buf();
    let dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));

    let mut watcher = notify::recommended_watcher(move |result: notify::Result<Event>| {
        let Ok(event) = result else { return };
        // 이 설정 파일을 건드린 이벤트만 처리.
        let touched = event.paths.iter().any(|p| same_file(p, &watched_path));
        if !touched {
            return;
        }
        if let Ok(config) = load(&watched_path) {
            // 수신 측이 사라졌으면 조용히 종료.
            let _ = tx.send(config);
        }
    })
    .context("파일 워처 생성 실패")?;

    watcher
        .watch(&dir, RecursiveMode::NonRecursive)
        .with_context(|| format!("디렉터리 감시 실패: {}", dir.display()))?;

    Ok((rx, watcher))
}

/// 두 경로가 같은 파일을 가리키는지 파일명 기준으로 비교(상대/절대 혼용 대응).
fn same_file(a: &Path, b: &Path) -> bool {
    match (a.file_name(), b.file_name()) {
        (Some(x), Some(y)) => x == y,
        _ => a == b,
    }
}

/// `#RGB` / `#RRGGBB` / `#RRGGBBAA` 16진 색 문자열을 파싱한다.
fn parse_hex_color(input: &str) -> Option<egui::Color32> {
    let hex = input.trim().strip_prefix('#')?;
    let byte = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).ok();

    match hex.len() {
        6 => Some(egui::Color32::from_rgb(byte(0)?, byte(2)?, byte(4)?)),
        8 => Some(egui::Color32::from_rgba_unmultiplied(
            byte(0)?,
            byte(2)?,
            byte(4)?,
            byte(6)?,
        )),
        3 => {
            // #RGB → 각 자리 두 번 반복.
            let nib = |c: char| c.to_digit(16).map(|v| (v * 17) as u8);
            let mut cs = hex.chars();
            Some(egui::Color32::from_rgb(
                nib(cs.next()?)?,
                nib(cs.next()?)?,
                nib(cs.next()?)?,
            ))
        }
        _ => None,
    }
}

/// 처음 실행 시 자동 생성되는 기본 설정 파일 내용.
pub const DEFAULT_CONFIG_TOML: &str = r##"# LiDAR 뷰어 설정 — 저장하면 뷰어에 즉시 반영됩니다(핫리로드).

[points]
size = 2.0              # 점 반지름(px)
color = "#34d399"       # 점 색 (#RRGGBB 또는 #RRGGBBAA)
color_by_intensity = true

[scan]
max_range_m = 12.0      # 표시 최대 거리(m)
show_range_rings = true # 거리 동심원 표시
# 화면은 한 바퀴(rotation)가 끝날 때마다 그 회전 전체를 스냅샷으로 그린다(decay 불필요).

[lidar]
model = "STL-27L"       # "STL-27L"=시리얼, "LDS-50C-E"=UDP. 모델별로 아래 해당 항목 사용
demo = false            # true면 하드웨어 없이 합성 데이터로 동작

# STL-27L (시리얼) 설정
port = "/dev/ttyUSB0"
baud = 921600           # 통신 속도. 데이터는 오는데 프레임이 안 잡히면 230400/115200 등으로 시도

# LDS-50C-E (UDP) 설정 — model = "LDS-50C-E"일 때 사용
sensor_ip = "192.168.158.98"   # 센서 IP (커맨드 전송 대상)
command_port = 6543            # 센서 커맨드 포트(고정)
host_ip = "0.0.0.0"            # 이 PC 수신 바인드 IP (특정 NIC면 그 IP로)
host_port = 6789               # 이 PC 데이터 수신 포트
"##;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_six_digit_hex() {
        assert_eq!(
            parse_hex_color("#ff8800"),
            Some(egui::Color32::from_rgb(255, 136, 0))
        );
    }

    #[test]
    fn parses_short_hex() {
        assert_eq!(
            parse_hex_color("#f80"),
            Some(egui::Color32::from_rgb(255, 136, 0))
        );
    }

    #[test]
    fn rejects_garbage_hex() {
        assert_eq!(parse_hex_color("not-a-color"), None);
    }

    #[test]
    fn default_config_toml_parses() {
        let config: ViewerConfig = toml::from_str(DEFAULT_CONFIG_TOML).unwrap();
        assert_eq!(config.points.size, 2.0);
        assert!(config.scan.show_range_rings);
    }
}
