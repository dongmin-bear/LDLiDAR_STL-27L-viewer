//! egui_plot 기반 2D LiDAR 포인트 뷰어(다중 라이다).
//!
//! - 좌측 패널에서 **라이다 개수·모델을 드롭다운으로 고르고 Start를 눌러야** 연결을 시작한다
//!   (실행 즉시 자동 연결하지 않는다). 설정 패널은 토글로 열고 닫는다.
//! - 여러 대를 동시에 연결해 한 화면에 색을 달리해 겹쳐 그린다.
//! - 그리드·좌표축·줌·팬은 egui_plot 내장. `config.toml`의 점 스타일은 핫리로드된다.
//!
//! 책임은 **렌더링·제어뿐**이다. 읽기·파싱·회전 재구성은 모두
//! [`crate::reader::data_collector`]가 끝내고, 뷰어는 완성된 [`Scan`]을 pull해서
//! 직교좌표([`projection`])로 변환해 그린다.

pub mod config;
pub mod projection;

use std::path::PathBuf;

use eframe::egui;
use egui_plot::{
    CoordinatesFormatter, Corner, Line, MarkerShape, Plot, PlotPoint, PlotPoints, Points, Text,
};
use notify::RecommendedWatcher;

use crate::reader::{self, ConnectionStatus, Model, Scan, ScanFeed};
use config::ViewerConfig;
use projection::CartesianPoint;

/// 의도별 색 코드를 한곳에 모아둔 팔레트.
mod palette {
    use eframe::egui::Color32;
    pub const RING: Color32 = Color32::from_rgb(60, 70, 80);
    pub const RING_LABEL: Color32 = Color32::from_rgb(120, 130, 140);
    /// 라이다 대수별 구분 색(겹쳐 그릴 때 장치를 구분).
    pub const DEVICE: [Color32; 4] = [
        Color32::from_rgb(0x34, 0xd3, 0x99), // green
        Color32::from_rgb(0x60, 0xa5, 0xfa), // blue
        Color32::from_rgb(0xfb, 0xbf, 0x24), // amber
        Color32::from_rgb(0xf4, 0x72, 0xb6), // pink
    ];
}

const INTENSITY_TIERS: usize = 5;
/// 동시에 설정할 수 있는 라이다 최대 대수.
const MAX_LIDARS: usize = 4;

/// 뷰어를 실행한다. `config_path`가 없으면 기본 설정으로 새로 만든다.
/// 실행 직후에는 **연결하지 않고** 설정 패널만 띄운다(Start를 눌러야 시작).
pub fn run(config_path: PathBuf) -> anyhow::Result<()> {
    let initial = config::load_or_create(&config_path)?;

    // 설정 파일 핫리로드 감시(워처 핸들은 앱이 살아있는 동안 보관해야 동작한다).
    let (config_rx, watcher) = config::watch(&config_path)?;

    let app = ViewerApp::new(initial, config_rx, watcher);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 760.0])
            .with_title("LiDAR Viewer"),
        ..Default::default()
    };

    eframe::run_native(
        "LiDAR Viewer",
        options,
        Box::new(|cc| {
            install_fonts(&cc.egui_ctx);
            Ok(Box::new(app))
        }),
    )
    .map_err(|error| anyhow::anyhow!("eframe 실행 실패: {error}"))
}

/// 드롭다운에서 고르는 라이다 종류(모델 + 데모).
#[derive(Clone, Copy, PartialEq, Eq)]
enum LidarKind {
    Stl27L,
    Lds50CE,
    Demo,
}

impl LidarKind {
    const ALL: [LidarKind; 3] = [LidarKind::Stl27L, LidarKind::Lds50CE, LidarKind::Demo];

    fn label(&self) -> &'static str {
        match self {
            LidarKind::Stl27L => "STL-27L (시리얼)",
            LidarKind::Lds50CE => "LDS-50C-E (UDP)",
            LidarKind::Demo => "데모 (합성)",
        }
    }
}

/// 설정 문자열을 드롭다운 종류로 해석한다(대소문자·구분자 무시). "demo"면 합성.
fn kind_from_name(name: &str) -> LidarKind {
    let key: String = name
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_lowercase())
        .collect();
    if key == "demo" {
        return LidarKind::Demo;
    }
    match Model::from_name(name) {
        Some(Model::Lds50CE) => LidarKind::Lds50CE,
        _ => LidarKind::Stl27L,
    }
}

/// 한 라이다의 런타임 편집 상태. 텍스트필드용으로 값은 문자열로 보관하고 Start 때 파싱한다.
#[derive(Clone)]
struct LidarEntry {
    kind: LidarKind,
    // 시리얼(STL-27L)
    port: String,
    baud: String,
    // UDP(LDS-50C-E)
    sensor_ip: String,
    command_port: String,
    host_ip: String,
    host_port: String,
}

impl LidarEntry {
    /// `config.toml`의 한 `[[lidar]]` 항목으로 편집 행을 채운다.
    fn from_config(c: &config::LidarConfig) -> Self {
        let kind = kind_from_name(&c.model);
        Self {
            kind,
            port: c.port.clone(),
            baud: c.baud.to_string(),
            sensor_ip: c.sensor_ip.clone(),
            command_port: c.command_port.to_string(),
            host_ip: c.host_ip.clone(),
            host_port: c.host_port.to_string(),
        }
    }

    /// 둘째 행부터 쓰는 합리적 기본값(LDS는 한 PC에서 데이터 포트가 겹치면 안 되므로 비워두지 않음).
    fn new_default(index: usize) -> Self {
        Self {
            kind: LidarKind::Stl27L,
            port: format!("/dev/ttyUSB{index}"),
            baud: "921600".to_string(),
            sensor_ip: "192.168.0.10".to_string(),
            command_port: "6543".to_string(),
            host_ip: "0.0.0.0".to_string(),
            host_port: (6789 + index as u16).to_string(),
        }
    }

    /// 편집값을 수집기 [`reader::Source`]로 변환한다. 숫자 파싱 실패는 기본값으로 보정한다.
    fn to_source(&self) -> reader::Source {
        match self.kind {
            LidarKind::Demo => reader::Source::Demo,
            LidarKind::Stl27L => reader::Source::Serial {
                model: Model::Stl27L,
                port: self.port.clone(),
                baud: self.baud.trim().parse().unwrap_or(921_600),
            },
            LidarKind::Lds50CE => reader::Source::Udp {
                model: Model::Lds50CE,
                sensor_ip: self.sensor_ip.trim().to_string(),
                command_port: self.command_port.trim().parse().unwrap_or(6543),
                host_ip: self.host_ip.trim().to_string(),
                host_port: self.host_port.trim().parse().unwrap_or(6789),
            },
        }
    }

    /// 상태 패널에 보일 짧은 엔드포인트 문구.
    fn endpoint(&self) -> String {
        match self.kind {
            LidarKind::Demo => "합성 데이터".to_string(),
            LidarKind::Stl27L => self.port.clone(),
            LidarKind::Lds50CE => format!("{}:{}", self.sensor_ip, self.command_port),
        }
    }
}

/// 한글이 깨지지 않도록 시스템 CJK 폰트를 egui에 등록한다. 후보 경로를 차례로
/// 시도하고, 하나도 없으면 경고만 남기고 기본 폰트로 진행한다.
fn install_fonts(ctx: &egui::Context) {
    const CANDIDATES: &[&str] = &[
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKkr-Regular.otf",
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    ];

    let Some((path, bytes)) = CANDIDATES
        .iter()
        .find_map(|path| std::fs::read(path).ok().map(|bytes| (*path, bytes)))
    else {
        eprintln!("[viewer] 한글 폰트를 찾지 못했습니다 — 라벨이 깨질 수 있습니다.");
        return;
    };
    eprintln!("[viewer] 한글 폰트 로드: {path}");

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "cjk".to_owned(),
        std::sync::Arc::new(egui::FontData::from_owned(bytes)),
    );
    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "cjk".to_owned());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .insert(0, "cjk".to_owned());

    ctx.set_fonts(fonts);
}

/// eframe 애플리케이션 상태.
struct ViewerApp {
    config: ViewerConfig,
    /// 설정 편집 행들(라이다별). Start 전까지는 이것만 편집한다.
    entries: Vec<LidarEntry>,
    /// 활성 수집 피드(Start 후 채워짐, Stop 시 비워짐).
    feeds: Vec<ScanFeed>,
    /// 각 피드의 마지막 완성 스캔(새 스캔이 오기 전까지 유지).
    scans: Vec<Scan>,
    /// 연결 동작 중인지(Start 누름 상태).
    running: bool,
    /// 설정 패널 열림 여부(토글).
    settings_open: bool,
    config_rx: std::sync::mpsc::Receiver<ViewerConfig>,
    /// notify 워처. Drop되면 감시가 멈추므로 필드로 살려둔다.
    _watcher: RecommendedWatcher,
}

impl ViewerApp {
    fn new(
        config: ViewerConfig,
        config_rx: std::sync::mpsc::Receiver<ViewerConfig>,
        watcher: RecommendedWatcher,
    ) -> Self {
        let mut entries: Vec<LidarEntry> =
            config.lidars.iter().map(LidarEntry::from_config).collect();
        if entries.is_empty() {
            entries.push(LidarEntry::new_default(0));
        }
        entries.truncate(MAX_LIDARS);
        Self {
            config,
            entries,
            feeds: Vec::new(),
            scans: Vec::new(),
            running: false,
            settings_open: true, // 시작 시 설정을 펼쳐 사용자가 고르게 한다.
            config_rx,
            _watcher: watcher,
        }
    }

    /// 핫리로드 채널에서 가장 최근 (스타일)설정을 받아 반영한다.
    fn drain_config(&mut self) {
        let mut latest = None;
        while let Ok(config) = self.config_rx.try_recv() {
            latest = Some(config);
        }
        if let Some(config) = latest {
            self.config = config;
        }
    }

    /// 각 피드에서 최신 스캔을 pull해 교체한다(없으면 직전 것 유지).
    fn pull_latest_scans(&mut self) {
        for (i, feed) in self.feeds.iter().enumerate() {
            if let Some(scan) = feed.take_latest() {
                self.scans[i] = scan;
            }
        }
    }

    /// 현재 편집 행대로 수집을 시작한다(기존 피드가 있으면 먼저 정리).
    fn start(&mut self) {
        self.stop_all();
        self.feeds = self
            .entries
            .iter()
            .map(|e| reader::spawn(e.to_source()))
            .collect();
        self.scans = vec![Scan::default(); self.feeds.len()];
        self.running = true;
        self.settings_open = false; // 시작하면 화면을 넓게 보도록 설정을 접는다.
    }

    /// 모든 수집 스레드를 멈추고 피드를 비운다.
    fn stop_all(&mut self) {
        for feed in &self.feeds {
            feed.stop();
        }
        self.feeds.clear();
        self.scans.clear();
        self.running = false;
    }

    /// 라이다 개수를 `count`에 맞춘다(늘면 기본값 추가, 줄면 뒤에서 자름).
    fn resize_entries(&mut self, count: usize) {
        let count = count.clamp(1, MAX_LIDARS);
        while self.entries.len() < count {
            self.entries
                .push(LidarEntry::new_default(self.entries.len()));
        }
        self.entries.truncate(count);
    }

    /// 장치 i의 표시 색: 한 대면 config 색, 여러 대면 구분 색.
    fn device_color(&self, index: usize) -> egui::Color32 {
        if self.feeds.len() <= 1 {
            self.config.point_color()
        } else {
            palette::DEVICE[index % palette::DEVICE.len()]
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_config();
        if self.running {
            self.pull_latest_scans();
        }

        egui::Panel::left("info")
            .resizable(true)
            .default_size(300.0)
            .show_inside(ui, |ui| self.left_panel(ui));

        // 중앙 플롯: 장치별 (점들, 색)을 모아 한 번에 그린다.
        let max_range = self.config.scan.max_range_m;
        let devices: Vec<(Vec<CartesianPoint>, egui::Color32)> = self
            .scans
            .iter()
            .enumerate()
            .map(|(i, scan)| {
                (
                    projection::project(&scan.points, max_range),
                    self.device_color(i),
                )
            })
            .collect();

        egui::CentralPanel::default().show_inside(ui, |ui| draw_plot(ui, &self.config, &devices));

        // 데이터가 계속 흐르므로 매 프레임 다시 그린다.
        ui.ctx().request_repaint();
    }
}

impl ViewerApp {
    /// 좌측 패널: 제목 → Start/Stop·설정 토글 → (설정 펼침 시) 편집 UI → 상태 목록.
    fn left_panel(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        ui.heading("LiDAR Viewer");
        ui.separator();

        // Start/Stop + 설정 토글을 가장 위에.
        ui.horizontal(|ui| {
            if self.running {
                if ui.button("■ Stop").clicked() {
                    self.stop_all();
                    self.settings_open = true;
                }
            } else if ui.button("▶ Start").clicked() {
                self.start();
            }
            let toggle = if self.settings_open {
                "⚙ 설정 닫기"
            } else {
                "⚙ 설정 열기"
            };
            if ui.button(toggle).clicked() {
                self.settings_open = !self.settings_open;
            }
        });
        ui.separator();

        if self.settings_open {
            egui::ScrollArea::vertical()
                .max_height(380.0)
                .show(ui, |ui| self.settings_ui(ui));
            ui.separator();
        }

        self.status_ui(ui);

        ui.separator();
        ui.label(egui::RichText::new("점 스타일(크기·색)은 config.toml 저장 시 즉시 반영").weak());
        ui.label(egui::RichText::new("휠: 확대/축소 · 드래그: 이동").weak());
    }

    /// 설정 편집 UI: 라이다 개수 드롭다운 + 행별 모델 드롭다운 + 파라미터.
    fn settings_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("라이다 개수");
            let mut count = self.entries.len();
            egui::ComboBox::from_id_salt("lidar_count")
                .selected_text(count.to_string())
                .show_ui(ui, |ui| {
                    for n in 1..=MAX_LIDARS {
                        ui.selectable_value(&mut count, n, n.to_string());
                    }
                });
            if count != self.entries.len() {
                self.resize_entries(count);
            }
        });

        let editable = !self.running; // 연결 중에는 잠그고, Stop 후 편집.
        if self.running {
            ui.label(
                egui::RichText::new("연결 중 — 값을 바꾸려면 Stop 후 편집하세요.")
                    .weak()
                    .italics(),
            );
        }

        for i in 0..self.entries.len() {
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let (rect, _) =
                    ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                ui.painter()
                    .rect_filled(rect, 2.0, palette::DEVICE[i % palette::DEVICE.len()]);
                ui.strong(format!("라이다 #{}", i + 1));
            });

            ui.add_enabled_ui(editable, |ui| {
                // 모델 드롭다운.
                egui::ComboBox::from_id_salt(("kind", i))
                    .selected_text(self.entries[i].kind.label())
                    .show_ui(ui, |ui| {
                        for k in LidarKind::ALL {
                            ui.selectable_value(&mut self.entries[i].kind, k, k.label());
                        }
                    });

                // 모델별 파라미터.
                match self.entries[i].kind {
                    LidarKind::Stl27L => {
                        labeled_edit(ui, "포트", &mut self.entries[i].port);
                        labeled_edit(ui, "Baud", &mut self.entries[i].baud);
                    }
                    LidarKind::Lds50CE => {
                        labeled_edit(ui, "센서 IP", &mut self.entries[i].sensor_ip);
                        labeled_edit(ui, "커맨드 포트", &mut self.entries[i].command_port);
                        labeled_edit(ui, "수신 IP", &mut self.entries[i].host_ip);
                        labeled_edit(ui, "수신 포트", &mut self.entries[i].host_port);
                    }
                    LidarKind::Demo => {
                        ui.label(egui::RichText::new("하드웨어 없이 합성 데이터").weak());
                    }
                }
            });
        }
    }

    /// 상태 목록: 장치별 연결 상태(색 점)·엔드포인트·점 개수. Start 전이면 안내만.
    fn status_ui(&mut self, ui: &mut egui::Ui) {
        if !self.running {
            ui.label(egui::RichText::new("Start를 누르면 위 설정대로 연결합니다.").weak());
            return;
        }

        for (i, feed) in self.feeds.iter().enumerate() {
            let status = feed.status();
            let (dot, _) = status_indicator(&status, "");
            let points = self.scans.get(i).map(|s| s.points.len()).unwrap_or(0);
            ui.horizontal(|ui| {
                ui.colored_label(palette::DEVICE[i % palette::DEVICE.len()], "■");
                ui.colored_label(dot, "●");
                ui.label(format!(
                    "#{} {} · {} · {}pts",
                    i + 1,
                    self.entries.get(i).map(|e| e.kind.label()).unwrap_or(""),
                    self.entries
                        .get(i)
                        .map(|e| e.endpoint())
                        .unwrap_or_default(),
                    points
                ));
            });
            if let ConnectionStatus::Error(reason) = &status {
                ui.label(egui::RichText::new(format!("   ↳ {reason}")).weak().small());
            }
        }
    }
}

/// "라벨 [텍스트필드]" 한 줄.
fn labeled_edit(ui: &mut egui::Ui, label: &str, value: &mut String) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::TextEdit::singleline(value).desired_width(150.0));
    });
}

/// 연결 상태를 (표시 색, 설명 문구)로 변환한다.
fn status_indicator(status: &ConnectionStatus, endpoint: &str) -> (egui::Color32, String) {
    match status {
        ConnectionStatus::Connected => (
            egui::Color32::from_rgb(0x34, 0xd3, 0x99),
            format!("연결됨 · {endpoint}"),
        ),
        ConnectionStatus::Connecting => (
            egui::Color32::from_rgb(0xfb, 0xbf, 0x24),
            format!("연결 중… · {endpoint}"),
        ),
        ConnectionStatus::Error(reason) => (
            egui::Color32::from_rgb(0xf8, 0x71, 0x71),
            format!("오류: {reason}"),
        ),
        ConnectionStatus::Demo => (
            egui::Color32::from_rgb(0x60, 0xa5, 0xfa),
            "데모 데이터".to_string(),
        ),
    }
}

/// 중앙 플롯: 그리드·좌표축·커서 좌표는 egui_plot이 내장 제공한다. 고정 범위로 흔들림 제거.
fn draw_plot(
    ui: &mut egui::Ui,
    config: &ViewerConfig,
    devices: &[(Vec<CartesianPoint>, egui::Color32)],
) {
    let r = (config.scan.max_range_m as f64) * 1.05;
    Plot::new("lidar")
        .data_aspect(1.0) // x:y = 1:1 → 거리 왜곡 없음
        .show_grid(true)
        .show_axes(true)
        .auto_bounds(false) // 데이터에 맞춰 범위 재조정 금지 → 흔들림 제거
        .default_x_bounds(-r, r)
        .default_y_bounds(-r, r)
        .coordinates_formatter(
            Corner::LeftBottom,
            CoordinatesFormatter::new(|p, _bounds| format!("x: {:.2} m\ny: {:.2} m", p.x, p.y)),
        )
        .show(ui, |plot_ui| {
            if config.scan.show_range_rings {
                draw_range_rings(plot_ui, config.scan.max_range_m);
            }
            for (index, (points, color)) in devices.iter().enumerate() {
                draw_points(plot_ui, config, points, *color, index);
            }
        });
}

/// 한 장치의 점 구름을 그린다. `color_by_intensity`면 세기별로 밝기를 나눠 그린다.
fn draw_points(
    plot_ui: &mut egui_plot::PlotUi,
    config: &ViewerConfig,
    points: &[CartesianPoint],
    base: egui::Color32,
    device_index: usize,
) {
    let radius = config.points.size;

    if !config.points.color_by_intensity {
        let series: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
        plot_ui.points(
            Points::new(format!("dev{device_index}"), series)
                .radius(radius)
                .color(base)
                .shape(MarkerShape::Circle),
        );
        return;
    }

    // 세기를 INTENSITY_TIERS단계로 나눠 각 단계를 다른 밝기로 그린다.
    let mut tiers: [Vec<[f64; 2]>; INTENSITY_TIERS] = std::array::from_fn(|_| Vec::new());
    for p in points {
        let tier = (p.intensity as usize * INTENSITY_TIERS / 256).min(INTENSITY_TIERS - 1);
        tiers[tier].push([p.x, p.y]);
    }

    for (index, series) in tiers.into_iter().enumerate() {
        if series.is_empty() {
            continue;
        }
        let factor = 0.35 + 0.65 * (index as f32 / (INTENSITY_TIERS - 1) as f32);
        plot_ui.points(
            Points::new("", series)
                .radius(radius)
                .color(scale_brightness(base, factor))
                .shape(MarkerShape::Circle),
        );
    }
}

/// 1m 간격 동심원(거리 링)과 거리 라벨을 그린다.
fn draw_range_rings(plot_ui: &mut egui_plot::PlotUi, max_range_m: f32) {
    let max_ring = max_range_m.floor() as i32;
    for r in 1..=max_ring {
        let radius = r as f64;
        let circle: Vec<[f64; 2]> = (0..=72)
            .map(|step| {
                let theta = step as f64 / 72.0 * std::f64::consts::TAU;
                [radius * theta.cos(), radius * theta.sin()]
            })
            .collect();

        plot_ui.line(
            Line::new(format!("{r}m"), PlotPoints::from(circle))
                .color(palette::RING)
                .width(1.0),
        );
        plot_ui.text(
            Text::new("", PlotPoint::new(0.0, radius), format!("{r} m")).color(palette::RING_LABEL),
        );
    }
}

/// 색의 RGB에 밝기 계수를 곱한다(알파는 유지).
fn scale_brightness(color: egui::Color32, factor: f32) -> egui::Color32 {
    let scale = |v: u8| (v as f32 * factor).round().clamp(0.0, 255.0) as u8;
    egui::Color32::from_rgba_unmultiplied(
        scale(color.r()),
        scale(color.g()),
        scale(color.b()),
        color.a(),
    )
}
