//! egui_plot 기반 2D LiDAR 포인트 뷰어.
//!
//! - 그리드 + 좌표축 + 커서 좌표 표시(egui_plot 내장)
//! - 마우스 휠 확대/축소, 드래그 팬
//! - `config.toml` 핫리로드로 점 크기·색·표시 옵션 실시간 변경
//!
//! 데이터 흐름: [`source`] 스레드 → 채널 → [`scan::ScanBuffer`] 누적 → egui 렌더링.

pub mod config;
pub mod scan;
pub mod source;

use std::path::PathBuf;

use eframe::egui;
use egui_plot::{
    CoordinatesFormatter, Corner, Line, MarkerShape, Plot, PlotPoint, PlotPoints, Points, Text,
};
use notify::RecommendedWatcher;

use crate::LidarPoint;
use config::ViewerConfig;
use scan::{CartesianPoint, ScanBuffer};
use source::{ConnectionStatus, Feed, SharedStatus};

/// 의도별 색 코드를 한곳에 모아둔 팔레트.
mod palette {
    use eframe::egui::Color32;
    pub const RING: Color32 = Color32::from_rgb(60, 70, 80);
    pub const RING_LABEL: Color32 = Color32::from_rgb(120, 130, 140);
}

const INTENSITY_TIERS: usize = 5;

/// 뷰어를 실행한다. `config_path`가 없으면 기본 설정으로 새로 만든다.
pub fn run(config_path: PathBuf) -> anyhow::Result<()> {
    let initial = config::load_or_create(&config_path)?;

    // 데이터 공급원 선택: demo=true거나 포트가 없으면 합성 데이터.
    let src = if initial.lidar.demo {
        source::Source::Demo
    } else {
        source::Source::Serial {
            port: initial.lidar.port.clone(),
            baud: initial.lidar.baud,
        }
    };
    let feed = source::spawn(src);

    // 설정 파일 핫리로드 감시(워처 핸들은 앱이 살아있는 동안 보관해야 동작한다).
    let (config_rx, watcher) = config::watch(&config_path)?;

    let app = ViewerApp::new(initial, feed, config_rx, watcher);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1000.0, 760.0])
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
    // 비례·고정폭 두 패밀리 모두 CJK 폰트를 최우선으로 둔다.
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
    scan: ScanBuffer,
    points_rx: std::sync::mpsc::Receiver<LidarPoint>,
    status: SharedStatus,
    config_rx: std::sync::mpsc::Receiver<ViewerConfig>,
    /// notify 워처. Drop되면 감시가 멈추므로 필드로 살려둔다.
    _watcher: RecommendedWatcher,
}

impl ViewerApp {
    fn new(
        config: ViewerConfig,
        feed: Feed,
        config_rx: std::sync::mpsc::Receiver<ViewerConfig>,
        watcher: RecommendedWatcher,
    ) -> Self {
        let scan = ScanBuffer::new(config.angle_bins());
        Self {
            config,
            scan,
            points_rx: feed.points,
            status: feed.status,
            config_rx,
            _watcher: watcher,
        }
    }

    /// 핫리로드 채널에서 가장 최근 설정을 받아 반영한다.
    fn drain_config(&mut self) {
        let mut latest = None;
        while let Ok(config) = self.config_rx.try_recv() {
            latest = Some(config);
        }
        if let Some(config) = latest {
            self.scan.resize(config.angle_bins());
            self.config = config;
        }
    }

    /// 공급원 채널에 쌓인 점을 모두 스캔 버퍼에 반영한다.
    fn drain_points(&mut self) {
        while let Ok(point) = self.points_rx.try_recv() {
            self.scan.ingest(&point);
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_config();
        self.drain_points();

        let points = self
            .scan
            .cartesian_points(self.config.scan.max_range_m, self.config.decay());
        let status = self
            .status
            .lock()
            .map(|guard| guard.clone())
            .unwrap_or(ConnectionStatus::Connecting);

        egui::Panel::left("info")
            .resizable(false)
            .default_size(220.0)
            .show_inside(ui, |ui| side_panel(ui, &self.config, &status, points.len()));

        egui::CentralPanel::default().show_inside(ui, |ui| draw_plot(ui, &self.config, &points));

        // 데이터가 계속 흐르므로 매 프레임 다시 그린다.
        ui.ctx().request_repaint();
    }
}

/// 좌측 정보 패널: 연결 상태·설정값·점 개수를 보여주고 편집 방법을 안내한다.
fn side_panel(
    ui: &mut egui::Ui,
    config: &ViewerConfig,
    status: &ConnectionStatus,
    point_count: usize,
) {
    ui.add_space(8.0);
    ui.heading("LiDAR Viewer");
    ui.separator();

    // 연결 상태를 색과 함께 가장 위에 강조.
    let (dot_color, text) = status_indicator(status, &config.lidar.port);
    ui.horizontal(|ui| {
        ui.colored_label(dot_color, "●");
        ui.label(text);
    });
    ui.separator();

    egui::Grid::new("config_grid")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("점 개수");
            ui.label(point_count.to_string());
            ui.end_row();

            ui.label("점 크기");
            ui.label(format!("{:.1} px", config.points.size));
            ui.end_row();

            ui.label("점 색");
            let (rect, _) = ui.allocate_exact_size(egui::vec2(40.0, 14.0), egui::Sense::hover());
            ui.painter().rect_filled(rect, 2.0, config.point_color());
            ui.end_row();

            ui.label("최대 거리");
            ui.label(format!("{:.1} m", config.scan.max_range_m));
            ui.end_row();

            ui.label("거리 링");
            ui.label(if config.scan.show_range_rings {
                "ON"
            } else {
                "OFF"
            });
            ui.end_row();

            ui.label("잔상(decay)");
            ui.label(match config.scan.decay_ms {
                0 => "무한".to_string(),
                ms => format!("{ms} ms"),
            });
            ui.end_row();

            ui.label("포트");
            ui.label(if config.lidar.demo {
                "(데모 모드)".to_string()
            } else {
                config.lidar.port.clone()
            });
            ui.end_row();

            ui.label("Baud");
            ui.label(config.lidar.baud.to_string());
            ui.end_row();
        });

    ui.separator();
    ui.label(egui::RichText::new("config.toml을 저장하면 즉시 반영됩니다.").weak());
    ui.label(egui::RichText::new("포트 변경: [lidar] port = \"/dev/ttyXXX\"").weak());
    ui.add_space(4.0);
    ui.label(egui::RichText::new("휠: 확대/축소 · 드래그: 이동").weak());
}

/// 연결 상태를 (표시 색, 설명 문구)로 변환한다.
fn status_indicator(status: &ConnectionStatus, port: &str) -> (egui::Color32, String) {
    match status {
        ConnectionStatus::Connected => (
            egui::Color32::from_rgb(0x34, 0xd3, 0x99),
            format!("연결됨 · {port}"),
        ),
        ConnectionStatus::Connecting => (
            egui::Color32::from_rgb(0xfb, 0xbf, 0x24),
            format!("연결 중… · {port}"),
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

/// 중앙 플롯: 그리드·좌표축·커서 좌표는 egui_plot이 내장 제공한다.
fn draw_plot(ui: &mut egui::Ui, config: &ViewerConfig, points: &[CartesianPoint]) {
    Plot::new("lidar")
        .data_aspect(1.0) // x:y = 1:1 → 거리 왜곡 없음
        .show_grid(true)
        .show_axes(true)
        .coordinates_formatter(
            Corner::LeftBottom,
            CoordinatesFormatter::new(|p, _bounds| format!("x: {:.2} m\ny: {:.2} m", p.x, p.y)),
        )
        .show(ui, |plot_ui| {
            if config.scan.show_range_rings {
                draw_range_rings(plot_ui, config.scan.max_range_m);
            }
            draw_points(plot_ui, config, points);
        });
}

/// 점 구름을 그린다. `color_by_intensity`면 신호 세기별로 밝기를 나눠 그린다.
fn draw_points(plot_ui: &mut egui_plot::PlotUi, config: &ViewerConfig, points: &[CartesianPoint]) {
    let base = config.point_color();
    let radius = config.points.size;

    if !config.points.color_by_intensity {
        let series: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
        plot_ui.points(
            Points::new("scan", series)
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
