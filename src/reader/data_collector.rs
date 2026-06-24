//! 데이터 수집기(data collector). 별도 스레드에서 LiDAR를 읽어 프레임을 파싱하고,
//! 점들을 **한 바퀴(rotation) 단위 [`Scan`]으로 재구성**해 "최신 한 장" 슬롯에 publish한다.
//! 뷰어는 이 슬롯에서 최신 스캔을 **pull**해서 그리기만 한다.
//!
//! 모델·전송 독립: 전송은 [`Transport`](시리얼/UDP), 코덱은 [`Decoder`](모델별)로 가려져
//! 있어, 이 수집 루프([`pump`])는 둘 다 몰라도 된다. 새 모델은 그 두 트레잇만 구현하면
//! 이 파이프라인(재연결·회전 조립·publish·워치독)을 그대로 재사용한다.
//!
//! 책임 경계:
//! - 이 모듈(수집): I/O · 재연결 · 프레임 디코딩 · 회전 조립. 결과는 표현과 무관한
//!   극좌표 [`Scan`].
//! - 뷰어(표현): [`ScanFeed`]에서 최신 [`Scan`]을 받아 직교좌표로 변환해 그리기만.
//!
//! 중간 계층은 FIFO 큐가 아니라 **최신-스냅샷(latest-wins) 슬롯**이다. 실시간 뷰어는
//! 항상 "가장 최근 한 바퀴"만 필요하므로, 소비가 늦어도 lag이 쌓이지 않는다.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use serialport::{DataBits, FlowControl, Parity, StopBits};

use super::model::{Decoder, Model};
use super::pacecat::lds50ce::command as pacecat_command;
use super::transport::{SerialTransport, Transport, UdpTransport};
use super::types::LidarPoint;

const READ_TIMEOUT_MS: u64 = 100;
const SERIAL_BUF: usize = 512;
/// UDP 데이터그램 한 개를 담을 버퍼(LDS-50C-E 패킷은 ~500B, 넉넉히 잡는다).
const UDP_BUF: usize = 2048;
/// 연결 실패 시 재시도 간격.
const RETRY_DELAY: Duration = Duration::from_secs(1);
/// 포트를 연 뒤 어댑터/장치가 안정될 때까지 잠깐 대기(재연결 직후 깨진 바이트 회피).
const OPEN_SETTLE: Duration = Duration::from_millis(120);
/// 연결은 됐는데 유효 프레임이 이 시간 동안 안 오면 "나쁜 상태"로 보고 끊어 재연결한다.
const DATA_TIMEOUT: Duration = Duration::from_millis(1500);

/// 각도가 이만큼(도) 줄어들면 한 바퀴가 끝난 것(wrap)으로 본다. 측정 노이즈로 인한
/// 소폭 후진과 진짜 회전 경계(≈360°)를 구분하려고 절반 회전을 임계로 둔다.
const WRAP_THRESHOLD_DEG: f32 = 180.0;
/// wrap이 안 잡히는 비정상 스트림에서 메모리 무한 증가를 막는 안전 상한.
const MAX_SCAN_POINTS: usize = 20_000;

/// 공급원 종류. 모델과 전송 파라미터를 함께 들고 있다.
pub enum Source {
    /// 시리얼 LiDAR(예: STL-27L). 실패하면 재연결을 시도한다(데모로 안 바꿈).
    Serial {
        model: Model,
        port: String,
        baud: u32,
    },
    /// UDP LiDAR(예: LDS-50C-E). 연결 시 시작 커맨드를 보내 스트리밍을 켠다.
    Udp {
        model: Model,
        sensor_ip: String,
        command_port: u16,
        host_ip: String,
        host_port: u16,
    },
    /// 합성 데이터(가상의 사각형 방).
    Demo,
}

/// 현재 연결 상태. UI가 그대로 표시한다.
#[derive(Clone, Debug)]
pub enum ConnectionStatus {
    /// 포트/소켓 여는 중 / 데이터 대기 중.
    Connecting,
    /// 정상 연결됨.
    Connected,
    /// 열기·읽기 오류(사유 포함). 곧 재연결을 시도한다.
    Error(String),
    /// 데모 데이터로 동작 중.
    Demo,
}

/// 한 바퀴(rotation)로 재구성된 측정 스냅샷. 극좌표 그대로 — 표현(렌더링)과 무관.
#[derive(Debug, Clone, Default)]
pub struct Scan {
    /// 이 회전의 점들(각도°, 거리mm, 세기). 거리 0(무효)은 이미 제외돼 있다.
    pub points: Vec<LidarPoint>,
    /// 0부터 증가하는 회전 인덱스(새 스캔 도착 감지·진단용).
    pub rotation: u64,
}

/// 스레드와 UI가 공유하는 연결 상태 핸들.
type SharedStatus = Arc<Mutex<ConnectionStatus>>;
/// "최신 한 장" 스캔 슬롯(latest-wins). 생산자가 덮어쓰고 소비자가 꺼내 간다.
type ScanSlot = Arc<Mutex<Option<Scan>>>;

/// 수집기 핸들: 최신 스캔 슬롯 + 공유 연결 상태 + 정지 플래그.
/// 뷰어는 이것으로 데이터를 pull하고, 필요하면 [`ScanFeed::stop`]으로 스레드를 멈춘다.
pub struct ScanFeed {
    latest: ScanSlot,
    status: SharedStatus,
    running: Arc<AtomicBool>,
}

impl ScanFeed {
    /// 최신 스캔이 있으면 꺼내 온다(없으면 `None`). 꺼내면 슬롯은 비워지므로,
    /// 새 스캔이 도착하기 전까지 호출자는 직전 스캔을 그대로 들고 있으면 된다.
    pub fn take_latest(&self) -> Option<Scan> {
        self.latest.lock().ok().and_then(|mut guard| guard.take())
    }

    /// 현재 연결 상태 사본.
    pub fn status(&self) -> ConnectionStatus {
        self.status
            .lock()
            .map(|guard| guard.clone())
            .unwrap_or(ConnectionStatus::Connecting)
    }

    /// 수집 스레드에 정지를 요청한다. 스레드는 다음 루프(≤읽기 타임아웃)에서 빠져나간다.
    /// 같은 포트/소켓을 다시 열기 전(재구성·재시작)에 반드시 호출해 자원 충돌을 막는다.
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

impl Drop for ScanFeed {
    /// 핸들이 사라지면 스레드도 자동으로 멈춘다(누수 방지).
    fn drop(&mut self) {
        self.stop();
    }
}

/// 수집기를 백그라운드 스레드로 띄운다. 프로세스가 끝나면 스레드도 함께 정리된다.
pub fn spawn(source: Source) -> ScanFeed {
    let latest: ScanSlot = Arc::new(Mutex::new(None));
    let status: SharedStatus = Arc::new(Mutex::new(ConnectionStatus::Connecting));
    let running = Arc::new(AtomicBool::new(true));
    let latest_thread = Arc::clone(&latest);
    let status_thread = Arc::clone(&status);
    let running_thread = Arc::clone(&running);

    thread::spawn(move || match source {
        Source::Serial { model, port, baud } => run_serial(
            model,
            &port,
            baud,
            &latest_thread,
            &status_thread,
            &running_thread,
        ),
        Source::Udp {
            model,
            sensor_ip,
            command_port,
            host_ip,
            host_port,
        } => run_udp(
            model,
            &sensor_ip,
            command_port,
            &host_ip,
            host_port,
            &latest_thread,
            &status_thread,
            &running_thread,
        ),
        Source::Demo => {
            set_status(&status_thread, ConnectionStatus::Demo);
            run_demo(&latest_thread, &running_thread);
        }
    });

    ScanFeed {
        latest,
        status,
        running,
    }
}

/// 공유 상태를 갱신한다(락 실패는 무시).
fn set_status(status: &SharedStatus, value: ConnectionStatus) {
    if let Ok(mut guard) = status.lock() {
        *guard = value;
    }
}

/// 완성된 한 바퀴를 최신 슬롯에 publish한다(직전 미소비분은 버려짐 = latest-wins).
fn publish(slot: &ScanSlot, scan: Scan) {
    if let Ok(mut guard) = slot.lock() {
        *guard = Some(scan);
    }
}

/// 점 스트림을 한 바퀴(rotation) 단위로 재구성하는 조립기.
///
/// 각도가 단조 증가하다가 절반 바퀴(180°)를 넘게 되감기면 0°를 막 지난 것 = 한 바퀴
/// 완성으로 본다. 완성되면 모인 점들을 통째로 돌려주고 새 회전을 시작한다.
struct Assembler {
    current: Vec<LidarPoint>,
    last_angle: Option<f32>,
}

impl Assembler {
    fn new() -> Self {
        Self {
            current: Vec::new(),
            last_angle: None,
        }
    }

    /// 점 하나를 누적한다. 거리 0(무효)은 버린다. 이번 점으로 한 바퀴가 **완성됐으면**
    /// 직전 회전의 점들(`Some(points)`)을, 아니면 `None`을 돌려준다.
    fn ingest(&mut self, point: LidarPoint) -> Option<Vec<LidarPoint>> {
        if point.distance_mm == 0 {
            return None;
        }
        let angle = point.angle_degrees.rem_euclid(360.0);

        let wrapped = self
            .last_angle
            .is_some_and(|prev| prev - angle > WRAP_THRESHOLD_DEG);
        let completed = if wrapped || self.current.len() >= MAX_SCAN_POINTS {
            Some(std::mem::take(&mut self.current))
        } else {
            None
        };

        self.last_angle = Some(angle);
        self.current.push(LidarPoint {
            angle_degrees: angle,
            ..point
        });
        completed
    }
}

/// 시리얼 포트를 열어 프레임을 읽는다. 실패하면 상태를 오류로 알리고 재시도한다.
fn run_serial(
    model: Model,
    port_name: &str,
    baud: u32,
    slot: &ScanSlot,
    status: &SharedStatus,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Relaxed) {
        set_status(status, ConnectionStatus::Connecting);

        let opened = serialport::new(port_name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_millis(READ_TIMEOUT_MS))
            .open();

        match opened {
            Ok(port) => {
                eprintln!(
                    "[collector] {} {port_name} 연결됨 — {baud} 8N1",
                    model.label()
                );
                // 재연결 직후 어댑터가 안정될 시간을 준 뒤, 묵은/조각 바이트를 비워
                // 디코더가 깨끗한 프레임 경계부터 시작하게 한다.
                thread::sleep(OPEN_SETTLE);
                let _ = port.clear(serialport::ClearBuffer::Input);
                set_status(status, ConnectionStatus::Connecting);

                let mut transport = SerialTransport::new(port);
                let mut decoder = model.new_decoder();
                pump(
                    &mut transport,
                    decoder.as_mut(),
                    slot,
                    status,
                    SERIAL_BUF,
                    running,
                );
            }
            Err(error) => {
                let reason = format!("{port_name}: {error}");
                eprintln!(
                    "[collector] 열기 실패 — {reason} ({}초 후 재시도)",
                    RETRY_DELAY.as_secs()
                );
                set_status(status, ConnectionStatus::Error(reason));
            }
        }

        sleep_interruptible(RETRY_DELAY, running);
    }
}

/// UDP 소켓을 열고 시작 커맨드를 보낸 뒤 데이터를 읽는다. 실패하면 재연결한다.
#[allow(clippy::too_many_arguments)]
fn run_udp(
    model: Model,
    sensor_ip: &str,
    command_port: u16,
    host_ip: &str,
    host_port: u16,
    slot: &ScanSlot,
    status: &SharedStatus,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Relaxed) {
        set_status(status, ConnectionStatus::Connecting);

        let connected = pacecat_command::connect(sensor_ip, command_port, host_ip, host_port)
            .and_then(|socket| UdpTransport::new(socket, Duration::from_millis(READ_TIMEOUT_MS)));

        match connected {
            Ok(mut transport) => {
                eprintln!(
                    "[collector] {} UDP {host_ip}:{host_port} 수신 — 센서 {sensor_ip}:{command_port}",
                    model.label()
                );
                set_status(status, ConnectionStatus::Connecting);
                let mut decoder = model.new_decoder();
                pump(
                    &mut transport,
                    decoder.as_mut(),
                    slot,
                    status,
                    UDP_BUF,
                    running,
                );
            }
            Err(error) => {
                let reason = format!("UDP {host_ip}:{host_port}: {error}");
                eprintln!(
                    "[collector] 열기 실패 — {reason} ({}초 후 재시도)",
                    RETRY_DELAY.as_secs()
                );
                set_status(status, ConnectionStatus::Error(reason));
            }
        }

        sleep_interruptible(RETRY_DELAY, running);
    }
}

/// 정지 요청에 빠르게 반응하도록 짧게 끊어 자는 sleep.
fn sleep_interruptible(total: Duration, running: &Arc<AtomicBool>) {
    let step = Duration::from_millis(50);
    let mut slept = Duration::ZERO;
    while slept < total && running.load(Ordering::Relaxed) {
        thread::sleep(step);
        slept += step;
    }
}

/// 열린 전송에서 바이트를 읽어 프레임→점→스캔으로 재구성해 publish한다.
/// 읽기 오류나 무데이터 워치독에 걸리면 반환해 재연결하게 한다.
///
/// 전송([`Transport`])·코덱([`Decoder`])에 무관한 공용 루프다. debug 빌드(또는
/// `LIDAR_DEBUG`)에서는 수신·파싱·완성 통계를 로그로 남긴다.
fn pump(
    transport: &mut dyn Transport,
    decoder: &mut dyn Decoder,
    slot: &ScanSlot,
    status: &SharedStatus,
    buf_size: usize,
    running: &Arc<AtomicBool>,
) {
    let debug = debug_enabled();
    let mut assembler = Assembler::new();
    let mut buffer = vec![0_u8; buf_size];
    let mut stats = DecodeStats::default();
    let mut last_report = Instant::now();
    let mut last_frame_at = Instant::now();
    let mut connected = false;
    let mut rotation = 0_u64;

    while running.load(Ordering::Relaxed) {
        match transport.read(&mut buffer) {
            Ok(read) if read > 0 => {
                if debug {
                    stats.bytes += read;
                    if !stats.logged_first_bytes {
                        let dump = read.min(160);
                        eprintln!("[debug] 첫 수신 {read}B — 앞 {dump}B 덤프 (헤더 패턴 확인용):");
                        log_hex_dump(&buffer[..dump]);
                        stats.logged_first_bytes = true;
                    }
                }

                let frames = decoder.push_bytes(&buffer[..read]);
                if debug {
                    stats.skipped += decoder.last_skipped();
                    stats.crc_failures += decoder.last_crc_failures();
                }

                if !frames.is_empty() {
                    last_frame_at = Instant::now();
                    if !connected {
                        connected = true;
                        set_status(status, ConnectionStatus::Connected);
                    }
                }

                for body in frames {
                    if debug {
                        stats.ok += 1;
                        if !stats.logged_first_frame {
                            eprintln!(
                                "[debug] 첫 프레임 파싱 OK: speed={}deg/s start={:.2} end={:.2} pts={}",
                                body.speed_degrees_per_second,
                                body.start_angle_degrees,
                                body.end_angle_degrees,
                                body.points.len()
                            );
                            stats.logged_first_frame = true;
                        }
                    }
                    for point in body.points {
                        if let Some(points) = assembler.ingest(point) {
                            if debug {
                                eprintln!("[debug] 완성 스캔 #{rotation}: {} points", points.len());
                            }
                            publish(slot, Scan { points, rotation });
                            rotation += 1;
                        }
                    }
                }

                if debug && last_report.elapsed() >= Duration::from_secs(1) {
                    stats.report();
                    stats.reset_interval();
                    last_report = Instant::now();
                }
            }
            // 데이터 없음(타임아웃 포함) → 워치독만 확인하고 계속.
            Ok(_) => {}
            Err(error) => {
                let reason = format!("읽기 오류: {error}");
                eprintln!("[collector] {reason} — 재연결합니다.");
                set_status(status, ConnectionStatus::Error(reason));
                return; // 재연결 필요.
            }
        }

        // 워치독: 링크는 살아있는데 유효 프레임이 한동안 없으면 스스로 끊고 재연결한다.
        if last_frame_at.elapsed() > DATA_TIMEOUT {
            eprintln!(
                "[collector] {}ms 동안 유효 프레임 없음 — 재연결합니다.",
                DATA_TIMEOUT.as_millis()
            );
            set_status(
                status,
                ConnectionStatus::Error("유효 데이터 없음".to_string()),
            );
            return;
        }
    }
}

/// debug 빌드이거나 `LIDAR_DEBUG`가 설정돼 있으면 디버그 로그를 켠다.
fn debug_enabled() -> bool {
    cfg!(debug_assertions) || std::env::var_os("LIDAR_DEBUG").is_some()
}

/// 바이트들을 한 줄 16바이트씩 오프셋과 함께 16진 덤프한다(포맷 분석용).
fn log_hex_dump(bytes: &[u8]) {
    for (line, chunk) in bytes.chunks(16).enumerate() {
        let hex = chunk
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<Vec<_>>()
            .join(" ");
        eprintln!("[debug]   {:04x}: {hex}", line * 16);
    }
}

/// 1초 단위 디코드 통계(디버그 전용).
#[derive(Default)]
struct DecodeStats {
    bytes: usize,
    ok: usize,
    skipped: usize,
    crc_failures: usize,
    logged_first_bytes: bool,
    logged_first_frame: bool,
}

impl DecodeStats {
    fn report(&self) {
        eprintln!(
            "[debug] 1초: {}B 수신 · 프레임 ok={} · 동기화 버림={}B · CRC불일치={}",
            self.bytes, self.ok, self.skipped, self.crc_failures
        );
    }

    fn reset_interval(&mut self) {
        self.bytes = 0;
        self.ok = 0;
        self.skipped = 0;
        self.crc_failures = 0;
    }
}

/// 가상의 사각형 방 벽을 스캔하는 합성 데이터. 중심에서 각도를 연속으로 쓸면서
/// 벽까지의 거리를 광선-사각형 교차로 계산해 스캔으로 재구성해 publish한다.
fn run_demo(slot: &ScanSlot, running: &Arc<AtomicBool>) {
    const HALF_X: f32 = 4.0;
    const HALF_Y: f32 = 3.0;
    const STEP_DEG: f32 = 0.5;
    const BATCH: usize = 12; // 실제 패킷처럼 한 번에 12점씩.

    let mut assembler = Assembler::new();
    let mut angle_deg = 0.0_f32;
    let mut noise = 0.0_f32;
    let mut rotation = 0_u64;

    while running.load(Ordering::Relaxed) {
        for _ in 0..BATCH {
            let angle_rad = angle_deg.to_radians();
            // 결정적이고 가벼운 의사 노이즈(±25mm).
            noise = (noise + 0.137).fract();
            let jitter = (noise - 0.5) * 0.05;
            let distance_m = (ray_to_box(angle_rad, HALF_X, HALF_Y) + jitter).max(0.0);
            let intensity = (120.0 + 100.0 * angle_rad.sin().abs()) as u8;

            let point = LidarPoint {
                angle_degrees: angle_deg,
                distance_mm: (distance_m * 1000.0) as u16,
                intensity,
            };
            if let Some(points) = assembler.ingest(point) {
                publish(slot, Scan { points, rotation });
                rotation += 1;
            }
            angle_deg = (angle_deg + STEP_DEG).rem_euclid(360.0);
        }
        // 실제 스캔 속도와 비슷하게 약간 쉰다.
        thread::sleep(Duration::from_millis(2));
    }
}

/// 원점에서 각도 `angle_rad`로 쏜 광선이 중심 정렬 사각형(반치수 hx,hy) 벽에
/// 닿는 거리(m). 내부에서 출발하므로 항상 양의 해가 존재한다.
fn ray_to_box(angle_rad: f32, hx: f32, hy: f32) -> f32 {
    let (sin, cos) = angle_rad.sin_cos();
    let tx = if cos.abs() < 1e-6 {
        f32::INFINITY
    } else {
        hx / cos.abs()
    };
    let ty = if sin.abs() < 1e-6 {
        f32::INFINITY
    } else {
        hy / sin.abs()
    };
    tx.min(ty)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(angle: f32, distance_mm: u16, intensity: u8) -> LidarPoint {
        LidarPoint {
            angle_degrees: angle,
            distance_mm,
            intensity,
        }
    }

    #[test]
    fn no_scan_before_first_full_rotation() {
        let mut asm = Assembler::new();
        for a in [0.0, 90.0, 180.0, 270.0] {
            assert!(asm.ingest(pt(a, 1000, 10)).is_none());
        }
    }

    #[test]
    fn wrap_finalizes_previous_rotation() {
        let mut asm = Assembler::new();
        for a in [0.0, 100.0, 200.0, 300.0] {
            asm.ingest(pt(a, 1000, 10));
        }
        let done = asm.ingest(pt(10.0, 1000, 10)); // 300→10 : 290°>180 → wrap
        assert_eq!(done.expect("rotation completed").len(), 4);
    }

    #[test]
    fn zero_distance_is_discarded() {
        let mut asm = Assembler::new();
        asm.ingest(pt(45.0, 0, 100)); // 버려짐
        for a in [100.0, 200.0, 300.0] {
            asm.ingest(pt(a, 1000, 10));
        }
        let done = asm.ingest(pt(10.0, 1000, 10)).expect("wrap");
        assert_eq!(done.len(), 3); // 45°/0mm 점은 제외
    }

    #[test]
    fn ray_along_x_hits_side_wall() {
        assert!((ray_to_box(0.0, 4.0, 3.0) - 4.0).abs() < 1e-3);
    }

    #[test]
    fn ray_along_y_hits_top_wall() {
        let d = ray_to_box(std::f32::consts::FRAC_PI_2, 4.0, 3.0);
        assert!((d - 3.0).abs() < 1e-3);
    }
}
