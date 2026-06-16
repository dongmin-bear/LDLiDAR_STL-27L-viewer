//! 측정점 공급원. 별도 스레드에서 시리얼 LiDAR를 읽어 채널로 흘려보낸다.
//!
//! 연결 상태([`ConnectionStatus`])를 공유해 UI가 실제 상태(연결됨 / 연결 중 /
//! 오류)를 그대로 보여줄 수 있게 한다. 실 포트 모드에서 열기·읽기에 실패하면
//! 데모로 조용히 바꾸지 않고 오류를 표면화한 뒤 주기적으로 재연결을 시도한다.
//! (데모 데이터는 `demo = true`로 명시했을 때만 쓴다.)

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use serialport::{DataBits, FlowControl, Parity, StopBits};

use crate::{FrameDecoder, LidarPoint};

const READ_TIMEOUT_MS: u64 = 100;
const SERIAL_BUF: usize = 512;
/// 연결 실패 시 재시도 간격.
const RETRY_DELAY: Duration = Duration::from_secs(1);
/// 포트를 연 뒤 어댑터/장치가 안정될 때까지 잠깐 대기(재연결 직후 깨진 바이트 회피).
const OPEN_SETTLE: Duration = Duration::from_millis(120);
/// 연결은 됐는데 유효 프레임이 이 시간 동안 안 오면 "나쁜 상태"로 보고 끊어 재연결한다.
/// (정상 연결은 매 read마다 프레임이 오므로 절대 걸리지 않는다.)
const DATA_TIMEOUT: Duration = Duration::from_millis(1500);

/// 공급원 종류.
pub enum Source {
    /// 지정한 포트/속도의 실제 LiDAR. 실패하면 재연결을 시도한다(데모로 안 바꿈).
    Serial { port: String, baud: u32 },
    /// 합성 데이터(가상의 사각형 방).
    Demo,
}

/// 현재 연결 상태. UI가 그대로 표시한다.
#[derive(Clone, Debug)]
pub enum ConnectionStatus {
    /// 포트 여는 중 / 데이터 대기 중.
    Connecting,
    /// 정상 연결됨.
    Connected,
    /// 열기·읽기 오류(사유 포함). 곧 재연결을 시도한다.
    Error(String),
    /// 데모 데이터로 동작 중.
    Demo,
}

/// 스레드와 UI가 공유하는 연결 상태 핸들.
pub type SharedStatus = Arc<Mutex<ConnectionStatus>>;

/// 공급원 핸들: 측정점 채널 + 공유 연결 상태.
pub struct Feed {
    pub points: Receiver<LidarPoint>,
    pub status: SharedStatus,
}

/// 공급원을 백그라운드 스레드로 띄운다. 수신 측(앱)이 사라지면 스레드는
/// 다음 전송에서 자연히 종료된다.
pub fn spawn(source: Source) -> Feed {
    let (tx, rx) = mpsc::channel();
    let status = Arc::new(Mutex::new(ConnectionStatus::Connecting));
    let status_thread = Arc::clone(&status);

    thread::spawn(move || match source {
        Source::Serial { port, baud } => run_serial(&port, baud, &tx, &status_thread),
        Source::Demo => {
            set_status(&status_thread, ConnectionStatus::Demo);
            run_demo(&tx);
        }
    });

    Feed { points: rx, status }
}

/// 공유 상태를 갱신한다(락 실패는 무시).
fn set_status(status: &SharedStatus, value: ConnectionStatus) {
    if let Ok(mut guard) = status.lock() {
        *guard = value;
    }
}

/// 시리얼 포트에서 프레임을 읽어 점을 전송. 실패하면 상태를 오류로 알리고
/// `RETRY_DELAY` 뒤 다시 연결을 시도한다.
fn run_serial(port_name: &str, baud: u32, tx: &Sender<LidarPoint>, status: &SharedStatus) {
    loop {
        set_status(status, ConnectionStatus::Connecting);

        let opened = serialport::new(port_name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_millis(READ_TIMEOUT_MS))
            .open();

        match opened {
            Ok(mut port) => {
                eprintln!("[viewer] {port_name} 연결됨 — {baud} 8N1");
                // 재연결 직후 어댑터가 안정될 시간을 준 뒤, 그동안 쌓인 묵은/조각
                // 바이트를 비워 디코더가 깨끗한 프레임 경계부터 시작하게 한다.
                thread::sleep(OPEN_SETTLE);
                let _ = port.clear(serialport::ClearBuffer::Input);
                // 링크는 열렸지만 유효 프레임을 받기 전까지는 "연결 중"으로 둔다.
                // (정상 데이터가 들어오면 pump_serial이 Connected로 바꾼다.)
                set_status(status, ConnectionStatus::Connecting);
                // 읽기 루프가 끝났다는 건 채널 종료(앱 종료) 또는 읽기 오류/무데이터.
                if pump_serial(&mut *port, tx, status) {
                    return; // 앱 종료.
                }
            }
            Err(error) => {
                let reason = format!("{port_name}: {error}");
                eprintln!(
                    "[viewer] 열기 실패 — {reason} ({}초 후 재시도)",
                    RETRY_DELAY.as_secs()
                );
                set_status(status, ConnectionStatus::Error(reason));
            }
        }

        thread::sleep(RETRY_DELAY);
    }
}

/// 열린 포트에서 점을 계속 읽어 전송한다.
/// 반환값 `true` = 채널이 닫혀 앱이 종료됨(스레드도 끝내야 함),
/// `false` = 읽기 오류로 재연결이 필요함.
///
/// debug 빌드(또는 `LIDAR_DEBUG` 환경변수)에서는 raw 바이트 수신·프레임 파싱
/// 결과를 로그로 남겨, 데이터가 들어오는지/파싱되는지 눈으로 확인할 수 있다.
fn pump_serial(
    port: &mut dyn serialport::SerialPort,
    tx: &Sender<LidarPoint>,
    status: &SharedStatus,
) -> bool {
    let debug = debug_enabled();
    let mut decoder = FrameDecoder::new();
    let mut buffer = [0_u8; SERIAL_BUF];
    let mut stats = DecodeStats::default();
    let mut last_report = Instant::now();
    let mut last_frame_at = Instant::now();
    let mut connected = false;

    loop {
        match port.read(&mut buffer) {
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
                        if tx.send(point).is_err() {
                            return true; // 수신 측 종료.
                        }
                    }
                }

                if debug && last_report.elapsed() >= Duration::from_secs(1) {
                    stats.report();
                    stats.reset_interval();
                    last_report = Instant::now();
                }
            }
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {}
            Err(error) => {
                let reason = format!("읽기 오류: {error}");
                eprintln!("[viewer] {reason} — 재연결합니다.");
                set_status(status, ConnectionStatus::Error(reason));
                return false; // 재연결 필요.
            }
        }

        // 워치독: 링크는 살아있는데 유효 프레임이 한동안 없으면(어댑터가 나쁜 상태로
        // 올라온 경우) 스스로 끊고 재연결한다. 정상 연결은 매번 프레임이 오므로 안 걸린다.
        if last_frame_at.elapsed() > DATA_TIMEOUT {
            eprintln!(
                "[viewer] {}ms 동안 유효 프레임 없음 — 재연결합니다.",
                DATA_TIMEOUT.as_millis()
            );
            set_status(status, ConnectionStatus::Error("유효 데이터 없음".to_string()));
            return false;
        }
    }
}

/// debug 빌드이거나 `LIDAR_DEBUG`가 설정돼 있으면 디버그 로그를 켠다.
pub(crate) fn debug_enabled() -> bool {
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
///
/// `ok>0`이면 정상. `ok=0`일 때 `crc_failures>0`이면 헤더는 잡히는데 CRC가 깨지는
/// 것(거의 맞음 — 비트오류/근접 baud), `crc_failures=0`이고 `skipped`만 크면 헤더(0x54)
/// 자체가 없는 것(완전히 다른 baud/포맷)을 뜻한다.
#[derive(Default)]
struct DecodeStats {
    bytes: usize,
    ok: usize,
    /// 동기화 못 잡아 버린 바이트.
    skipped: usize,
    /// 헤더는 맞췄으나 CRC 불일치로 버린 횟수.
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

    /// 1초 카운터만 리셋(최초 로그 플래그는 유지).
    fn reset_interval(&mut self) {
        self.bytes = 0;
        self.ok = 0;
        self.skipped = 0;
        self.crc_failures = 0;
    }
}

/// 가상의 사각형 방 벽을 스캔하는 합성 데이터. 중심에서 각도를 연속으로 쓸면서
/// 벽까지의 거리를 광선-사각형 교차로 계산해 보낸다.
fn run_demo(tx: &Sender<LidarPoint>) {
    // 방 반치수(m)와 약간의 측정 노이즈.
    const HALF_X: f32 = 4.0;
    const HALF_Y: f32 = 3.0;
    const STEP_DEG: f32 = 0.5;
    const BATCH: usize = 12; // 실제 패킷처럼 한 번에 12점씩.

    let mut angle_deg = 0.0_f32;
    let mut noise = 0.0_f32;

    loop {
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
            if tx.send(point).is_err() {
                return;
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

    #[test]
    fn ray_along_x_hits_side_wall() {
        // 0° → +x 벽(hx)에 닿아야 한다.
        assert!((ray_to_box(0.0, 4.0, 3.0) - 4.0).abs() < 1e-3);
    }

    #[test]
    fn ray_along_y_hits_top_wall() {
        // 90° → +y 벽(hy)에 닿아야 한다.
        let d = ray_to_box(std::f32::consts::FRAC_PI_2, 4.0, 3.0);
        assert!((d - 3.0).abs() < 1e-3);
    }
}
