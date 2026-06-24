//! LDS-50C-E 시작 커맨드 빌드와 UDP 연결·핸드셰이크.
//!
//! STL-27L은 포트를 열면 바로 데이터가 흐르지만, LDS-50C-E는 **센서에 시작 커맨드를
//! 보내야** 스트리밍이 시작된다. C++ 레퍼런스(`Parser/LidarModule/Sender.cpp`,
//! `Reader.cpp`)의 절차를 그대로 옮겼다:
//!
//! 1. 호스트가 데이터를 받을 UDP 소켓을 bind하고 heartbeat 멀티캐스트에 가입한다.
//! 2. 커맨드 포트로 `LVERSH`(버전 질의)를 보내고, 500ms 뒤 `LSTARH`(시작)를 보낸다.
//! 3. 커맨드는 8B 헤더 + 4바이트 정렬 페이로드 + STM32 CRC32(4B)로 구성된다.

use std::io;
use std::net::{Ipv4Addr, UdpSocket};
use std::thread;
use std::time::Duration;

/// 커맨드 헤더 시그니처 "LH".
const CMD_SIGN: u16 = 0x484C;
/// 질의(query) 커맨드 코드.
const CMD_QUERY: u16 = 0x0043;
/// heartbeat 멀티캐스트 그룹.
const HEARTBEAT_GROUP: Ipv4Addr = Ipv4Addr::new(225, 225, 225, 225);
/// 첫 커맨드와 둘째 커맨드 사이 대기(센서가 첫 커맨드를 처리할 시간).
const INTER_CMD_DELAY: Duration = Duration::from_millis(500);

/// STM32 하드웨어 CRC32(다항식 0x04C11DB7, 초기값 0xFFFFFFFF, 비반전 MSB-우선).
/// C++ `stm32crc`와 비트 단위로 동일하다. 입력은 리틀엔디언 u32 워드열.
fn stm32_crc(words: &[u32]) -> u32 {
    const POLY: u32 = 0x04C1_1DB7;
    let mut crc: u32 = 0xFFFF_FFFF;
    for &data in words {
        let mut xbit: u32 = 1 << 31;
        for _ in 0..32 {
            if crc & 0x8000_0000 != 0 {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
            if data & xbit != 0 {
                crc ^= POLY;
            }
            xbit >>= 1;
        }
    }
    crc
}

/// ASCII 커맨드(예: `"LSTARH"`)를 전송용 바이트열로 만든다.
///
/// 레이아웃: `[sign u16][cmd u16][sn u16][len u16]` + 4바이트 정렬 페이로드 + `[crc u32]`.
/// 모두 리틀엔디언. CRC는 헤더(2워드)+페이로드 전체를 덮는다.
pub fn build_command(cmd: &str, sn: u16) -> Vec<u8> {
    let payload = cmd.as_bytes();
    let padded_len = payload.len().div_ceil(4) * 4;

    let mut buf = Vec::with_capacity(8 + padded_len + 4);
    buf.extend_from_slice(&CMD_SIGN.to_le_bytes());
    buf.extend_from_slice(&CMD_QUERY.to_le_bytes());
    buf.extend_from_slice(&sn.to_le_bytes());
    buf.extend_from_slice(&(padded_len as u16).to_le_bytes());
    buf.extend_from_slice(payload);
    buf.resize(8 + padded_len, 0); // 페이로드 0 패딩

    // 버퍼를 리틀엔디언 u32 워드로 읽어 CRC 계산.
    let words: Vec<u32> = buf
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();
    let crc = stm32_crc(&words);
    buf.extend_from_slice(&crc.to_le_bytes());
    buf
}

/// 데이터 수신용 UDP 소켓을 열고, 센서에 시작 커맨드를 보내 스트리밍을 켠다.
///
/// `host_ip`/`host_port`에 bind해 데이터를 받고, `sensor_ip`/`command_port`로 커맨드를
/// 보낸다. 일부 장치는 커맨드 포트 +1도 쓰므로 두 포트 모두로 보낸다(무해).
/// 응답은 따로 검사하지 않고, 데이터가 오는지로 수집 루프의 워치독이 판단한다.
pub fn connect(
    sensor_ip: &str,
    command_port: u16,
    host_ip: &str,
    host_port: u16,
) -> io::Result<UdpSocket> {
    let bind_ip: Ipv4Addr = host_ip.parse().unwrap_or(Ipv4Addr::UNSPECIFIED);
    let socket = UdpSocket::bind((bind_ip, host_port))?;

    // heartbeat 멀티캐스트 가입(실패해도 데이터 수신엔 지장 없음).
    if let Err(e) = socket.join_multicast_v4(&HEARTBEAT_GROUP, &Ipv4Addr::UNSPECIFIED) {
        eprintln!("[pacecat] heartbeat 멀티캐스트 가입 실패(무시): {e}");
    }

    send_start_sequence(&socket, sensor_ip, command_port)?;
    Ok(socket)
}

/// `LVERSH` → (500ms) → `LSTARH`를 커맨드 포트와 그 +1 포트로 보낸다.
fn send_start_sequence(socket: &UdpSocket, sensor_ip: &str, command_port: u16) -> io::Result<()> {
    let ports = [command_port, command_port + 1];

    let vers = build_command("LVERSH", 0x0001);
    for &port in &ports {
        let _ = socket.send_to(&vers, (sensor_ip, port));
    }
    eprintln!("[pacecat] LVERSH 전송 → {sensor_ip}:{command_port}(+1)");

    thread::sleep(INTER_CMD_DELAY);

    let start = build_command("LSTARH", 0x0002);
    for &port in &ports {
        socket.send_to(&start, (sensor_ip, port))?;
    }
    eprintln!("[pacecat] LSTARH 전송 → {sensor_ip}:{command_port}(+1)");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_layout_is_header_payload_crc() {
        // "LSTARH"(6자) → 8바이트로 패딩 → 헤더8 + 페이로드8 + CRC4 = 20바이트.
        let cmd = build_command("LSTARH", 0x0002);
        assert_eq!(cmd.len(), 8 + 8 + 4);
        assert_eq!(u16::from_le_bytes([cmd[0], cmd[1]]), CMD_SIGN);
        assert_eq!(u16::from_le_bytes([cmd[2], cmd[3]]), CMD_QUERY);
        assert_eq!(u16::from_le_bytes([cmd[4], cmd[5]]), 0x0002); // sn
        assert_eq!(u16::from_le_bytes([cmd[6], cmd[7]]), 8); // padded len
        assert_eq!(&cmd[8..14], b"LSTARH");
        assert_eq!(cmd[14], 0); // 패딩
        assert_eq!(cmd[15], 0);
    }

    #[test]
    fn crc_is_deterministic_and_nonzero() {
        let a = build_command("LSTARH", 1);
        let b = build_command("LSTARH", 1);
        assert_eq!(a, b);
        let crc = &a[a.len() - 4..];
        assert_ne!(crc, &[0, 0, 0, 0]);
    }

    #[test]
    fn different_payload_changes_crc() {
        let a = build_command("LSTARH", 1);
        let b = build_command("LVERSH", 1);
        assert_ne!(a[a.len() - 4..], b[b.len() - 4..]);
    }
}
