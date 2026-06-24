//! 전송 추상화 — 시리얼이든 UDP든 "바이트를 읽는다" 한 가지 모양으로 다룬다.
//!
//! 모델마다 물리 전송이 다르다: STL-27L은 시리얼(UART), LDS-50C-E는 UDP(네트워크)다.
//! [`Transport`]로 그 차이를 가리면, 수집 루프([`super::data_collector::pump`])는 전송을
//! 몰라도 `read()`만 호출하면 된다. UDP는 시작 커맨드 핸드셰이크까지 끝낸 소켓을 감싼다.

use std::io;
use std::net::UdpSocket;
use std::time::Duration;

/// 바이트 소스. 한 번 호출에 받은 만큼(0일 수 있음)을 `buf`에 채우고 길이를 돌려준다.
/// 타임아웃은 데이터 없음(`Ok(0)` 또는 `TimedOut`)으로 다루고, 그 외 오류는 전파한다.
pub trait Transport: Send {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

/// 시리얼 포트 전송(STL-27L 등).
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Self {
        Self { port }
    }
}

impl Transport for SerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            // 시리얼 타임아웃은 "이번엔 데이터 없음"으로 본다.
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(e),
        }
    }
}

/// UDP 데이터그램 전송(LDS-50C-E 등). 각 `read`는 데이터그램 하나(=대개 패킷 하나)를
/// 통째로 받는다. 읽기 타임아웃이 걸려 있어 수집 루프의 워치독이 동작한다.
pub struct UdpTransport {
    socket: UdpSocket,
}

impl UdpTransport {
    /// 이미 bind·핸드셰이크가 끝난 소켓을 감싼다. 읽기 타임아웃을 걸어둔다.
    pub fn new(socket: UdpSocket, read_timeout: Duration) -> io::Result<Self> {
        socket.set_read_timeout(Some(read_timeout))?;
        Ok(Self { socket })
    }
}

impl Transport for UdpTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.socket.recv_from(buf) {
            Ok((n, _from)) => Ok(n),
            // recv 타임아웃: 플랫폼에 따라 WouldBlock/TimedOut 둘 다 "데이터 없음".
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(e),
        }
    }
}
