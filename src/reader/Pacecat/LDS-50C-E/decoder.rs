//! 바이트(UDP 데이터그램) 스트림에서 `0xFAC7` 패킷을 잘라내는 스트리밍 디코더.
//!
//! UDP는 보통 데이터그램 하나가 패킷 하나라 단순 파싱으로 충분하지만, heartbeat
//! 멀티캐스트(`"LiDA"`)나 잘린 데이터그램이 섞일 수 있으므로 STL-27L과 같은 방식으로
//! 동기·검증한다: 헤더(`0xFAC7`)와 16-bit sum 체크섬이 **모두** 맞는 위치만 소비한다.
//!
//! 가변 길이라 헤더에서 점 개수를 먼저 읽어 패킷 길이를 정한 뒤, 그만큼 다 모이면 검증한다.
//! 점 개수가 비정상이거나 체크섬이 틀리면 1바이트만 밀어 가짜 동기를 흘려보낸다.

use std::collections::VecDeque;

use super::frame::{HEADER_LEN, packet_len, parse, peek_header};
use crate::reader::model::Decoder;
use crate::reader::types::LidarBody;

/// `0xFAC7` 동기 헤더의 첫/둘째 바이트(리틀엔디언).
const SYNC0: u8 = 0xC7;
const SYNC1: u8 = 0xFA;

/// 바이트 스트림 → 패킷 디코더.
pub struct PacketDecoder {
    buffer: VecDeque<u8>,
    last_skipped: usize,
    last_crc_failures: usize,
}

impl PacketDecoder {
    pub fn new() -> Self {
        Self {
            buffer: VecDeque::new(),
            last_skipped: 0,
            last_crc_failures: 0,
        }
    }

    /// 버퍼 앞에서 `n`바이트를 복사한다(VecDeque는 비연속이라 슬라이스를 못 빌린다).
    fn copy_front(&self, n: usize) -> Vec<u8> {
        (0..n).map(|i| self.buffer[i]).collect()
    }

    /// 버퍼 앞에서 검증된 패킷 하나를 찾아 소비한다.
    fn next_frame(&mut self) -> Option<LidarBody> {
        loop {
            match self.buffer.front().copied() {
                None => return None,
                Some(byte) if byte != SYNC0 => {
                    self.buffer.pop_front();
                    self.last_skipped += 1;
                }
                Some(_) => {
                    // 둘째 동기 바이트 확인(아직 안 왔으면 보류).
                    match self.buffer.get(1).copied() {
                        None => return None,
                        Some(b) if b != SYNC1 => {
                            self.buffer.pop_front();
                            self.last_skipped += 1;
                            continue;
                        }
                        Some(_) => {}
                    }
                    // 헤더가 다 와야 점 개수를 읽을 수 있다.
                    if self.buffer.len() < HEADER_LEN {
                        return None;
                    }
                    let head_bytes = self.copy_front(HEADER_LEN);
                    let Some(header) = peek_header(&head_bytes) else {
                        // 동기는 맞지만 점 개수가 비정상 → 가짜 동기. 1바이트만 밀기.
                        self.buffer.pop_front();
                        self.last_skipped += 1;
                        continue;
                    };
                    let len = packet_len(header.subcontract_points);
                    // 패킷이 다 안 모였으면 더 받을 때까지 보류.
                    if self.buffer.len() < len {
                        return None;
                    }
                    let packet = self.copy_front(len);
                    match parse(&packet) {
                        Ok(body) => {
                            self.buffer.drain(..len);
                            return Some(body);
                        }
                        Err(_) => {
                            // 헤더는 맞췄지만 체크섬 불일치 → 1바이트만 밀고 재탐색.
                            self.buffer.pop_front();
                            self.last_skipped += 1;
                            self.last_crc_failures += 1;
                        }
                    }
                }
            }
        }
    }
}

impl Default for PacketDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder for PacketDecoder {
    fn push_bytes(&mut self, bytes: &[u8]) -> Vec<LidarBody> {
        self.buffer.extend(bytes);
        self.last_skipped = 0;
        self.last_crc_failures = 0;

        let mut frames = Vec::new();
        while let Some(frame) = self.next_frame() {
            frames.push(frame);
        }
        frames
    }

    fn last_skipped(&self) -> usize {
        self.last_skipped
    }

    fn last_crc_failures(&self) -> usize {
        self.last_crc_failures
    }
}

#[cfg(test)]
mod tests {
    use super::super::frame::{CHECKSUM_LEN, HEADER, HEADER_LEN, packet_len, peek_header};
    use super::*;

    /// 점 N개짜리 유효 0xFAC7 패킷(체크섬 포함)을 만든다.
    fn build_packet(start_mdeg: u32, pts: &[(u16, u16, u8)]) -> Vec<u8> {
        let npts = pts.len();
        let mut buf = vec![0u8; packet_len(npts)];
        buf[0..2].copy_from_slice(&HEADER.to_le_bytes());
        buf[2..4].copy_from_slice(&(npts as u16).to_le_bytes());
        buf[4..6].copy_from_slice(&(npts as u16).to_le_bytes());
        buf[8..12].copy_from_slice(&start_mdeg.to_le_bytes());

        let dist_off = HEADER_LEN;
        let angle_off = dist_off + npts * 2;
        let strength_off = angle_off + npts * 2;
        for (k, &(d, a, s)) in pts.iter().enumerate() {
            buf[dist_off + k * 2..dist_off + k * 2 + 2].copy_from_slice(&d.to_le_bytes());
            buf[angle_off + k * 2..angle_off + k * 2 + 2].copy_from_slice(&a.to_le_bytes());
            buf[strength_off + k] = s;
        }
        // 체크섬: frame 모듈의 비공개 함수 대신 parse가 통과하도록 동일 계산을 재현한다.
        let header = peek_header(&buf).unwrap();
        let npts = header.subcontract_points;
        let mut sum: u32 = 0;
        let mut i = 2;
        while i + 1 < HEADER_LEN {
            sum = sum.wrapping_add(u16::from_le_bytes([buf[i], buf[i + 1]]) as u32);
            i += 2;
        }
        for k in 0..npts {
            sum = sum
                .wrapping_add(
                    u16::from_le_bytes([buf[dist_off + k * 2], buf[dist_off + k * 2 + 1]]) as u32,
                );
            sum = sum.wrapping_add(u16::from_le_bytes([
                buf[angle_off + k * 2],
                buf[angle_off + k * 2 + 1],
            ]) as u32);
            sum = sum.wrapping_add(buf[strength_off + k] as u32);
        }
        let chk_off = packet_len(npts) - CHECKSUM_LEN;
        buf[chk_off..chk_off + 2].copy_from_slice(&(sum as u16).to_le_bytes());
        buf
    }

    #[test]
    fn extracts_single_packet() {
        let mut decoder = PacketDecoder::new();
        let pkt = build_packet(0, &[(1000, 0, 10), (1200, 500, 20)]);
        let frames = decoder.push_bytes(&pkt);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].points.len(), 2);
    }

    #[test]
    fn skips_leading_noise_and_heartbeat_bytes() {
        let mut decoder = PacketDecoder::new();
        let mut bytes = vec![0x4C, 0x69, 0x44, 0x41, 0x00, 0x11]; // "LiDA"… 잡음
        bytes.extend_from_slice(&build_packet(1000, &[(900, 0, 5)]));
        let frames = decoder.push_bytes(&bytes);
        assert_eq!(frames.len(), 1);
        assert!(decoder.last_skipped() >= 6);
    }

    #[test]
    fn waits_for_full_packet_then_completes() {
        let mut decoder = PacketDecoder::new();
        let pkt = build_packet(0, &[(1000, 0, 10), (1200, 500, 20), (1300, 1000, 30)]);
        let split = HEADER_LEN + 3;
        assert!(decoder.push_bytes(&pkt[..split]).is_empty());
        let frames = decoder.push_bytes(&pkt[split..]);
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn two_packets_in_one_buffer() {
        let mut decoder = PacketDecoder::new();
        let mut bytes = build_packet(0, &[(1000, 0, 10)]);
        bytes.extend_from_slice(&build_packet(2000, &[(1100, 0, 11), (1100, 500, 12)]));
        let frames = decoder.push_bytes(&bytes);
        assert_eq!(frames.len(), 2);
    }
}
