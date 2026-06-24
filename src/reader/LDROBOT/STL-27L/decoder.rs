//! 바이트 스트림에서 프레임 경계를 찾아 잘라내는 스트리밍 디코더.
//!
//! 동기화 방식: 헤더(`0x54`) + 버전(`0x2c`) **그리고 CRC8까지 모두 맞는** 위치를
//! 찾을 때까지 1바이트씩 밀며 검증한다. 확정된 경우에만 `PACKET_LEN`바이트를
//! 소비하므로, 우연히 나온 가짜 헤더 때문에 진짜 프레임을 통째로 까먹는 일이 없다.
//!
//! - CRC까지 통과 → 그 47바이트를 소비하고 [`LidarBody`] 반환.
//! - 헤더는 맞췄으나 CRC 불일치 → **1바이트만** 밀고 재탐색(프레임 보존).
//! - 헤더가 아님 → 1바이트 버리고 계속.
//! - 버퍼가 한 프레임보다 짧음 → 더 받을 때까지 보류(헤더 후보 유지).

use std::collections::VecDeque;

use super::frame::{CRC_OFFSET, HEADER, PACKET_LEN, VER_LEN, crc8, parse};
use crate::reader::model::Decoder;
use crate::reader::types::LidarBody;

/// 바이트 스트림 → 프레임 디코더.
pub struct FrameDecoder {
    buffer: VecDeque<u8>,
    /// 직전 `push_bytes`에서 동기화를 못 잡아 버린 바이트 수(진단용).
    last_skipped: usize,
    /// 직전 `push_bytes`에서 헤더는 맞췄으나 CRC가 틀려 버린 횟수(진단용).
    last_crc_failures: usize,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self {
            buffer: VecDeque::new(),
            last_skipped: 0,
            last_crc_failures: 0,
        }
    }

    /// 새 바이트를 밀어 넣고, CRC까지 검증된 프레임들만 돌려준다.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<LidarBody> {
        self.buffer.extend(bytes);
        self.last_skipped = 0;
        self.last_crc_failures = 0;

        let mut frames = Vec::new();
        while let Some(frame) = self.next_frame() {
            frames.push(frame);
        }
        frames
    }

    /// 직전 push에서 동기화에 실패해 버린 바이트 수.
    pub fn last_skipped(&self) -> usize {
        self.last_skipped
    }

    /// 직전 push에서 헤더 일치 후 CRC 불일치로 버린 횟수.
    pub fn last_crc_failures(&self) -> usize {
        self.last_crc_failures
    }

    /// 버퍼 앞에서 CRC까지 검증된 프레임 하나를 찾아 소비한다.
    fn next_frame(&mut self) -> Option<LidarBody> {
        loop {
            match self.buffer.front().copied() {
                None => return None,
                // 빠른 경로: 헤더가 아니면 버린다.
                Some(byte) if byte != HEADER => {
                    self.buffer.pop_front();
                    self.last_skipped += 1;
                }
                Some(_) => {
                    // 헤더 후보. 한 프레임이 다 모이기 전이면 보류(헤더 보존).
                    if self.buffer.len() < PACKET_LEN {
                        return None;
                    }
                    // 버전 바이트 확인.
                    if self.buffer.get(1).copied() != Some(VER_LEN) {
                        self.buffer.pop_front();
                        self.last_skipped += 1;
                        continue;
                    }
                    // 후보 47바이트를 복사해 CRC 검증.
                    let mut window = [0_u8; PACKET_LEN];
                    for (i, slot) in window.iter_mut().enumerate() {
                        *slot = self.buffer[i];
                    }
                    if crc8(&window[..CRC_OFFSET]) != window[CRC_OFFSET] {
                        // 헤더는 맞췄지만 CRC 불일치 → 47B 통째로 버리지 않고 1바이트만 밀기.
                        self.buffer.pop_front();
                        self.last_skipped += 1;
                        self.last_crc_failures += 1;
                        continue;
                    }
                    // 검증 완료 → 소비하고 반환.
                    self.buffer.drain(..PACKET_LEN);
                    return parse(&window).ok();
                }
            }
        }
    }
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// 공용 [`Decoder`] 트레잇 구현(수집기가 모델을 모른 채 돌리기 위함). 인헌트 메서드에
/// 위임하므로 `ldlidar` 콘솔 바이너리처럼 구체 타입을 직접 쓰는 코드도 그대로 동작한다.
impl Decoder for FrameDecoder {
    fn push_bytes(&mut self, bytes: &[u8]) -> Vec<LidarBody> {
        FrameDecoder::push_bytes(self, bytes)
    }

    fn last_skipped(&self) -> usize {
        FrameDecoder::last_skipped(self)
    }

    fn last_crc_failures(&self) -> usize {
        FrameDecoder::last_crc_failures(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PDF_EXAMPLE_FRAME: [u8; PACKET_LEN] = [
        0x54, 0x2c, 0x68, 0x08, 0xab, 0x7e, 0xe0, 0x00, 0xe4, 0xdc, 0x00, 0xe2, 0xd9, 0x00, 0xe5,
        0xd5, 0x00, 0xe3, 0xd3, 0x00, 0xe4, 0xd0, 0x00, 0xe9, 0xcd, 0x00, 0xe4, 0xca, 0x00, 0xe2,
        0xc7, 0x00, 0xe9, 0xc5, 0x00, 0xe5, 0xc2, 0x00, 0xe5, 0xc0, 0x00, 0xe5, 0xbe, 0x82, 0x3a,
        0x1a, 0x50,
    ];

    #[test]
    fn decoder_skips_noise_and_extracts_frame() {
        let mut decoder = FrameDecoder::new();
        let mut bytes = vec![0x00, 0x11, 0x54, 0x00, 0x54];
        bytes.extend_from_slice(&PDF_EXAMPLE_FRAME[1..]);

        let frames = decoder.push_bytes(&bytes);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].points.len(), 12);
    }

    #[test]
    fn false_header_with_bad_crc_advances_one_byte_only() {
        // 가짜 "54 2c …"(CRC 불일치) 바로 뒤에 진짜 프레임을 둔다.
        // 가짜 때문에 진짜 프레임을 까먹지 않고 결국 1개를 뽑아내야 한다.
        let mut decoder = FrameDecoder::new();
        let mut bytes = vec![0x54, 0x2c];
        bytes.extend(std::iter::repeat_n(0xAA, PACKET_LEN)); // 가짜 페이로드
        bytes.extend_from_slice(&PDF_EXAMPLE_FRAME); // 진짜 프레임

        let frames = decoder.push_bytes(&bytes);

        assert_eq!(frames.len(), 1);
        assert!(decoder.last_crc_failures() >= 1);
    }

    #[test]
    fn waits_for_full_frame_before_emitting() {
        let mut decoder = FrameDecoder::new();
        // 헤더만 있고 프레임이 덜 찼으면 아무것도 내지 않고 보류.
        let half = decoder.push_bytes(&PDF_EXAMPLE_FRAME[..20]);
        assert!(half.is_empty());
        // 나머지가 도착하면 완성.
        let rest = decoder.push_bytes(&PDF_EXAMPLE_FRAME[20..]);
        assert_eq!(rest.len(), 1);
    }
}
