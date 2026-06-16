//! 바이트 스트림에서 프레임 경계를 찾아 잘라내는 스트리밍 디코더.
//!
//! 시리얼은 임의 크기로 쪼개져 들어오므로, 내부 버퍼에 모아두고 헤더(`0x54`) +
//! 버전(`0x2c`)으로 프레임 시작을 찾은 뒤 `PACKET_LEN`바이트씩 떼어 파싱한다.

use std::collections::VecDeque;

use super::frame::{parse, HEADER, PACKET_LEN, VER_LEN};
use super::types::{LidarBody, ParseError};

/// 바이트 스트림 → 프레임 디코더.
pub struct FrameDecoder {
    buffer: VecDeque<u8>,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self {
            buffer: VecDeque::new(),
        }
    }

    /// 새 바이트를 밀어 넣고, 그 결과 완성된 프레임들을 파싱해 돌려준다.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<Result<LidarBody, ParseError>> {
        self.buffer.extend(bytes);

        let mut frames = Vec::new();
        while let Some(raw_frame) = self.next_raw_frame() {
            frames.push(parse(&raw_frame));
        }

        frames
    }

    fn next_raw_frame(&mut self) -> Option<Vec<u8>> {
        self.discard_until_header();

        if self.buffer.len() < PACKET_LEN {
            return None;
        }

        if self.buffer.get(1).copied() != Some(VER_LEN) {
            self.buffer.pop_front();
            return self.next_raw_frame();
        }

        Some(self.buffer.drain(..PACKET_LEN).collect())
    }

    fn discard_until_header(&mut self) {
        while self
            .buffer
            .front()
            .copied()
            .is_some_and(|byte| byte != HEADER)
        {
            self.buffer.pop_front();
        }
    }
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
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
        assert!(frames[0].is_ok());
    }
}
