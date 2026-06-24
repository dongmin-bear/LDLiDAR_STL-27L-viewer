//! 모델 추상화 — 어떤 LiDAR든 "바이트 스트림 → 프레임" 한 가지 모양으로 다룬다.
//!
//! [`Decoder`]는 모델별 코덱이 구현하는 공통 트레잇이다. 수집기([`super::data_collector`])는
//! 구체 모델을 모른 채 `Box<dyn Decoder>`만 돌리므로, 새 모델을 추가해도 수집·뷰어
//! 코드는 그대로다. [`Model`]은 설정 문자열("STL-27L" 등)을 구체 디코더로 잇는 팩토리.

use super::types::LidarBody;

/// 바이트 스트림을 받아 검증된 프레임들로 잘라내는 스트리밍 디코더.
///
/// 모델마다 헤더·길이·체크섬 규칙은 다르지만, 외부에서 보는 모양은 같다:
/// 들어온 바이트를 내부 버퍼에 모아두고, 완성·검증된 [`LidarBody`]만 돌려준다.
pub trait Decoder: Send {
    /// 새 바이트를 밀어 넣고, 검증을 통과한 프레임들을 돌려준다.
    fn push_bytes(&mut self, bytes: &[u8]) -> Vec<LidarBody>;

    /// 직전 push에서 동기화를 못 잡아 버린 바이트 수(진단용, 기본 0).
    fn last_skipped(&self) -> usize {
        0
    }

    /// 직전 push에서 헤더는 맞췄으나 체크섬 불일치로 버린 횟수(진단용, 기본 0).
    fn last_crc_failures(&self) -> usize {
        0
    }
}

/// 지원하는 LiDAR 모델.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// LDROBOT STL-27L — 시리얼 47B 프레임.
    Stl27L,
    /// Pacecat(Bluesea) LDS-50C-E — UDP 0xFAC7 패킷.
    Lds50CE,
}

impl Model {
    /// 지원 모델 전체(UI 드롭다운 등에서 순회용).
    pub const ALL: [Model; 2] = [Model::Stl27L, Model::Lds50CE];

    /// 설정 문자열을 모델로 해석한다(대소문자·하이픈 무시). 모르면 `None`.
    pub fn from_name(name: &str) -> Option<Self> {
        let key: String = name
            .chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .map(|c| c.to_ascii_lowercase())
            .collect();
        match key.as_str() {
            "stl27l" => Some(Self::Stl27L),
            "lds50ce" | "lds50c" => Some(Self::Lds50CE),
            _ => None,
        }
    }

    /// 사람이 읽는 모델명.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Stl27L => "LDROBOT STL-27L",
            Self::Lds50CE => "Pacecat LDS-50C-E",
        }
    }

    /// 이 모델의 새 스트리밍 디코더를 만든다.
    pub fn new_decoder(&self) -> Box<dyn Decoder> {
        match self {
            Self::Stl27L => Box::new(super::ldrobot::stl27l::decoder::FrameDecoder::new()),
            Self::Lds50CE => Box::new(super::pacecat::lds50ce::decoder::PacketDecoder::new()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_model_names_loosely() {
        assert_eq!(Model::from_name("STL-27L"), Some(Model::Stl27L));
        assert_eq!(Model::from_name("stl27l"), Some(Model::Stl27L));
        assert_eq!(Model::from_name("LDS-50C-E"), Some(Model::Lds50CE));
        assert_eq!(Model::from_name("lds_50c_e"), Some(Model::Lds50CE));
        assert_eq!(Model::from_name("unknown"), None);
    }
}
