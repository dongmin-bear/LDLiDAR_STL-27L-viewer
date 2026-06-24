//! STL-27L 47바이트 프레임의 바이너리 레이아웃과 파싱.
//!
//! C++의 `(LiDARFrameTypeDef *)buffer` 캐스팅처럼, 수신 바이트 슬라이스를 복사 없이
//! 구조체로 들여다본 뒤(zero-copy) CRC8 검증을 거쳐 [`LidarBody`]로 변환한다.

use zerocopy::byteorder::little_endian::U16;
use zerocopy::{FromBytes, Immutable, KnownLayout, Unaligned};

use crate::reader::types::{LidarBody, LidarPoint, ParseError};

pub const HEADER: u8 = 0x54;
pub const VER_LEN: u8 = 0x2c;
pub const POINTS_PER_PACKET: usize = 12;
pub const POINT_SIZE: usize = 3;
pub const PACKET_LEN: usize = 1 + 1 + 2 + 2 + POINTS_PER_PACKET * POINT_SIZE + 2 + 2 + 1;
pub const CRC_OFFSET: usize = PACKET_LEN - 1;

/// 측정 점 1개: 거리(2B, LE) + 신호 세기(1B). PDF §3.1 `LidarPointStructDef`와 동일.
#[repr(C)]
#[derive(FromBytes, KnownLayout, Immutable, Unaligned)]
struct RawPoint {
    distance: U16,
    intensity: u8,
}

#[repr(C)]
#[derive(FromBytes, KnownLayout, Immutable, Unaligned)]
struct RawBody {
    ver_len: u8,
    speed: U16,
    start_angle: U16,
    points: [RawPoint; POINTS_PER_PACKET],
    end_angle: U16,
    timestamp: U16,
}

/// 47바이트 프레임 전체 레이아웃 — PDF §3.1 `LiDARFrameTypeDef`와 1:1 대응.
/// `U16`은 리틀엔디언 바이트 배열이라 호스트 엔디언과 무관하게 동작하고,
/// 모든 필드의 정렬이 1이라 패딩 없이 정확히 `PACKET_LEN` 바이트가 된다.
#[repr(C)]
#[derive(FromBytes, KnownLayout, Immutable, Unaligned)]
struct RawData {
    header: u8,
    body: RawBody,
    crc8: u8,
}

// 레이아웃이 PDF 명세(47B)와 어긋나면 컴파일 자체가 실패한다.
const _: () = assert!(core::mem::size_of::<RawData>() == PACKET_LEN);

const CRC_TABLE: [u8; 256] = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
];

/// 바이트열의 CRC8(LDROBOT 전용 테이블)을 계산한다.
pub fn crc8(bytes: &[u8]) -> u8 {
    bytes
        .iter()
        .fold(0_u8, |crc, byte| CRC_TABLE[(crc ^ byte) as usize])
}

/// 정확히 `PACKET_LEN`바이트인 슬라이스를 프레임으로 파싱한다.
///
/// 길이·정렬을 검사한 뒤 복사 없이 바이트 슬라이스를 구조체로 들여다본다(zero-copy).
/// `RawData`는 `Unaligned`라 정렬 실패는 없고, 길이가 안 맞을 때만 에러가 난다.
pub fn parse(data: &[u8]) -> Result<LidarBody, ParseError> {
    let raw = RawData::ref_from_bytes(data).map_err(|_| ParseError::InvalidLength {
        expected: PACKET_LEN,
        actual: data.len(),
    })?;

    if raw.header != HEADER {
        return Err(ParseError::InvalidHeader(raw.header as u16));
    }

    if raw.body.ver_len != VER_LEN {
        return Err(ParseError::InvalidVerLen(raw.body.ver_len));
    }

    let expected_crc = crc8(&data[..CRC_OFFSET]);
    if expected_crc != raw.crc8 {
        return Err(ParseError::CrcMismatch {
            expected: expected_crc as u32,
            actual: raw.crc8 as u32,
        });
    }

    let start_angle_raw = raw.body.start_angle.get();
    let end_angle_raw = raw.body.end_angle.get();

    Ok(LidarBody {
        speed_degrees_per_second: raw.body.speed.get(),
        start_angle_degrees: raw_angle_to_degrees(start_angle_raw as f32),
        end_angle_degrees: raw_angle_to_degrees(end_angle_raw as f32),
        timestamp_ms: raw.body.timestamp.get() as u32,
        points: build_points(&raw.body.points, start_angle_raw, end_angle_raw),
    })
}

/// 시작·끝 각도를 등분 보간해 점 12개의 각도를 채운다.
fn build_points(
    raw_points: &[RawPoint; POINTS_PER_PACKET],
    start_angle_raw: u16,
    end_angle_raw: u16,
) -> Vec<LidarPoint> {
    let start = start_angle_raw as f32;
    let mut end = end_angle_raw as f32;

    if end < start {
        // 추정: PDF는 0도 경계 통과 시 보정 규칙을 따로 설명하지 않는다.
        // 실제 연속 스캔에서는 359.x -> 0.x도 구간이 가능하므로 보간용으로 360도를 더한다.
        end += 36_000.0;
    }

    let step = (end - start) / (POINTS_PER_PACKET as f32 - 1.0);

    raw_points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let angle = (start + step * index as f32) % 36_000.0;

            LidarPoint {
                angle_degrees: raw_angle_to_degrees(angle),
                distance_mm: point.distance.get(),
                intensity: point.intensity,
            }
        })
        .collect()
}

fn raw_angle_to_degrees(raw_angle: f32) -> f32 {
    raw_angle / 100.0
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
    fn crc_matches_pdf_example() {
        assert_eq!(crc8(&PDF_EXAMPLE_FRAME[..CRC_OFFSET]), 0x50);
    }

    #[test]
    fn parses_pdf_example() {
        let frame = parse(&PDF_EXAMPLE_FRAME).expect("PDF example should parse");

        assert_eq!(frame.speed_degrees_per_second, 2_152);
        assert_eq!(frame.start_angle_degrees, 324.27);
        assert_eq!(frame.end_angle_degrees, 334.70);
        assert_eq!(frame.timestamp_ms, 0x1a3a);
        assert_eq!(frame.points.len(), POINTS_PER_PACKET);
        assert_eq!(frame.points[0].distance_mm, 224);
        assert_eq!(frame.points[0].intensity, 228);
        assert_eq!(frame.points[1].distance_mm, 220);
        assert_eq!(frame.points[1].intensity, 226);
        assert_eq!(frame.points[11].distance_mm, 192);
        assert_eq!(frame.points[11].intensity, 229);
    }
}
