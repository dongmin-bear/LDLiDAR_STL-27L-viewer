//! Pacecat LDS-50C-E `0xFAC7` 데이터 패킷의 레이아웃과 파싱.
//!
//! C++ 레퍼런스(`Parser/LidarModule/Reader.cpp`, `include/type.hpp`)를 그대로 옮겼다.
//! STL-27L과 다른 점 두 가지를 유의:
//!
//! 1. **가변 길이** — 한 패킷의 점 개수(`subcontract_points`)가 헤더에 들어 있어,
//!    패킷 길이 = 28(헤더) + N×5(점) + 2(체크섬)로 매번 달라진다.
//! 2. **SoA 배치** — 점이 `{거리,각도,세기}` 구조체 배열이 아니라, 거리 N개 → 각도 N개
//!    → 세기 N개 순서의 "배열들의 묶음"이다. 그래서 zero-copy 캐스팅 대신 오프셋으로 읽는다.

use crate::reader::types::{LidarBody, LidarPoint, ParseError};

/// 데이터 패킷 동기 헤더(리틀엔디언 메모리상 바이트는 `c7 fa`).
pub const HEADER: u16 = 0xFAC7;
/// 고정 헤더 길이(바이트).
pub const HEADER_LEN: usize = 28;
/// 점 1개가 차지하는 총 바이트(거리 2 + 각도 2 + 세기 1) — 단, SoA라 흩어져 있다.
pub const POINT_STRIDE: usize = 5;
/// 체크섬 길이(바이트).
pub const CHECKSUM_LEN: usize = 2;
/// 한 패킷이 담을 수 있는 점 개수 상한(가짜 동기 거부용 안전값).
pub const MAX_POINTS_PER_PACKET: usize = 1500;

/// 거리(mm)와 각도(0.001°) 환산 계수.
const ANGLE_SCALE_DEG: f32 = 0.001;

fn read_u16_le(data: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([data[offset], data[offset + 1]])
}

fn read_u32_le(data: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ])
}

/// 헤더에서 읽은, 패킷 해석에 필요한 핵심 값.
#[derive(Debug, Clone, Copy)]
pub struct Header {
    pub subcontract_points: usize,
    pub total_points: u16,
    pub start_angle_mdeg: u32,
    pub end_angle_mdeg: u32,
    pub timestamp: u32,
}

/// 버퍼 앞 28바이트를 헤더로 해석한다. 동기 헤더가 아니거나 점 개수가 비정상이면 `None`.
/// (디코더가 가짜 동기를 빨리 거르는 데 쓴다.)
pub fn peek_header(data: &[u8]) -> Option<Header> {
    if data.len() < HEADER_LEN {
        return None;
    }
    if read_u16_le(data, 0) != HEADER {
        return None;
    }
    let subcontract_points = read_u16_le(data, 2) as usize;
    if subcontract_points == 0 || subcontract_points > MAX_POINTS_PER_PACKET {
        return None;
    }
    Some(Header {
        subcontract_points,
        total_points: read_u16_le(data, 4),
        start_angle_mdeg: read_u32_le(data, 8),
        end_angle_mdeg: read_u32_le(data, 12),
        timestamp: read_u32_le(data, 20),
    })
}

/// 점 개수 N으로 결정되는 패킷 총 길이.
pub fn packet_len(subcontract_points: usize) -> usize {
    HEADER_LEN + subcontract_points * POINT_STRIDE + CHECKSUM_LEN
}

/// 16-bit sum 체크섬(C++ `CalcChecksum`과 동일).
///
/// 헤더는 오프셋 2부터(동기 2바이트 제외) 16비트 워드 단위로 더하고, 점은 거리+각도+세기를
/// 더한다. 누산은 32비트로 하되 하위 16비트만 비교한다.
fn checksum(data: &[u8], header: &Header) -> u16 {
    let npts = header.subcontract_points;
    let mut sum: u32 = 0;

    // 헤더 절반워드(오프셋 2..28).
    let mut i = 2;
    while i + 1 < HEADER_LEN {
        sum = sum.wrapping_add(read_u16_le(data, i) as u32);
        i += 2;
    }

    // SoA: 거리 N개 → 각도 N개 → 세기 N개.
    let dist_off = HEADER_LEN;
    let angle_off = dist_off + npts * 2;
    let strength_off = angle_off + npts * 2;
    for k in 0..npts {
        sum = sum.wrapping_add(read_u16_le(data, dist_off + k * 2) as u32);
        sum = sum.wrapping_add(read_u16_le(data, angle_off + k * 2) as u32);
        sum = sum.wrapping_add(data[strength_off + k] as u32);
    }

    sum as u16
}

/// 정확히 한 패킷(`packet_len`바이트)인 슬라이스를 파싱한다.
///
/// 길이·헤더·체크섬을 검증한 뒤, SoA 배열에서 점을 모아 절대각으로 변환한다.
/// 각 점의 절대각(도) = (각도 오프셋 + 시작각) × 0.001.
pub fn parse(data: &[u8]) -> Result<LidarBody, ParseError> {
    let header = peek_header(data).ok_or(ParseError::InvalidHeader(if data.len() >= 2 {
        read_u16_le(data, 0)
    } else {
        0
    }))?;

    let expected_len = packet_len(header.subcontract_points);
    if data.len() < expected_len {
        return Err(ParseError::InvalidLength {
            expected: expected_len,
            actual: data.len(),
        });
    }

    let expected = checksum(data, &header);
    let actual = read_u16_le(data, expected_len - CHECKSUM_LEN);
    if expected != actual {
        return Err(ParseError::CrcMismatch {
            expected: expected as u32,
            actual: actual as u32,
        });
    }

    let npts = header.subcontract_points;
    let dist_off = HEADER_LEN;
    let angle_off = dist_off + npts * 2;
    let strength_off = angle_off + npts * 2;

    let points = (0..npts)
        .map(|k| {
            let distance_mm = read_u16_le(data, dist_off + k * 2);
            let angle_offset = read_u16_le(data, angle_off + k * 2) as u32;
            let intensity = data[strength_off + k];
            let absolute_mdeg = angle_offset.wrapping_add(header.start_angle_mdeg);
            LidarPoint {
                angle_degrees: absolute_mdeg as f32 * ANGLE_SCALE_DEG,
                distance_mm,
                intensity,
            }
        })
        .collect();

    Ok(LidarBody {
        // 데이터 패킷에는 회전 속도가 없다(heartbeat에만 있음) → 0으로 둔다.
        speed_degrees_per_second: 0,
        start_angle_degrees: header.start_angle_mdeg as f32 * ANGLE_SCALE_DEG,
        end_angle_degrees: header.end_angle_mdeg as f32 * ANGLE_SCALE_DEG,
        timestamp_ms: header.timestamp,
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 점 N개짜리 0xFAC7 패킷을 만들어 끝에 올바른 체크섬을 붙인다(테스트용).
    fn build_packet(start_mdeg: u32, pts: &[(u16, u16, u8)]) -> Vec<u8> {
        let npts = pts.len();
        let mut buf = vec![0u8; packet_len(npts)];
        buf[0..2].copy_from_slice(&HEADER.to_le_bytes());
        buf[2..4].copy_from_slice(&(npts as u16).to_le_bytes());
        buf[4..6].copy_from_slice(&(npts as u16).to_le_bytes()); // total
        buf[8..12].copy_from_slice(&start_mdeg.to_le_bytes());
        let end = start_mdeg + pts.last().map(|p| p.1 as u32).unwrap_or(0);
        buf[12..16].copy_from_slice(&end.to_le_bytes());

        let dist_off = HEADER_LEN;
        let angle_off = dist_off + npts * 2;
        let strength_off = angle_off + npts * 2;
        for (k, &(d, a, s)) in pts.iter().enumerate() {
            buf[dist_off + k * 2..dist_off + k * 2 + 2].copy_from_slice(&d.to_le_bytes());
            buf[angle_off + k * 2..angle_off + k * 2 + 2].copy_from_slice(&a.to_le_bytes());
            buf[strength_off + k] = s;
        }

        let header = peek_header(&buf).unwrap();
        let chk = checksum(&buf, &header);
        let chk_off = packet_len(npts) - CHECKSUM_LEN;
        buf[chk_off..chk_off + 2].copy_from_slice(&chk.to_le_bytes());
        buf
    }

    #[test]
    fn parses_packet_with_absolute_angles() {
        // 시작각 45.000°, 점 2개(각도 오프셋 0°, 1.000°).
        let pkt = build_packet(45_000, &[(1000, 0, 200), (2000, 1000, 180)]);
        let body = parse(&pkt).expect("valid packet should parse");

        assert_eq!(body.points.len(), 2);
        assert_eq!(body.points[0].distance_mm, 1000);
        assert_eq!(body.points[0].intensity, 200);
        assert!((body.points[0].angle_degrees - 45.0).abs() < 1e-3);
        assert!((body.points[1].angle_degrees - 46.0).abs() < 1e-3);
        assert!((body.start_angle_degrees - 45.0).abs() < 1e-3);
    }

    #[test]
    fn rejects_corrupted_checksum() {
        let mut pkt = build_packet(0, &[(1000, 0, 10), (1000, 500, 10)]);
        let len = pkt.len();
        pkt[len - 1] ^= 0xFF; // 체크섬 깨뜨리기
        assert!(matches!(parse(&pkt), Err(ParseError::CrcMismatch { .. })));
    }

    #[test]
    fn rejects_wrong_header() {
        let mut pkt = build_packet(0, &[(1000, 0, 10)]);
        pkt[0] = 0x00;
        assert!(matches!(parse(&pkt), Err(ParseError::InvalidHeader(_))));
    }
}
