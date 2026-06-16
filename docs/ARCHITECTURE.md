# 아키텍처 — raw 프레임이 LidarBody가 되고, 뷰어까지 흐르는 과정

이 문서는 두 가지를 설명한다.

1. **수신 바이트(raw 프레임)의 "포인터"가 어떻게 `LidarBody`로 바뀌는가** — C++의 구조체 캐스팅에 대응하는 zero-copy 파싱.
2. **그 `LidarBody`가 어떻게 뷰어 화면까지 연결되는가** — 스레드 → 채널 → 스캔 버퍼 → egui_plot.

---

## 1. 모듈 지도

현대식(2018+) 레이아웃: 폴더와 같은 이름의 `*.rs`가 모듈 루트, 하위 모듈은 같은 이름 폴더 안에 둔다.

```
src/
├── lib.rs              # 모듈 선언 + 공개 API 재노출
├── main.rs             # ldlidar : 콘솔 덤프 바이너리
├── bin/viewer.rs       # viewer  : GUI 진입점 → viewer::run()
│
├── reader.rs           # [모듈 루트] 읽기·파싱
├── reader/
│   ├── frame.rs        #   47B 프레임 레이아웃 + zero-copy parse() + CRC8
│   ├── decoder.rs      #   FrameDecoder : 바이트 스트림 → 프레임 절단
│   └── types.rs        #   LidarPoint / LidarBody / ParseError (공개 타입)
│
├── viewer.rs           # [모듈 루트] egui_plot 뷰어 + run()
└── viewer/
    ├── config.rs       #   config.toml 스키마 + notify 핫리로드
    ├── source.rs       #   시리얼 리더 스레드 + 데모 + 연결 상태
    └── scan.rs         #   극좌표 → 직교좌표 누적 버퍼
```

---

## 2. raw 프레임 "포인터" → `LidarBody`

### 2.1 C++ 캐스팅과의 대응

펌웨어 예제(C++)는 수신 버퍼를 구조체 포인터로 그냥 캐스팅한다.

```cpp
// C++
LiDARFrameTypeDef *frame = (LiDARFrameTypeDef *)buffer; // 복사 없음
uint16_t speed = frame->speed;                          // 그 자리에서 읽기
```

Rust에서도 **복사 없이** 같은 일을 하되, 길이·정렬을 안전하게 검사한다. 그 핵심이
`reader/frame.rs`의 `parse()`다.

```rust
// reader/frame.rs
pub fn parse(data: &[u8]) -> Result<LidarBody, ParseError> {
    // (1) &[u8] → &RawData : 같은 바이트를 "들여다보는" 참조(=포인터). 복사 없음.
    let raw = RawData::ref_from_bytes(data).map_err(|_| ParseError::InvalidLength { .. })?;

    // (2) 검증 : 헤더(0x54) → 버전/길이(0x2c) → CRC8
    if raw.header != HEADER { return Err(ParseError::InvalidHeader(raw.header)); }
    if raw.body.ver_len != VER_LEN { return Err(ParseError::InvalidVerLen(raw.body.ver_len)); }
    let expected = crc8(&data[..CRC_OFFSET]);
    if expected != raw.crc8 { return Err(ParseError::CrcMismatch { .. }); }

    // (3) 필요한 값을 "복사해 꺼내" 소유 타입 LidarBody 생성
    Ok(LidarBody {
        speed_degrees_per_second: raw.body.speed.get(),
        start_angle_degrees: raw_angle_to_degrees(raw.body.start_angle.get() as f32),
        ...
        points: build_points(&raw.body.points, start_raw, end_raw),
    })
}
```

### 2.2 `ref_from_bytes`가 "포인터"인 이유

`RawData::ref_from_bytes(data)`가 돌려주는 `&RawData`는 **새 메모리가 아니다.** `data`가
가리키는 바로 그 바이트 위에 구조체 레이아웃을 겹쳐 놓은 참조(borrow)다. 즉 C++의
`(LiDARFrameTypeDef*)buffer`와 같은 "포인터 뷰"이고, 비용은 0이다.

```
data: &[u8]   ┌──┬────────────── body (43B) ──────────────┬──┐
 (47 bytes)   │54│2c│speed│start│ point0 … point11 │end│ts │CRC│
              └▲─┴──┴─────┴─────┴──────────────────┴───┴───┴──┘
               │
       raw: &RawData  ← 같은 주소를 RawData로 해석 (복사 X)
               │
      raw.body.speed.get()  → little-endian U16를 u16로 읽음
```

이게 성립하려면 `RawData`가 세 가지 조건을 만족해야 하고, 이를 derive로 컴파일 타임에 강제한다.

| 조건 | 의미 | 보장 방법 |
|---|---|---|
| `#[repr(C)]` | 필드 순서/오프셋이 명세대로 고정 | repr(C) |
| `Unaligned` | 정렬 요구가 1 → 임의 위치 슬라이스에 겹쳐도 안전 | 모든 필드 정렬 1 (`u8`, LE `U16`) |
| `FromBytes` | 어떤 비트 패턴이든 유효한 값 | derive (`u8`/`U16` 모두 충족) |

또한 엔디안 독립성을 위해 다중바이트 필드는 `zerocopy::…::little_endian::U16`을 쓴다. 호스트가
빅엔디안이어도 `.get()`이 항상 리틀엔디안으로 해석하므로 안전하다.

마지막으로, 레이아웃이 명세(47B)와 어긋나면 **컴파일 자체가 실패**하도록 정적 단언을 둔다.

```rust
const _: () = assert!(core::mem::size_of::<RawData>() == PACKET_LEN); // 47
```

### 2.3 `&RawData`(빌림) vs `LidarBody`(소유)

- `raw: &RawData` — 수신 버퍼를 빌려 보는 **일시적 뷰**. 버퍼가 사라지면 같이 무효.
- `LidarBody` — 필요한 값만 꺼내 담은 **독립 소유 스냅샷**(`points`는 `Vec`). 버퍼와 수명이
  분리돼 채널로 넘기거나 저장해도 안전.

즉 `parse()`는 "포인터로 들여다보고(zero-copy) → 검증하고 → 값으로 복사해 나온다"의 3단계다.

### 2.4 한 프레임 안의 12개 점 각도 채우기

한 프레임은 시작각/끝각과 점 12개를 담는다. 각 점의 각도는 두 각도 사이를 **등분 보간**한다
(`build_points`). 0° 경계를 넘는 경우(끝각 < 시작각)는 끝각에 360°를 더해 보간한 뒤 다시 360°로
나눈 나머지를 취한다.

```
angle[i] = (start + (end-start)/11 * i) mod 360
```

---

## 3. 스트림 → 프레임 : `FrameDecoder`

시리얼은 47B 단위로 깔끔히 오지 않고 임의 크기로 쪼개진다. `reader/decoder.rs`의
`FrameDecoder`가 내부 `VecDeque<u8>` 버퍼에 모아두고 프레임을 잘라낸다.

```rust
// reader/decoder.rs
pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<Result<LidarBody, ParseError>> {
    self.buffer.extend(bytes);
    let mut frames = Vec::new();
    while let Some(raw) = self.next_raw_frame() {   // 헤더 0x54 + 버전 0x2c 정렬
        frames.push(parse(&raw));                   // 각 47B를 parse()로
    }
    frames
}
```

절단 규칙(**CRC 게이트 동기화**): ① 앞에서 `0x54`가 아닌 바이트를 버린다 → ② 두 번째 바이트가
`0x2c`가 아니면 그 `0x54`도 버린다 → ③ 헤더·버전이 맞으면 47B 창의 **CRC8까지 검증** → ④ CRC가
맞아야만 47B를 소비해 프레임으로 반환하고, **CRC가 틀리면 1바이트만 밀어 재탐색**한다(우연한
가짜 헤더 때문에 진짜 프레임을 통째로 까먹지 않도록). 버퍼가 한 프레임보다 짧으면 헤더 후보를
유지한 채 더 받을 때까지 보류한다.

> 디버그 빌드에서는 `source.rs`가 수신 바이트 수와 프레임 ok/err(유형별) 통계를 1초마다
> 로그로 남긴다. `ok=0`인데 바이트는 들어오면 헤더/baud 불일치를 의심한다.

---

## 4. 프레임 → 뷰어 화면

### 4.1 전체 흐름

```
 [시리얼 /dev/ttyUSB0]
        │ 바이트
        ▼
 source.rs : 별도 스레드 (run_serial → pump_serial)
        │   FrameDecoder.push_bytes() → Vec<Result<LidarBody>>
        │   body.points 각각을 채널로 send
        ▼
   mpsc::channel  ───────────────► Feed { points: Receiver<LidarPoint>,
        │                                  status: Arc<Mutex<ConnectionStatus>> }
        ▼
 viewer.rs : ViewerApp::ui()  (매 프레임)
        │   drain_points() : 채널 비우기 → ScanBuffer.ingest(point)
        ▼
 scan.rs : ScanBuffer
        │   각도 버킷에 "최신 한 바퀴" 유지 (극좌표)
        │   cartesian_points(max_range) → Vec<CartesianPoint{x,y,intensity}>
        ▼
 viewer.rs : draw_plot() → egui_plot (그리드·좌표축·줌·팬 내장)
                          draw_points() : 점 렌더 (config 색/크기)
```

### 4.2 공급 스레드 (`viewer/source.rs`)

`source::spawn(Source)`는 백그라운드 스레드를 띄우고 `Feed`를 돌려준다.

```rust
pub struct Feed {
    pub points: Receiver<LidarPoint>,        // 측정점 스트림
    pub status: Arc<Mutex<ConnectionStatus>>,// 연결 상태(UI 표시용)
}
```

- `run_serial` : 포트를 열어 `pump_serial`로 계속 읽는다. 실패하면 **데모로 조용히 바꾸지 않고**
  상태를 `Error(사유)`로 알린 뒤 1초마다 재연결을 시도한다. (데모는 `demo = true`일 때만.)
- `pump_serial` : `FrameDecoder.push_bytes()` 결과의 `Ok(body)`마다 `body.points`를 채널로
  보낸다. 수신 측(앱)이 사라지면 `send` 실패로 스레드가 종료된다.

### 4.3 누적 버퍼 (`viewer/scan.rs`)

`LidarPoint`(각도°, 거리mm)를 직교좌표로 바꿔 화면에 그린다. 같은 각도가 계속 다시 들어오므로,
각도를 일정 간격으로 나눈 **버킷마다 최신 값만** 보관하면 회전 경계를 따로 감지하지 않아도
안정적인 "현재 한 바퀴" 화면이 된다.

각 버킷 샘플은 **수신 시각**을 함께 들고 있어, `[scan] decay_ms`가 켜져 있으면 마지막 갱신 후
그 시간이 지난(=측정이 끊긴) 점을 `cartesian_points()`에서 제외한다. `0`이면 시간 제한 없이 유지.

```
x = (distance_mm/1000) · cos(angle)
y = (distance_mm/1000) · sin(angle)   // 0° = +x축, 반시계 = +각도
```

### 4.4 렌더와 설정 (`viewer.rs` + `viewer/config.rs`)

- `ViewerApp::ui()`가 매 프레임 `drain_config()`(핫리로드) → `drain_points()`(점 수집) →
  `draw_plot()` 순으로 실행된다.
- `draw_plot()`은 `egui_plot::Plot`에 `show_grid`/`show_axes`/`coordinates_formatter`를 켜서
  그리드·좌표·줌·팬을 내장으로 제공한다.
- `config.toml`은 `notify`로 감시(`config::watch`)하여 저장 즉시 점 크기·색·표시 옵션과 최대
  거리 등이 채널로 전달돼 다음 프레임에 반영된다. (단, `[lidar] port`/`baud` 변경은 시리얼
  연결을 다시 잡아야 하므로 뷰어 재시작이 필요하다.)

---

## 5. 한눈 요약

| 단계 | 위치 | 입력 → 출력 | 핵심 |
|---|---|---|---|
| 절단 | `reader/decoder.rs` | 바이트 스트림 → 47B 프레임 | 0x54/0x2c 정렬 |
| 파싱 | `reader/frame.rs` | `&[u8]` → `&RawData`(포인터) → `LidarBody`(소유) | zero-copy + CRC8 |
| 공급 | `viewer/source.rs` | 프레임 → `LidarPoint` 채널 | 스레드 + 상태/재연결 |
| 누적 | `viewer/scan.rs` | 극좌표 → 직교좌표 | 각도 버킷 최신값 |
| 렌더 | `viewer.rs` | 점 → 화면 | egui_plot + config |
