# 아키텍처 — raw 프레임이 LidarBody가 되고, 뷰어까지 흐르는 과정

이 문서는 두 가지를 설명한다.

1. **수신 바이트(raw 프레임)의 "포인터"가 어떻게 `LidarBody`로 바뀌는가** — C++의 구조체 캐스팅에 대응하는 zero-copy 파싱.
2. **그 `LidarBody`가 어떻게 뷰어 화면까지 연결되는가** — 스레드 → 채널 → 스캔 버퍼 → egui_plot.

---

## 1. 모듈 지도

현대식(2018+) 레이아웃: 폴더와 같은 이름의 `*.rs`가 모듈 루트, 하위 모듈은 같은 이름 폴더 안에 둔다.
**여러 제조사/모델**을 한 뷰어로 쓰려고, 모델별 코덱은 `<제조사>/<모델>/` 폴더에 두고
공용 계층(수집·뷰어)은 모델을 모른 채 트레잇으로만 다룬다.

```
src/
├── lib.rs              # 모듈 선언 + 공개 API 재노출
├── main.rs             # ldlidar : 콘솔 덤프 바이너리(STL-27L)
├── bin/viewer.rs       # viewer  : GUI 진입점 → viewer::run()
│
├── reader.rs           # [모듈 루트] 읽기·파싱·수집
├── reader/
│   ├── types.rs          #   [공용] LidarPoint / LidarBody / ParseError (모델 독립 도메인 타입)
│   ├── model.rs          #   [공용] Decoder 트레잇 + Model enum(설정명 → 코덱 팩토리)
│   ├── transport.rs      #   [공용] Transport 트레잇 + Serial/UDP 구현
│   ├── data_collector.rs #   [공용] 수집 스레드 + 데모 + 연결상태 + 회전 조립 → Scan publish
│   ├── LDROBOT/STL-27L/   #   [모델] 시리얼 47B 프레임
│   │   ├── frame.rs       #     47B 레이아웃 + zero-copy parse() + CRC8
│   │   └── decoder.rs     #     FrameDecoder : 바이트 스트림 → 프레임 절단(impl Decoder)
│   └── Pacecat/LDS-50C-E/ #   [모델] UDP 0xFAC7 패킷
│       ├── frame.rs       #     28B 헤더 + dist/ang/strength 배열 + 16-bit sum 파싱
│       ├── decoder.rs     #     PacketDecoder : 가변 길이 패킷 절단(impl Decoder)
│       └── command.rs     #     LVERSH/LSTARH 시작 커맨드(STM32 CRC32) + UDP 연결
│
├── viewer.rs           # [모듈 루트] egui_plot 뷰어 + run() (렌더링만, 모델 독립)
└── viewer/
    ├── config.rs       #   config.toml 스키마(+model/네트워크) + notify 핫리로드
    └── projection.rs   #   극좌표 Scan → 직교좌표 CartesianPoint (표현 변환)
```

> 폴더명에 하이픈(`STL-27L`)이 있어 Rust 식별자로 못 쓰므로, `reader.rs`에서 `#[path]`로
> 폴더를 잇고 모듈명은 `ldrobot::stl27l` / `pacecat::lds50ce`로 노출한다.

**책임 3계층**: ① **모델 코덱**(`LDROBOT/…`, `Pacecat/…` — 순수 파싱: 바이트⇄프레임/점, IO·스레드 없음)
→ ② **공용 수집기**(`data_collector` — `Transport`로 시리얼/UDP를, `Decoder`로 모델을 가린 채
IO·재연결·프레임 디코딩·회전 재구성 → 완성 `Scan` publish) → ③ **뷰어**(표현: 최신 `Scan` pull →
직교좌표 변환 → 렌더). 수집·뷰어는 모델을 전혀 모르므로, 새 모델은 코덱 폴더와 `Model` 한 줄만
추가하면 붙는다.

### 모델 추상화 — 두 개의 트레잇

| 트레잇 | 위치 | 역할 | 구현 |
|---|---|---|---|
| `Decoder` | `reader/model.rs` | 바이트 스트림 → 검증된 `LidarBody` 절단 | `FrameDecoder`(STL-27L), `PacketDecoder`(LDS-50C-E) |
| `Transport` | `reader/transport.rs` | "바이트를 읽는다" 한 가지 모양 | `SerialTransport`(UART), `UdpTransport`(소켓) |

`Model`(STL-27L/LDS-50C-E)이 설정 문자열을 받아 디코더를 만들고, `Source`(Serial/Udp/Demo)가
전송 파라미터를 들고 있다. `data_collector::pump`는 이 둘만 받아 돌리므로 모델·전송에 무관하다.

### 모델별 차이 한눈에

| | LDROBOT STL-27L | Pacecat LDS-50C-E |
|---|---|---|
| 전송 | 시리얼 921600 8N1 | UDP (host:port bind, sensor:6543 커맨드) |
| 시작 | 포트 열면 바로 스트림 | `LVERSH`→`LSTARH` 커맨드 필요(STM32 CRC32) |
| 프레임 | 고정 47B | 가변(28B 헤더 + N×5 + 2B 체크섬) |
| 동기/검증 | 헤더 `0x54`+버전 `0x2c`+CRC8 | 헤더 `0xFAC7`+16-bit sum |
| 점 배치 | `{거리,세기}` 12개 + 시작/끝각 보간 | SoA(거리 N·각도 N·세기 N), 절대각=(각도+시작각)×0.001° |

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
pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<LidarBody> {
    self.buffer.extend(bytes);
    let mut frames = Vec::new();
    while let Some(frame) = self.next_frame() {     // 헤더 0x54 + 버전 0x2c + CRC8 통과분만
        frames.push(frame);                         // CRC까지 검증된 프레임만 반환
    }
    frames
}
```

절단 규칙(**CRC 게이트 동기화**): ① 앞에서 `0x54`가 아닌 바이트를 버린다 → ② 두 번째 바이트가
`0x2c`가 아니면 그 `0x54`도 버린다 → ③ 헤더·버전이 맞으면 47B 창의 **CRC8까지 검증** → ④ CRC가
맞아야만 47B를 소비해 프레임으로 반환하고, **CRC가 틀리면 1바이트만 밀어 재탐색**한다(우연한
가짜 헤더 때문에 진짜 프레임을 통째로 까먹지 않도록). 버퍼가 한 프레임보다 짧으면 헤더 후보를
유지한 채 더 받을 때까지 보류한다.

> 디버그 빌드에서는 `data_collector.rs`가 수신 바이트 수와 프레임 ok/err(유형별) 통계를 1초마다
> 로그로 남긴다. `ok=0`인데 바이트는 들어오면 헤더/baud 불일치를 의심한다.

---

## 4. 프레임 → 뷰어 화면

### 4.1 전체 흐름

수집기 스레드가 점을 한 바퀴 `Scan`으로 재구성해 **"최신 한 장" 슬롯**에 publish하고, 뷰어는
매 프레임 그 슬롯에서 최신 스캔을 **pull**한다. 둘을 잇는 건 FIFO 큐가 아니라 latest-wins
슬롯이라, 렌더가 늦어도 밀린 점이 쌓이지 않는다(항상 최근 한 바퀴만 그림).

```
 [시리얼 /dev/ttyUSB0]
        │ 바이트
        ▼
 reader/data_collector.rs : 별도 스레드 (run_serial → pump_serial)
        │   FrameDecoder.push_bytes() → Vec<LidarBody>
        │   Assembler.ingest(point) : 각도 wrap 감지로 한 바퀴 조립
        │   완성되면 publish(Scan{ points, rotation })
        ▼
   Arc<Mutex<Option<Scan>>>  ──► ScanFeed { latest, status }
        │   (latest-wins 슬롯 — 큐 아님)        take_latest() / status()
        ▼
 viewer.rs : ViewerApp::ui()  (매 프레임)
        │   pull_latest_scan() : 새 스캔 있으면 교체(없으면 직전 유지)
        ▼
 viewer/projection.rs : project(&scan.points, max_range)
        │   극좌표 → Vec<CartesianPoint{x,y,intensity}>  (max_range 필터)
        ▼
 viewer.rs : draw_plot() → egui_plot (그리드·좌표축·줌·팬 내장)
                          draw_points() : 점 렌더 (config 색/크기)
```

### 4.2 수집기 (`reader/data_collector.rs`)

`reader::spawn(Source)`는 백그라운드 스레드를 띄우고 `ScanFeed`를 돌려준다.

```rust
pub struct Scan { pub points: Vec<LidarPoint>, pub rotation: u64 } // 한 바퀴(극좌표)

pub struct ScanFeed { /* latest: Arc<Mutex<Option<Scan>>>, status */ }
impl ScanFeed {
    pub fn take_latest(&self) -> Option<Scan>;     // 최신 스캔 꺼내기(없으면 None)
    pub fn status(&self) -> ConnectionStatus;      // 연결 상태 사본
}
```

- `run_serial` : 포트를 열어 `pump_serial`로 계속 읽는다. 실패하면 **데모로 조용히 바꾸지 않고**
  상태를 `Error(사유)`로 알린 뒤 1초마다 재연결을 시도한다. (데모는 `demo = true`일 때만.)
- `pump_serial` : `FrameDecoder.push_bytes()`로 프레임을 얻고, 각 `body.points`를 `Assembler`에
  먹인다. 한 바퀴가 완성되면 `Scan`을 슬롯에 publish(직전 미소비분은 덮어씀 = latest-wins).
- 무프레임 워치독(1.5초)·열기 직후 입력버퍼 flush 등 견고화는 그대로.

### 4.3 회전 재구성 (`Assembler`, `data_collector.rs` 내부)

`LidarPoint`(각도°, 거리mm) 스트림을 **한 바퀴 단위로 재구성**한다. 각도가 단조 증가하다
절반 바퀴(180°)를 넘게 되감기면 0°를 막 지난 것 = 한 바퀴 완성으로 보고 모은 점들을 통째로
돌려준다(`std::mem::take`). 거리 0(무효)은 버린다.

```rust
fn ingest(&mut self, point: LidarPoint) -> Option<Vec<LidarPoint>>;
// 완성된 회전의 점들(Some) 또는 None
```

단순 `angle < prev`가 아니라 **180° 임계**인 이유: 노이즈로 인한 미세 후진(`90.2°→90.1°`)을
무시하고 진짜 0° 통과(≈356° 급락)만 잡으려고. `MAX_SCAN_POINTS`로 wrap이 안 잡히는 비정상
스트림의 메모리 폭주를 막는다.

> 이 "재구성"이 핵심 책임 이동이다. 예전엔 뷰어의 UI 스레드에서 점을 누적했지만, 이제 수집
> 계층이 회전을 조립해 완성된 스캔만 넘긴다.

### 4.4 표현 변환 (`viewer/projection.rs`)

극→직교 변환과 `max_range` 필터는 **렌더링 준비**이고 `max_range`가 뷰어 설정이라, 수집이 아닌
뷰어에 둔다(수집 계층은 표현·config를 모름).

```
x = (distance_mm/1000) · cos(angle)
y = (distance_mm/1000) · sin(angle)   // 0° = +x축, 반시계 = +각도
```

### 4.5 렌더와 설정 (`viewer.rs` + `viewer/config.rs`)

- `ViewerApp`은 최신 `Scan`과 `ViewerConfig`만 들고 있다. `ui()`가 매 프레임 `drain_config()`
  (핫리로드) → `pull_latest_scan()`(최신 스캔 교체) → `project()` → `draw_plot()` 순으로 실행.
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
| 수집 | `reader/data_collector.rs` | 프레임 → `Scan`(한 바퀴) publish | 스레드 + 재연결 + 회전 조립 |
| 핸드오프 | `ScanFeed` | `Scan` 슬롯 ←pull→ 뷰어 | latest-wins (큐 아님) |
| 표현 | `viewer/projection.rs` | 극좌표 → 직교좌표 | `max_range` 필터 |
| 렌더 | `viewer.rs` | `CartesianPoint` → 화면 | egui_plot + config (그리기만) |
