# rtc_communication

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **헤더 전용(header-only) 통신 추상화 계층**입니다. 실시간 제어 루프에서 외부 디바이스와의 네트워크 통신을 위한 전송 인터페이스, UDP 소켓 래퍼, 패킷 코덱 개념, 전이중 트랜시버를 제공합니다.

**핵심 설계 원칙:**
- 생성 후 동적 할당 없음 — RT-safe I/O
- 모든 I/O 메서드 `noexcept` 보장
- RAII 기반 소켓 수명 관리
- C++20 Concept 기반 코덱 추상화
- 향후 CAN-FD, EtherCAT, RS485 확장 지원

---

## 패키지 구조

```
rtc_communication/
├── CMakeLists.txt
├── package.xml
└── include/rtc_communication/
    ├── transport_interface.hpp    ← 전송 계층 추상 인터페이스
    ├── packet_codec.hpp          ← 패킷 코덱 C++20 Concept + 헬퍼
    ├── transceiver.hpp           ← 전이중 트랜시버 (수신 루프 + 코덱)
    └── udp/
        ├── udp_socket.hpp        ← RAII UDP 소켓 래퍼
        └── udp_transport.hpp     ← TransportInterface UDP 구현체
```

---

## 아키텍처

```
┌─────────────────────────────────────────────────────┐
│              Transceiver<Codec>                      │
│  (템플릿 기반 전이중 통신 — 수신 루프 + 상태 스냅샷)   │
└────────────┬────────────────────────────────────────┘
             │ 소유 (unique_ptr)
             ▼
┌────────────────────────────────┐
│   TransportInterface (추상)    │
│   Open / Close / Send / Recv   │
└────────────┬───────────────────┘
             │ 구현
             ▼
┌────────────────────────────────┐
│     UdpTransport (구현체)       │
│   송신/수신 소켓 분리 관리       │
└────────────┬───────────────────┘
             │ 사용
             ▼
┌────────────────────────────────┐
│      UdpSocket (RAII 래퍼)     │
│   AF_INET, SOCK_DGRAM          │
│   Bind (수신) / Connect (송신)  │
└────────────────────────────────┘
```

---

## 컴포넌트 상세

### TransportInterface (`transport_interface.hpp`)

모든 전송 백엔드의 추상 기반 클래스입니다.

| 메서드 | 반환 | 설명 |
|--------|------|------|
| `Open()` | `bool` | 전송 채널 열기 |
| `Close()` | `void` | 채널 닫기 (`noexcept`) |
| `Send(span<const uint8_t>)` | `ssize_t` | 데이터 송신 (`noexcept`) |
| `Recv(span<uint8_t>)` | `ssize_t` | 데이터 수신 (`noexcept`) |
| `SetRecvTimeout(int ms)` | `void` | 수신 타임아웃 설정 (`noexcept`) |
| `SetRecvBufferSize(int)` | `void` | 수신 버퍼 크기 설정 (`noexcept`) |
| `is_open()` | `bool` | 열림 상태 조회 (`noexcept`) |

- Non-copyable, non-movable
- 모든 반환값 `[[nodiscard]]`

---

### UdpSocket (`udp/udp_socket.hpp`)

raw UDP 소켓의 RAII 래퍼입니다. 파일 디스크립터를 소유하며 소멸자에서 자동 해제합니다.

**두 가지 동작 모드:**

| 모드 | 설정 | I/O |
|------|------|-----|
| **수신 (Bind)** | `Bind(port)` → INADDR_ANY:port | `Recv()` |
| **송신 (Connect)** | `Connect(ip, port)` → 대상 주소 저장 | `Send()` |

```cpp
// 수신 모드
UdpSocket rx;
rx.Open();
rx.Bind(9000);
rx.SetRecvTimeout(100);  // 100ms 타임아웃
char buf[1024];
auto n = rx.Recv({buf, sizeof(buf)});

// 송신 모드
UdpSocket tx;
tx.Open();
tx.Connect("192.168.1.100", 9000);
tx.Send({data.data(), data.size()});
```

---

### UdpTransport (`udp/udp_transport.hpp`)

`TransportInterface`의 UDP 구현체입니다. 송신/수신 소켓을 별도로 관리합니다.

#### 설정 (`UdpTransportConfig`)

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `bind_address` | `string` | `"0.0.0.0"` | 수신 바인드 주소 |
| `bind_port` | `int` | `0` | 수신 포트 |
| `target_address` | `string` | — | 송신 대상 IP |
| `target_port` | `int` | `0` | 송신 대상 포트 |
| `recv_buffer_size` | `int` | `256 KB` | 소켓 수신 버퍼 (SO_RCVBUF) |
| `recv_timeout_ms` | `int` | `100` | 수신 타임아웃 (SO_RCVTIMEO) |

```cpp
UdpTransportConfig config{
    .bind_address = "0.0.0.0",
    .bind_port = 9000,
    .target_address = "192.168.1.100",
    .target_port = 9001,
    .recv_buffer_size = 256 * 1024,
    .recv_timeout_ms = 100
};
auto transport = std::make_unique<UdpTransport>(config);
transport->Open();
```

---

### PacketCodec Concept (`packet_codec.hpp`)

사용자 정의 패킷 코덱을 위한 C++20 Concept입니다.

**요구 사항:**

| 타입/메서드 | 제약 | 설명 |
|------------|------|------|
| `Codec::RecvPacket` | `trivially_copyable` | 수신 패킷 와이어 포맷 |
| `Codec::SendPacket` | `trivially_copyable` | 송신 패킷 와이어 포맷 |
| `Codec::State` | — | 디코딩된 애플리케이션 상태 |
| `Codec::Decode(span, State&)` | static | 바이트 → State 파싱 |

**헬퍼 함수:**

| 함수 | 설명 |
|------|------|
| `DecodePacket<T>(span, T&)` | memcpy 기반 제네릭 디코더 (`trivially_copyable` 제약) |
| `EncodePacket<T>(T&, span)` | memcpy 기반 제네릭 인코더 (`trivially_copyable` 제약) |

```cpp
// 사용자 코덱 정의 예시
struct MyCodec {
    struct RecvPacket { float data[6]; } __attribute__((packed));
    struct SendPacket { float cmd[6]; } __attribute__((packed));
    struct State { std::array<float, 6> values; bool valid; };

    static bool Decode(std::span<const uint8_t> buf, State& out) {
        RecvPacket pkt;
        if (!DecodePacket(buf, pkt)) return false;
        std::copy_n(pkt.data, 6, out.values.begin());
        out.valid = true;
        return true;
    }
};
```

---

### Transceiver (`transceiver.hpp`)

`TransportInterface` + `PacketCodec`를 결합한 전이중 트랜시버 템플릿입니다.

**주요 특징:**
- `TransportInterface` 소유 (unique_ptr)
- `std::jthread`로 전용 수신 루프 실행
- 최신 디코딩 상태를 스레드 안전하게 제공
- 선택적 상태 변경 콜백 지원
- 할당 없는 송신 경로

| 메서드 | 설명 |
|--------|------|
| `StartRecv()` | 전송 열기 + jthread 수신 루프 시작 |
| `Stop()` | 수신 루프 중지 + 전송 닫기 |
| `Send(SendPacket&)` | 할당 없는 패킷 송신 (`noexcept`) |
| `GetLatestState()` | 최신 디코딩 상태 스냅샷 (mutex 보호) |
| `SetCallback(fn)` | 상태 변경 콜백 등록 |
| `recv_count()` | 수신 패킷 수 (atomic) |
| `send_count()` | 송신 패킷 수 (atomic) |

```cpp
auto transport = std::make_unique<UdpTransport>(config);
Transceiver<MyCodec> xcvr(std::move(transport), kUdpRecvConfig);

xcvr.SetCallback([](const MyCodec::State& state) {
    // 새 데이터 수신 시 호출
});

xcvr.StartRecv();  // jthread 시작 (SCHED_FIFO, Core 5)

// 송신
MyCodec::SendPacket cmd{};
xcvr.Send(cmd);  // RT-safe

// 최신 상태 조회
auto state = xcvr.GetLatestState();
```

---

## 스레딩 모델

수신 루프는 `rtc_base`의 `ThreadConfig`를 사용하여 RT 스레드로 실행됩니다.

| 항목 | 기본값 |
|------|--------|
| CPU 코어 | Core 5 (6코어 기준) |
| 스케줄러 | SCHED_FIFO |
| 우선순위 | 65 |
| 스레드 이름 | `udp_recv` |

> `Transceiver` 생성자에서 `ThreadConfig`를 지정하여 커스텀 스레드 레이아웃 적용이 가능합니다.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_base` | 스레드 구성 (`ThreadConfig`, `ApplyThreadConfig`) |

**시스템 의존성:** POSIX 소켓 API (`sys/socket.h`, `arpa/inet.h`, `netinet/in.h`)

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_communication
source install/setup.bash
```

헤더 전용 라이브러리이므로 컴파일되는 바이너리는 없습니다.

---

## 의존성 그래프 내 위치

```
rtc_base  ← 스레드 구성 제공
    ↓
rtc_communication  ← 전송 추상화 계층
    ↑
    ├── rtc_controller_manager   (네트워크 통신)
    └── ur5e_hand_driver         (핸드 UDP 통신)
```

### 데이터 흐름 예시 (핸드 드라이버)

```
ur5e_hand_driver
    ↓ Transceiver<HandUdpCodec> 생성
    ↓ StartRecv() → jthread 수신 루프 (Core 5, SCHED_FIFO 65)
    │
    ├─→ RecvLoop: UDP 수신 → HandUdpCodec::Decode → HandState 스냅샷 갱신
    │
    └─→ Send: 모터 커맨드 인코딩 → UDP 송신 (할당 없음)
```

---

## 향후 확장

`TransportInterface` 추상화를 통해 추가 전송 백엔드를 지원할 수 있습니다:

| 백엔드 | 상태 | 용도 |
|--------|------|------|
| UDP | **구현 완료** | 핸드 센서/모터 통신 |
| CAN-FD | 계획 | 산업용 필드버스 |
| EtherCAT | 계획 | 고속 산업용 이더넷 |
| RS485 | 계획 | 시리얼 통신 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
