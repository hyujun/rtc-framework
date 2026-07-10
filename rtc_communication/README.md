# rtc_communication

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **헤더 전용(header-only) C++ 라이브러리**로, 실시간(RT-safe) 네트워크 전송 추상화를 제공합니다. UDP·CAN·CAN FD 전송, 패킷 코덱, 전이중 트랜시버 템플릿으로 구성되어 있으며, 실시간 제어 루프에서 외부 디바이스와의 통신에 사용됩니다.

**설계 원칙:**
- 생성 이후 동적 메모리 할당 없음 (RT-safe)
- 모든 I/O 메서드 `noexcept` 보장
- RAII 기반 소켓 수명 관리
- C++20 기능 활용: `concept`, `std::jthread`, `std::stop_token`, `std::span`
- `[[likely]]`/`[[unlikely]]` 분기 힌트로 수신 루프 최적화

---

## 패키지 구조

```
rtc_communication/
├── CMakeLists.txt
├── package.xml
├── include/rtc_communication/
│   ├── transport_interface.hpp    -- 전송 계층 추상 인터페이스
│   ├── packet_codec.hpp          -- 패킷 코덱 C++20 Concept + 헬퍼
│   ├── transceiver.hpp           -- 전이중 트랜시버 템플릿 (수신 루프 + 코덱)
│   ├── udp/
│   │   ├── udp_socket.hpp        -- RAII UDP 소켓 래퍼
│   │   └── udp_transport.hpp     -- TransportInterface의 UDP 구현체
│   └── can/
│       ├── can_socket.hpp        -- RAII SocketCAN raw 소켓 래퍼 (classic + FD frame I/O)
│       ├── can_transport.hpp     -- TransportInterface의 classic CAN 구현체 (payload ≤ 8B)
│       └── canfd_transport.hpp   -- TransportInterface의 CAN FD 구현체 (payload ≤ 64B)
└── test/
    ├── fake_codec.hpp            -- PacketCodec concept을 만족하는 최소 코덱 (테스트 하니스용)
    ├── test_udp_loopback.cpp     -- UdpSocket/UdpTransport 바인드·연결·타임아웃·RAII 테스트
    ├── test_can_loopback.cpp     -- CanSocket/CanTransport vcan0 라운드 트립·필터·RAII 테스트
    ├── test_canfd_loopback.cpp   -- CanFdTransport 64B 라운드 트립·classic/FD 혼재 수신 테스트
    └── test_transceiver.cpp      -- Transceiver 기동·종료·디코딩·콜백·Send 경로 테스트
```

---

## 주요 컴포넌트

### TransportInterface (`transport_interface.hpp`)

모든 전송 백엔드의 추상 기반 클래스입니다. `rtc` 네임스페이스에 정의되어 있습니다.

| 메서드 | 반환 | 설명 |
|--------|------|------|
| `Open()` | `bool` | 전송 채널 열기 (`[[nodiscard]]`) |
| `Close()` | `void` | 채널 닫기 (`noexcept`) |
| `Send(span<const uint8_t>)` | `ssize_t` | 데이터 송신 (`noexcept`, `[[nodiscard]]`) |
| `Recv(span<uint8_t>)` | `ssize_t` | 데이터 수신 (`noexcept`, `[[nodiscard]]`) |
| `SetRecvTimeout(int ms)` | `void` | 수신 타임아웃 설정 (`noexcept`) |
| `SetRecvBufferSize(int)` | `void` | 수신 버퍼 크기 설정 (`noexcept`) |
| `is_open()` | `bool` | 열림 상태 조회 (`const noexcept`, `[[nodiscard]]`) |

- Non-copyable, non-movable
- 생성자는 `protected` (직접 인스턴스화 불가)

---

### UdpSocket (`udp/udp_socket.hpp`)

raw UDP 소켓(`AF_INET`, `SOCK_DGRAM`)의 RAII 래퍼입니다. 파일 디스크립터를 소유하며 소멸자에서 `close()`를 호출합니다.

**두 가지 사용 모드:**

| 모드 | 설정 | I/O |
|------|------|-----|
| **수신 (Bind)** | `Bind(address, port)` -- 지정 주소:포트에 바인드 (`"0.0.0.0"` 또는 빈 문자열이면 `INADDR_ANY`) | `Recv(span<uint8_t>)` |
| **송신 (Connect)** | `Connect(ip, port)` -- 대상 주소 저장 (`sendto` 사용) | `Send(span<const uint8_t>)` |

주요 특징:
- `Bind()`는 `std::string_view` 주소와 포트를 매개변수로 받음. `"0.0.0.0"` 또는 빈 문자열은 모든 인터페이스(`INADDR_ANY`)에 바인드
- `Recv()`는 `std::span<uint8_t>`를 매개변수로 받음
- `Send()`는 `std::span<const uint8_t>`를 매개변수로 받음
- `Connect()`는 `std::string_view`를 받되, 내부적으로 null 종료 문자열로 변환하여 `inet_pton` 호출
- `SetRecvBufferSize(int bytes)` -- 소켓 수신 버퍼 크기(`SO_RCVBUF`) 설정 (`noexcept`). `fd` 가 열려 있지 않으면 아무 동작 없음
- `is_open()` -- 소켓이 열려 있는지 조회 (`fd_ >= 0`). `[[nodiscard]] bool const noexcept`
- `fd()` 접근자를 통해 raw 파일 디스크립터 조회 가능
- Non-copyable, non-movable

```cpp
// 수신 모드
UdpSocket rx;
rx.Open();
rx.Bind("0.0.0.0", 9000);  // 모든 인터페이스에 바인드
rx.SetRecvTimeout(100);  // 100ms
uint8_t buf[1024];
auto n = rx.Recv({buf, sizeof(buf)});

// 송신 모드
UdpSocket tx;
tx.Open();
tx.Connect("192.168.1.100", 9000);
std::array<uint8_t, 64> data{};
tx.Send({data.data(), data.size()});
```

---

### UdpTransport (`udp/udp_transport.hpp`)

`TransportInterface`의 UDP 구현체입니다. 내부적으로 송신용/수신용 `UdpSocket`을 각각 관리합니다.

#### 설정 (`UdpTransportConfig`)

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `bind_address` | `string` | `"0.0.0.0"` | 수신 바인드 주소. `"0.0.0.0"` 또는 빈 문자열이면 모든 인터페이스(`INADDR_ANY`)에 바인드 |
| `bind_port` | `int` | `0` | 수신 포트 (0이면 수신 소켓 미생성) |
| `target_address` | `string` | `""` | 송신 대상 IP (빈 문자열이면 송신 소켓 미생성) |
| `target_port` | `int` | `0` | 송신 대상 포트 |
| `recv_buffer_size` | `int` | `256 * 1024` | 소켓 수신 버퍼 크기 (`SO_RCVBUF`) |
| `recv_timeout_ms` | `int` | `100` | 수신 타임아웃 (`SO_RCVTIMEO`) |

- `Open()`에서 `bind_port > 0`이면 수신 소켓을 바인드하고, `target_address`가 비어있지 않고 `target_port > 0`이면 송신 소켓을 연결
- `is_open()`은 수신 또는 송신 소켓 중 하나라도 열려 있으면 `true` 반환
- `recv_socket()`과 `send_socket()` 접근자를 통해 하위 `UdpSocket`에 직접 접근 가능 (레거시 코드 마이그레이션용)

```cpp
UdpTransportConfig config{
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

### CanSocket (`can/can_socket.hpp`)

Linux SocketCAN raw 소켓(`PF_CAN`, `SOCK_RAW`, `CAN_RAW`)의 RAII 래퍼입니다. UDP와 달리 **인터페이스에 바인드한 단일 fd로 송수신을 모두** 수행합니다.

주요 특징:
- `Bind(interface_name)` -- `if_nametoindex()`로 인터페이스 이름(`"can0"`, `"vcan0"`)을 index로 변환 후 `sockaddr_can`에 바인드. 존재하지 않는 인터페이스면 `false` 반환 + 소켓 닫힘
- `SetFilter(can_id, can_mask)` -- `CAN_RAW_FILTER` 수신 필터. matching: `(recv_id & mask) == (can_id & mask)`
- `SetLoopback(bool)` / `SetRecvOwnMessages(bool)` -- `CAN_RAW_LOOPBACK`(커널 기본 on) / `CAN_RAW_RECV_OWN_MSGS`(커널 기본 off)
- `SetFdFrames(bool)` -- `CAN_RAW_FD_FRAMES` 활성화. 활성 시 classic·FD frame 모두 수신 가능
- `SendFrame`/`RecvFrame` -- `can_frame` 단위 I/O. `SendFdFrame`/`RecvFdFrame` -- `canfd_frame` 단위 I/O
- `SetRecvTimeout`/`SetRecvBufferSize`, `fd()`, `is_open()` -- UdpSocket과 동일 계약
- Non-copyable, non-movable, 소멸자에서 fd `close()`

---

### CanTransport (`can/can_transport.hpp`)

`TransportInterface`의 classic CAN 구현체입니다. **payload-only 계약**: `Send(span)`은 payload를 설정된 `tx_can_id`의 단일 frame으로 송신하고, `Recv(span)`은 필터를 통과한 frame의 payload만 반환합니다. frame metadata(CAN ID, flags)는 config에 고정되므로 인스턴스 하나가 tx ID 하나 / rx 필터 하나에 대응합니다.

#### 설정 (`CanTransportConfig`)

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `interface_name` | `string` | `"can0"` | CAN 인터페이스 이름 |
| `tx_can_id` | `canid_t` | `0` | 송신 frame의 CAN ID |
| `rx_can_id` | `canid_t` | `0` | 수신 필터 ID |
| `rx_can_mask` | `canid_t` | `0` | 수신 필터 mask. **0이면 필터 미설치(전체 수신, 커널 기본)** — 이때 `rx_can_id`는 효력 없음. 설치 시 EFF/RTR 비트가 mask에 자동 포함. RTR frame은 필터와 무관하게 `Recv()`에서 항상 drop |
| `extended_frame` | `bool` | `false` | `true`면 29-bit ID (`CAN_EFF_FLAG`) 송신 및 필터 적용 |
| `receive_own_messages` | `bool` | `false` | 자기 송신 frame 수신 (`CAN_RAW_RECV_OWN_MSGS`) |
| `loopback` | `bool` | `true` | 동일 호스트 다른 소켓으로 로컬 echo (`CAN_RAW_LOOPBACK`) |
| `recv_timeout_ms` | `int` | `100` | 수신 타임아웃 (`SO_RCVTIMEO`) |

- **payload는 최대 8 bytes (`CAN_MAX_DLEN`)** — 초과 시 `Send()`가 `-1` 반환
- `is_open()`은 단일 소켓 상태 반환 (UDP의 recv/send 2-소켓 구조와 다름)

```cpp
CanTransportConfig config{
    .interface_name = "can0",
    .tx_can_id = 0x123,
    .rx_can_id = 0x456,
    .rx_can_mask = CAN_SFF_MASK,
    .recv_timeout_ms = 100
};
auto transport = std::make_unique<CanTransport>(config);
transport->Open();
```

---

### CanFdTransport (`can/canfd_transport.hpp`)

`TransportInterface`의 CAN FD 구현체입니다. `CanTransport`와 동일한 payload-only 계약에 다음이 추가됩니다.

- **payload 최대 64 bytes (`CANFD_MAX_DLEN`)** — 초과 시 `Send()`가 `-1` 반환
- `Open()`에서 `CAN_RAW_FD_FRAMES`를 활성화 — 소켓이 classic frame(`CAN_MTU`)과 FD frame(`CANFD_MTU`)을 **모두 수신**하며 `Recv()`가 양쪽을 처리
- 비표준 payload 길이(CAN FD DLC 집합 0-8/12/16/20/24/32/48/64 외)는 실제 컨트롤러 송신 시 커널이 상향 패딩

#### 설정 (`CanFdTransportConfig`)

`CanTransportConfig`와 동일 필드에 FD 전용 2개 추가:

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `bitrate_switch` | `bool` | `false` | `CANFD_BRS` — 데이터 구간을 높은 bitrate로 송신 |
| `error_state_indicator` | `bool` | `false` | `CANFD_ESI` flag |

#### PacketCodec 크기 제약 (중요)

`Transceiver<Codec>::Send`는 `sizeof(SendPacket)` 전체를 한 번에 `transport_->Send()`로 넘기고, 수신 루프는 `n < sizeof(RecvPacket)`인 frame을 버립니다. 따라서 CAN 계열 transport 위에서 쓰는 codec은 **packet 타입 크기가 payload 한도 안**이어야 합니다:

| Transport | `sizeof(SendPacket)` / `sizeof(RecvPacket)` 한도 |
|-----------|--------------------------------------------------|
| `CanTransport` | ≤ 8 bytes |
| `CanFdTransport` | ≤ 64 bytes |

한도 초과 시 `Transceiver::Send`는 조용히 `false`를 반환합니다 (transport guard가 `-1` 리턴). multi-frame 분할이나 frame별 CAN ID 해석이 필요한 프로토콜은 payload-only 계약 범위 밖입니다 (frame-aware API는 후속 확장).

---

### PacketCodec Concept (`packet_codec.hpp`)

사용자 정의 패킷 코덱을 위한 C++20 Concept입니다.

**Concept 요구 사항:**

| 타입/메서드 | 제약 | 설명 |
|------------|------|------|
| `Codec::RecvPacket` | `trivially_copyable` | 수신 패킷 와이어 포맷 |
| `Codec::SendPacket` | `trivially_copyable` | 송신 패킷 와이어 포맷 |
| `Codec::State` | -- | 디코딩된 애플리케이션 상태 타입 |
| `Codec::Decode(span<const uint8_t>, State&)` | `static`, 반환 `bool` | **필수.** 바이트 버퍼를 State로 디코딩 |

`Encode`는 Concept에서 요구하지 않습니다. 인코딩이 필요한 경우 `EncodePacket<T>()` 헬퍼를 사용할 수 있습니다.

**헬퍼 함수:**

| 함수 | 설명 |
|------|------|
| `DecodePacket<T>(span<const uint8_t>, T&) -> bool` | `memcpy` 기반 디코더. 버퍼가 `sizeof(T)` 미만이면 `false` 반환. `trivially_copyable` 제약. |
| `EncodePacket<T>(const T&, span<uint8_t, sizeof(T)>) -> void` | `memcpy` 기반 인코더. 고정 크기 `span` 사용. `trivially_copyable` 제약. |

```cpp
// 코덱 정의 예시
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

`TransportInterface`와 `PacketCodec`를 결합한 전이중 트랜시버 클래스 템플릿입니다.

**주요 특징:**
- `TransportInterface`를 `unique_ptr`로 소유
- `std::jthread`로 전용 수신 루프 실행
- 최신 디코딩 상태를 `mutex`로 보호하여 스레드 안전하게 제공
- 선택적 상태 변경 콜백 지원
- 송신 경로는 스택 배열을 사용하여 할당 없음

| 메서드 | 시그니처 | 설명 |
|--------|---------|------|
| `StartRecv()` | `[[nodiscard]] bool` | 전송 `Open()` + jthread 수신 루프 시작 |
| `Stop()` | `void noexcept` | `running_` 해제 + `request_stop()` + 전송 `Close()` |
| `Send()` | `[[nodiscard]] bool noexcept` | 스택 배열로 `memcpy` 후 전송 (`SendPacket` 크기) |
| `GetLatestState()` | `[[nodiscard]] State const` | `mutex` 잠금 후 최신 상태 복사본 반환 |
| `SetCallback()` | `void noexcept` | 상태 변경 콜백 등록 (`std::function<void(const State&)>`) |
| `IsRunning()` | `[[nodiscard]] bool const noexcept` | `running_` 원자적 조회 (`memory_order_acquire`) |
| `recv_count()` | `[[nodiscard]] size_t const noexcept` | 수신 패킷 수 (`relaxed`) |
| `send_count()` | `[[nodiscard]] size_t const noexcept` | 송신 성공 패킷 카운터 (`Send()` 성공 시 증가) |
| `transport()` | `TransportInterface* const noexcept` | 하위 전송 계층 포인터 반환 |

생성자: `Transceiver(unique_ptr<TransportInterface>, const ThreadConfig& = kRtUdpRecvConfig)`

```cpp
auto transport = std::make_unique<UdpTransport>(config);
Transceiver<MyCodec> xcvr(std::move(transport));

xcvr.SetCallback([](const MyCodec::State& state) {
    // 새 데이터 수신 시 호출 (mutex 해제 후 호출됨)
});

xcvr.StartRecv();

// 송신
MyCodec::SendPacket cmd{};
xcvr.Send(cmd);

// 최신 상태 조회
auto state = xcvr.GetLatestState();
```

#### 수신 루프 (`RecvLoop`) 상세

`RecvLoop`는 `ApplyThreadConfig()`를 호출하여 RT 스레드 설정을 적용한 뒤, 다음을 반복합니다:

```
while (!stop_requested):
  1. transport_->Recv(buffer)     -- 커널 블로킹 (SO_RCVTIMEO)
  2. 수신 크기 < 0 이면 continue   -- 타임아웃 또는 전송 닫힘
  3. 수신 크기 < sizeof(RecvPacket) 이면 continue
  4. Codec::Decode(buffer, state)  -- 실패 시 무시
  5. lock_guard(data_mutex_)       -- latest_state_ 갱신 (mutex 범위 종료)
  6. callback_(state)              -- 콜백 호출 (mutex 해제 후)
  7. recv_count_++                 -- atomic (relaxed)
```

주요 사항:
- 수신 버퍼 크기: `sizeof(RecvPacket) + 64` (오버사이즈 데이터그램 감지용)
- 콜백은 `data_mutex_` 해제 후 호출됨
- 디코딩 성공 시에만 `recv_count_` 증가
- `Stop()` 호출 시 소켓 타임아웃만큼 지연 가능 (기본 100ms)

#### 메모리 순서 보장

| 원자적 변수 | 쓰기 | 읽기 | 용도 |
|------------|------|------|------|
| `running_` | `release` | `acquire` | 셧다운 동기화 |
| `recv_count_` | `relaxed` | `relaxed` | 통계 |
| `send_count_` | `relaxed` | `relaxed` | 통계 (`Send()` 성공 시 증가) |

#### 호출 컨텍스트 (RT vs non-RT)

`Transceiver`의 메서드는 RT-안전성 보장이 다릅니다. 호출 컨텍스트에 맞게 사용해야 합니다.

| 메서드 | RT-safe? | 사유 |
|--------|----------|------|
| `Send(pkt)` | ✅ Yes | 스택 memcpy + 단일 `sendto()` syscall, `noexcept`. RT 정기 tick 루프 (default 500 Hz, `control_rate`로 가변)에서 직접 호출 가능 |
| `IsRunning()` / `recv_count()` / `send_count()` | ✅ Yes | atomic load만 수행 |
| `GetLatestState()` | ❌ **No** | `std::mutex` 획득 — non-RT 컨텍스트(진단 스레드, ROS2 콜백)에서만 호출 |
| `StartRecv()` / `Stop()` / `SetCallback()` | ❌ No | 초기화/셧다운 경로 (jthread 생성·조인) |

RT 루프에서 디코딩된 상태가 필요하면 `GetLatestState()`를 직접 호출하지 말고, 비-RT 스레드(예: 센서 callback group)에서 한 번 읽어 `SeqLock<State>` 또는 SPSC 큐로 RT 루프에 전달하는 패턴을 사용하세요.

---

## 테스트

| 테스트 파일 | 프레임워크 | 다루는 항목 |
|------------|-----------|------------|
| `test/test_udp_loopback.cpp` | GTest | UdpSocket bind/connect 라운드 트립, `SO_RCVTIMEO` 만료, RAII fd 닫힘, UdpTransport bind+connect 수명 (5 케이스) |
| `test/test_can_loopback.cpp` | GTest | CanSocket/CanTransport 수명, invalid interface, vcan0 라운드 트립, 타임아웃, RAII, >8B 거부, 필터 accept/reject, extended ID, RTR drop, receive_own_messages (12 케이스, vcan 의존 8개는 guarded) |
| `test/test_canfd_loopback.cpp` | GTest | CanFdTransport invalid interface, 64B 라운드 트립, >64B 거부, classic/FD 혼재 수신 (4 케이스, vcan 의존 3개는 guarded) |
| `test/test_transceiver.cpp` | GTest | `Transceiver<FakeCodec>` 기동·종료, 외부 송신 디코딩, 콜백 호출, 짧은 데이터그램 무시, Send 경로 외부 수신자 도달 (4 케이스) |
| `test/fake_codec.hpp` | -- | `PacketCodec` concept을 만족하는 최소 코덱 (테스트 하니스용) |

```bash
colcon test --packages-select rtc_communication --event-handlers console_direct+
```

### CAN 테스트 환경 (vcan)

CAN/CANFD 통합 테스트는 가상 CAN 인터페이스 `vcan0`이 있어야 실행되며, 없으면 `GTEST_SKIP()`으로 건너뜁니다 (CI green 유지):

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

수동 smoke test는 [can-utils](https://github.com/linux-can/can-utils) 사용 (`sudo apt install can-utils`):

```bash
candump vcan0          # 수신 모니터
cansend vcan0 123#11223344
cangen vcan0           # 랜덤 트래픽 생성
canfdtest vcan0        # CAN FD 검증
```

---

## 의존성

| 의존성 | 유형 | 용도 |
|--------|------|------|
| `ament_cmake` | 빌드 도구 | ROS 2 빌드 시스템 |
| `rtc_base` | 런타임 | `ThreadConfig`, `ApplyThreadConfig` (스레드 설정) |
| `ament_cmake_gtest` | 테스트 | UDP/CAN 루프백 / Transceiver 통합 테스트 |

**시스템 의존성:** POSIX 소켓 API (`sys/socket.h`, `arpa/inet.h`, `netinet/in.h`, `unistd.h`), Linux SocketCAN 커널 헤더 (`linux/can.h`, `linux/can/raw.h`, `net/if.h`) — 별도 패키지 설치 불필요

**컴파일러 요구 사항:** C++20 (`-std=c++20`), 엄격 경고 플래그 (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`)

---

## 빌드

```bash
colcon build --packages-select rtc_communication
```

헤더 전용 라이브러리이므로 컴파일되는 바이너리는 없습니다. 다른 패키지에서 `find_package(rtc_communication REQUIRED)`로 의존성을 추가하면 include 경로가 자동으로 설정됩니다.

---

## 라이선스

MIT License
