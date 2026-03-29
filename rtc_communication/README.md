# rtc_communication

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **헤더 전용(header-only) C++ 라이브러리**로, 실시간(RT-safe) 네트워크 전송 추상화를 제공합니다. UDP 전송, 패킷 코덱, 전이중 트랜시버 템플릿으로 구성되어 있으며, 실시간 제어 루프에서 외부 디바이스와의 통신에 사용됩니다.

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
└── include/rtc_communication/
    ├── transport_interface.hpp    -- 전송 계층 추상 인터페이스
    ├── packet_codec.hpp          -- 패킷 코덱 C++20 Concept + 헬퍼
    ├── transceiver.hpp           -- 전이중 트랜시버 템플릿 (수신 루프 + 코덱)
    └── udp/
        ├── udp_socket.hpp        -- RAII UDP 소켓 래퍼
        └── udp_transport.hpp     -- TransportInterface의 UDP 구현체
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
| **수신 (Bind)** | `Bind(port)` -- `INADDR_ANY:port`에 바인드 | `Recv(span<char>)` |
| **송신 (Connect)** | `Connect(ip, port)` -- 대상 주소 저장 (`sendto` 사용) | `Send(span<const uint8_t>)` |

주요 특징:
- `Recv()`는 `std::span<char>`를 매개변수로 받음
- `Send()`는 `std::span<const uint8_t>`를 매개변수로 받음
- `Connect()`는 `std::string_view`를 받되, 내부적으로 null 종료 문자열로 변환하여 `inet_pton` 호출
- `fd()` 접근자를 통해 raw 파일 디스크립터 조회 가능
- Non-copyable, non-movable

```cpp
// 수신 모드
UdpSocket rx;
rx.Open();
rx.Bind(9000);
rx.SetRecvTimeout(100);  // 100ms
char buf[1024];
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
| `bind_address` | `string` | `"0.0.0.0"` | 수신 바인드 주소 (현재 `Bind()` 호출 시 미사용, `INADDR_ANY` 고정) |
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
| `send_count()` | `[[nodiscard]] size_t const noexcept` | 송신 패킷 카운터 (현재 증가 로직 미구현) |
| `transport()` | `TransportInterface* const noexcept` | 하위 전송 계층 포인터 반환 |

생성자: `Transceiver(unique_ptr<TransportInterface>, const ThreadConfig& = kUdpRecvConfig)`

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
| `send_count_` | `relaxed` | `relaxed` | 통계 (현재 미사용) |

---

## 의존성

| 의존성 | 유형 | 용도 |
|--------|------|------|
| `ament_cmake` | 빌드 도구 | ROS 2 빌드 시스템 |
| `rtc_base` | 런타임 | `ThreadConfig`, `ApplyThreadConfig` (스레드 설정) |
| `ament_lint_auto` | 테스트 | 린트 자동 검사 |
| `ament_lint_common` | 테스트 | 공통 린트 규칙 |

**시스템 의존성:** POSIX 소켓 API (`sys/socket.h`, `arpa/inet.h`, `netinet/in.h`, `unistd.h`)

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
