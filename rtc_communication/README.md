# rtc_communication

RT-safe 네트워크 전송 추상화 (UDP · CAN · CAN FD · RS485) 와 패킷 코덱·트랜시버 템플릿. **유형**: 라이브러리 패키지 (header-only)

## 개요

외부 디바이스와 통신하는 전송 계층을 헤더 전용 C++20 라이브러리로 제공한다. `TransportInterface` 아래에 UDP·classic CAN·CAN FD·RS485 구현이 있고, `PacketCodec` concept 과 `Transceiver<Codec>` 가 수신 스레드·디코딩·송신을 묶는다. 설계 원칙: 생성 이후 동적 할당 없음, 모든 I/O 메서드 `noexcept`, RAII 소켓 수명 (RT 규칙 자체는 [invariants.md](../agent_docs/invariants.md) §RT Path Invariants).

소비자: [rtc_controller_manager](../rtc_controller_manager/README.md) · [udp_hand_driver](../udp_hand_driver/README.md). 스레드 설정 (`ThreadConfig`) 은 `rtc_base` 에서 가져온다.

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지만: `./build.sh --tests -p rtc_communication`.

소비자 `package.xml` 에 `<depend>rtc_communication</depend>` 를 추가하고 `CMakeLists.txt` 에서 의존한다.

```cmake
find_package(rtc_communication REQUIRED)
ament_target_dependencies(my_target rtc_communication)
```

UDP 로 `MyCodec` 패킷을 주고받는 최소 예 (코덱 정의는 [PacketCodec](#packetcodec-packet_codechpp)):

```cpp
UdpTransportConfig config{.bind_port = 9000,
                          .target_address = "192.168.1.100",
                          .target_port = 9001};
Transceiver<MyCodec> xcvr(std::make_unique<UdpTransport>(config));
xcvr.SetCallback([](const MyCodec::State& s) { /* mutex 해제 후 호출 */ });
if (!xcvr.StartRecv()) { /* Open 실패 */ }

MyCodec::SendPacket cmd{};
xcvr.Send(cmd);                      // RT-safe
auto state = xcvr.GetLatestState();  // non-RT 전용 (mutex)
```

## 인터페이스

모든 타입은 `rtc` 네임스페이스에 있다. 시그니처·`noexcept`·`[[nodiscard]]` 는 헤더 Doxygen 이 SSoT 다.

| 헤더 | 주요 타입 | 역할 |
|---|---|---|
| `transport_interface.hpp` | `TransportInterface` | 전송 추상 기반 (`Open` `Close` `Send` `Recv` `SetRecvTimeout` `SetRecvBufferSize` `is_open`). non-copyable, 생성자 `protected` |
| `udp/udp_socket.hpp` · `udp/udp_transport.hpp` | `UdpSocket` · `UdpTransport` | RAII UDP 소켓 / 송수신 소켓 2개를 관리하는 전송 |
| `can/can_socket.hpp` · `can/can_transport.hpp` | `CanSocket` · `CanTransport` | RAII SocketCAN raw 소켓 / classic CAN 전송 (payload ≤ 8 B) |
| `can/canfd_transport.hpp` | `CanFdTransport` | CAN FD 전송 (payload ≤ 64 B) + 프레임 API |
| `rs485/serial_port.hpp` · `rs485/rs485_transport.hpp` | `SerialPort` · `Rs485Transport` | RAII termios tty / length-prefixed 프레이머 전송 |
| `packet_codec.hpp` | `PacketCodec` · `DecodePacket` · `EncodePacket` | 코덱 concept 과 `memcpy` 헬퍼 |
| `transceiver.hpp` | `Transceiver<Codec>` | 수신 jthread + 코덱 + 송신 |
| `socket_options.hpp` | (내부 헬퍼) | UDP/CAN 공용 `SO_RCVBUF` · `SO_RCVTIMEO` · `SO_PRIORITY` 설정 |
| `testing/can_test_support.hpp` | `SKIP_IF_NO_VCAN` · `SKIP_IF_NO_FD_VCAN` | 테스트 전용 (gtest): `vcan0` 감지 · skip. 소비 패키지 테스트가 재사용 |

### UdpTransport

`bind_port > 0` 이면 수신 소켓을 바인드하고, `target_address` 가 비어 있지 않고 `target_port > 0` 이면 송신 소켓을 연결한다. `is_open()` 은 둘 중 하나라도 열려 있으면 `true`. `recv_socket()` / `send_socket()` 으로 하위 `UdpSocket` 에 접근할 수 있다 (레거시 이관용).

`UdpTransportConfig`:

| 필드 | 기본값 | 설명 |
|---|---|---|
| `bind_address` | `"0.0.0.0"` | 수신 바인드 주소. 빈 문자열도 `INADDR_ANY` |
| `bind_port` | `0` | 수신 포트. 0 이면 수신 소켓 미생성 |
| `target_address` | `""` | 송신 대상 IP. 비어 있으면 송신 소켓 미생성 |
| `target_port` | `0` | 송신 대상 포트 |
| `recv_buffer_size` | `256 * 1024` | `SO_RCVBUF` |
| `recv_timeout_ms` | `100` | `SO_RCVTIMEO` |

`UdpSocket` 을 직접 쓸 때는 수신은 `Bind(addr, port)` + `Recv(span)`, 송신은 `Connect(ip, port)` + `Send(span)` 이다.

### CanTransport / CanFdTransport

**payload-only 계약**이다. `Send(span)` 은 payload 를 설정된 `tx_can_id` 의 frame 하나로 보내고, `Recv(span)` 은 필터를 통과한 frame 의 payload 만 돌려준다. CAN ID·flag 는 config 에 고정되므로 인스턴스 하나가 tx ID 하나와 rx 필터 하나에 대응한다. 단일 fd 로 송수신하므로 `is_open()` 은 소켓 하나의 상태다. payload 한도를 넘기면 `Send()` 는 `-1` 을 돌려준다.

`CanTransportConfig`:

| 필드 | 기본값 | 설명 |
|---|---|---|
| `interface_name` | `"can0"` | CAN 인터페이스 이름 |
| `tx_can_id` | `0` | 송신 frame 의 CAN ID |
| `rx_can_id` | `0` | 수신 필터 ID |
| `rx_can_mask` | `0` | 필터 mask. **0 이면 필터 미설치 (전체 수신)** 이고 `rx_can_id` 는 효력이 없다. RTR frame 은 항상 drop |
| `extended_frame` | `false` | `true` 면 29-bit ID |
| `receive_own_messages` | `false` | 자기 송신 frame 수신 (`CAN_RAW_RECV_OWN_MSGS`) |
| `loopback` | `true` | 같은 호스트 소켓으로 local echo (`CAN_RAW_LOOPBACK`) |
| `recv_timeout_ms` | `100` | `SO_RCVTIMEO` |

`CanFdTransportConfig` 는 `CanTransportConfig` 를 상속하고 `bitrate_switch` (`CANFD_BRS`, 기본 `false`) 와 `error_state_indicator` (`CANFD_ESI`, 기본 `false`) 를 더한다. `Open()` 에서 `CAN_RAW_FD_FRAMES` 를 켜므로 classic·FD frame 을 모두 수신한다.

**프레임 API (`CanFdTransport` 전용).** 한 소켓으로 여러 CAN ID 를 다루는 프로토콜은 payload-only 계약 대신 `SendFrame(const canfd_frame&)` (classic 은 `can_frame` overload) · `RecvFrameWithTimeout(canfd_frame&, const timespec&)` (`ppoll`, sub-ms deadline) · `RecvFrameNonBlocking(canfd_frame&)` · `SetPriority(int)` (`SO_PRIORITY`) 를 쓴다. ID · len · `CANFD_BRS` 는 호출마다 호출자가 채우고 (`tx_can_id` · `bitrate_switch` 미적용), 수신은 같은 `rx_can_id` · `rx_can_mask` 필터를 지나 frame 그대로 온다. 반환값 · errno 규약은 헤더 주석이 SSoT 다.

```cpp
CanTransportConfig config{.interface_name = "can0",
                          .tx_can_id = 0x123,
                          .rx_can_id = 0x456,
                          .rx_can_mask = CAN_SFF_MASK};
auto transport = std::make_unique<CanTransport>(config);
transport->Open();
```

**PacketCodec 크기 제약.** `Transceiver::Send` 는 `sizeof(SendPacket)` 전체를 한 번에 `Send()` 로 넘기고, 수신 루프는 `n < sizeof(RecvPacket)` 인 frame 을 버린다. 따라서 CAN 위의 codec 은 packet 크기가 payload 한도 안이어야 한다 (`CanTransport` ≤ 8 B, `CanFdTransport` ≤ 64 B). 초과하면 `Transceiver::Send` 는 조용히 `false` 를 돌려준다. multi-frame 분할이나 frame 별 ID 해석이 필요한 프로토콜은 이 계약 밖이다.

### Rs485Transport

시리얼은 커널 프레이밍이 없는 raw 바이트 스트림이다. `SerialPort` 가 라인 소유 (open · 설정 · 방향 제어 · raw I/O) 를 맡고, `Rs485Transport` 가 `Recv()` 안에서 **벤더-중립 length-prefixed 프레이머**를 돌려 패킷 경계를 복원한다. sync prefix 와 length 필드로 파라미터화되어 header/length 형 와이어 포맷에 맞출 수 있다.

- payload-only 계약: `Recv()` 는 호출당 완전한 raw frame 하나 (sync..trailer) 를, `Send()` 는 호출자가 인코딩한 frame 을 그대로 쓴다. 프레임이 호출자 버퍼보다 크면 자르지 않고 프레임을 버리고 `-1`. **CRC 검증·필드 파싱은 codec 책임**이다.
- `recv_timeout_ms` 는 read 1회가 아니라 **`Recv()` 호출 전체**의 deadline 이다. `<= 0` 이면 프레임 완성까지 무한 대기하고, 양수는 100 ms 단위로 올림된다.
- 반이중 방향: `rs485_enabled` 면 `TIOCSRS485` 를 best-effort 로 쓰고, 성공하면 `Open()` 이 `drain_after_send` 를 자동으로 끈다. 실패하면 `tcdrain` fallback 이 켜지며, 이때 `Send()` 는 프레임이 UART 를 떠날 때까지 (≈ `frame_bytes × 10 / baud` 초) 블록하므로 **정기 RT tick 에서 호출하면 예산에 넣어야 한다.**
- self-echo: 송신 중에도 수신기가 켜진 2-wire 어댑터는 송신 프레임이 rx 로 돌아온다. `discard_echo=true` 면 `Send()` 가 drain 후 송신 바이트 수만큼 읽어 버린다. echo 가 확실한 하드웨어에서만 켠다 (비-echo 라인에서는 Send 당 수신 타임아웃 1회 비용과 응답 바이트 소비 위험).
- `frame_total = length_offset + length_size + length_value + length_adjust`. 미완성 프레임은 다음 `Recv()` 로 이월하고, corrupt length 는 1바이트씩 drop 하며 리싱크한다.

`Rs485TransportConfig`:

| 필드 | 기본값 | 설명 |
|---|---|---|
| `device` | `"/dev/ttyUSB0"` | tty 경로 |
| `baud_rate` | `57600` | 통신 속도 |
| `data_bits` / `parity` / `stop_bits` | 8 / `kNone` / 1 | 라인 프레이밍 |
| `rs485_enabled` | `true` | `TIOCSRS485` 커널 방향 제어 |
| `rts_on_send` / `rts_after_send` | `true` / `false` | DE 극성 |
| `delay_rts_before_us` / `delay_rts_after_us` | `0` | 방향 전환 지연 |
| `drain_after_send` | `true` | `TIOCSRS485` 실패 시 `tcdrain` fallback |
| `discard_echo` | `false` | 송신 후 self-echo 를 읽어 버림 (옵트인) |
| `sync_pattern` | `{FF,FF,FD,00}` | 프레임 시작 sync. 비우면 고정 길이 |
| `length_offset` | `5` | 프레임 시작 기준 length 필드 offset |
| `length_size` | `2` | length 필드 폭 (0/1/2). 0 이면 고정 길이 |
| `length_little_endian` | `true` | 2바이트 length 엔디안 |
| `length_adjust` | `0` | length 보정. 고정 길이 모드에서는 프레임 크기 |
| `max_frame_size` | `512` | 프레임 상한. 초과 시 corrupt 처리 (`kBufferCapacity` 1024 B 로 clamp) |
| `recv_timeout_ms` | `100` | `Recv()` 전체 deadline |

기본값은 Dynamixel Protocol 2.0 프레임 `FF FF FD 00 | ID | LEN(2,LE) | INST | PARAMS | CRC16(2)` 에 그대로 매핑된다 (`frame_total = 7 + LEN`). 고정 길이 프레이밍은 `sync_pattern` 을 비우고 **`length_offset=0`**, `length_size=0`, `length_adjust=프레임 크기` 로 지정한다 — `length_offset` 기본값 5 를 두면 프레임이 5바이트 길게 잘린다.

```cpp
Rs485TransportConfig config{.device = "/dev/ttyUSB0", .baud_rate = 1000000};
auto transport = std::make_unique<Rs485Transport>(config);
transport->Open();
```

RS485 버스는 반이중 master-initiated 트랜잭션이 일반적이다. `Transceiver` 의 상시 수신 스레드 모델은 스트리밍 디바이스에 맞고, request→status 동기 트랜잭션이나 벤더 CRC/byte-stuffing 이 필요한 드라이버는 codec + 트랜잭션 계층을 별도 패키지로 둔다.

### PacketCodec (`packet_codec.hpp`)

사용자 정의 코덱이 만족해야 하는 C++20 concept 이다.

| 요구 | 제약 |
|---|---|
| `Codec::RecvPacket` · `Codec::SendPacket` | `trivially_copyable` 와이어 포맷 |
| `Codec::State` | 디코딩된 애플리케이션 상태 |
| `Codec::Decode(span<const uint8_t>, State&)` | `static`, `bool` 반환. 필수 |

`Encode` 는 요구하지 않는다. 헬퍼: `DecodePacket<T>(span, T&)` (버퍼가 `sizeof(T)` 미만이면 `false`) · `EncodePacket<T>(const T&, span<uint8_t, sizeof(T)>)`.

```cpp
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

### Transceiver (`transceiver.hpp`)

`TransportInterface` 를 `unique_ptr` 로 소유하고, `std::jthread` 수신 루프가 `Recv` → 크기 검사 → `Codec::Decode` → 최신 상태 갱신 (mutex) → 콜백 순으로 돈다. 생성자는 `Transceiver(unique_ptr<TransportInterface>, const ThreadConfig& = kRtUdpRecvConfig)` 이고, 수신 스레드의 RT 설정 적용이 실패해도 계속 돈다 (rtprio 가 없는 사용자 대비). `Stop()` 은 소켓 타임아웃 (기본 100 ms) 만큼 지연될 수 있다.

| 메서드 | 설명 |
|---|---|
| `StartRecv()` | 전송 `Open()` + 수신 루프 시작 |
| `Stop()` | 수신 중단 + 전송 `Close()` |
| `Send(pkt)` | 스택 배열로 `memcpy` 후 전송 |
| `GetLatestState()` | 최신 상태 복사본 (mutex) |
| `SetCallback(fn)` | 새 데이터 수신 시 콜백 (`std::function<void(const State&)>`) |
| `IsRunning()` · `recv_count()` · `send_count()` | atomic 조회. `recv_count()` 는 디코딩 성공 시에만 증가 |
| `transport()` | 하위 전송 포인터 |

**호출 컨텍스트 (RT vs non-RT).** 정기 RT tick (`control_rate` — 범위는 invariants.md §RT Path Invariants) 에서 쓸 수 있는 것과 없는 것이 갈린다.

| 메서드 | RT-safe | 사유 |
|---|---|---|
| `Send(pkt)` | 예 | 스택 `memcpy` + `sendto()` 1회, `noexcept` |
| `IsRunning()` · `recv_count()` · `send_count()` | 예 | atomic load |
| `GetLatestState()` | **아니오** | `std::mutex` 획득 — 진단 스레드·ROS 콜백 등 non-RT 에서만 |
| `StartRecv()` · `Stop()` · `SetCallback()` | 아니오 | 초기화/셧다운 (jthread 생성·조인) |

RT 루프에 디코딩된 상태가 필요하면 `GetLatestState()` 를 직접 부르지 말고, non-RT 스레드에서 읽어 `SeqLock<State>` 또는 SPSC 큐로 전달한다.

## 디렉토리

```
rtc_communication/
├── include/rtc_communication/   헤더 (udp/ can/ rs485/ 하위)
└── test/                        loopback 테스트와 지원 헤더
```

파일별 설명은 각 헤더 상단 주석과 Doxygen 을 본다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| CAN `Transceiver::Send` 가 항상 `false` | `sizeof(SendPacket)` 가 payload 한도 (8 B / 64 B) 초과 | 코덱 packet 을 한도 안으로 줄인다 |
| CAN `Recv()` 에 아무것도 안 온다 | `rx_can_mask = 0` 이 아닌데 `rx_can_id` 불일치, 또는 RTR frame | mask/ID 확인. RTR 은 항상 drop |
| CAN/CAN FD 테스트가 SKIP | `vcan0` 없음 | 아래 vcan 설정 |
| RS485 `Send()` 가 오래 블록 | `TIOCSRS485` 미지원으로 `tcdrain` fallback | RT tick 밖에서 호출하거나 프레임 시간을 예산에 반영 |
| RS485 에서 송신 프레임이 수신으로 돌아온다 | 2-wire echo 어댑터 | `discard_echo=true` |

## 더 읽기

테스트: `colcon test --packages-select rtc_communication --event-handlers console_direct+` (GTest, 설계는 `test/` 파일 헤더).

CAN/CAN FD 테스트는 가상 인터페이스 `vcan0` 이 있어야 실행되고 없으면 `GTEST_SKIP()` 으로 건너뛴다.

```bash
sudo ../repo_scripts/scripts/setup_vcan.sh   # vcan0, MTU 72
```

수동 smoke test 는 [can-utils](https://github.com/linux-can/can-utils) (`sudo apt install can-utils`) 의 `candump vcan0` · `cansend vcan0 123#11223344` · `cangen vcan0` · `canfdtest vcan0` 를 쓴다. RS485 테스트는 PTY 페어를 시리얼 링크 대용으로 쓰므로 root·모듈 없이 항상 실행되며, `TIOCSRS485` 는 PTY 에서 미지원이라 프레이밍·타임아웃·바이트 투명성만 검증한다 (실기 검증은 USB-RS485 어댑터로 별도).

## 의존성

[package.xml](package.xml) — `rtc_base` (`ThreadConfig`, `ApplyThreadConfig`), 테스트 `ament_cmake_gtest`. C++20, 엄격 경고 플래그. 시스템 헤더 (POSIX 소켓, SocketCAN `linux/can.h`, termios `linux/serial.h`) 만 쓰므로 별도 설치가 필요 없다. MIT ([LICENSE](../LICENSE)).
