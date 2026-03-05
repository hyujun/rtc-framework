# 변경 이력 — ur5e_hand_udp

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [1.0.0] - 2026-03-04

### 추가

- **초기 분리**: `ur5e_rt_controller` 단일 패키지에서 UDP 손 브리지 기능을 독립 패키지로 추출
- `include/ur5e_hand_udp/hand_udp_receiver.hpp` — UDP 수신기
  - `std::jthread` (C++20 협동 취소) 기반 소켓 수신 루프
  - RT 스레드 설정: Core 3, SCHED_FIFO/65 (`kUdpRecvConfig`)
  - 패킷 형식: 77개 `double` (616바이트) — 위치/속도/전류/센서
  - 패킷 간격 통계 (5초마다 로깅)
- `include/ur5e_hand_udp/hand_udp_sender.hpp` — UDP 송신기
  - 11개 `double`을 리틀 엔디언으로 인코딩하여 전송
  - 정규화된 모터 명령 (0.0–1.0)
- `src/hand_udp_receiver_node.cpp` — `hand_udp_receiver_node` ROS2 노드
  - UDP 콜백 → `/hand/joint_states` 100Hz 재퍼블리시
- `src/hand_udp_sender_node.cpp` — `hand_udp_sender_node` ROS2 노드
  - `/hand/command` 구독 → UDP 전송
- `launch/hand_udp.launch.py` — 두 노드 동시 실행 (`udp_port`, `target_ip`, `target_port` 파라미터)
- `config/hand_udp_receiver.yaml` — UDP 포트, 버퍼 크기, 타임아웃, 퍼블리시 주기 설정
- `CMakeLists.txt` — `find_package(ur5e_rt_base REQUIRED)`, 두 실행 파일 빌드
- `package.xml` — `<depend>ur5e_rt_base</depend>` (ur5e_rt_controller 의존성 없음)

### 변경

- 인클루드 경로 변경 (네임스페이스는 `ur5e_rt_controller` 유지):
  - `#include "ur5e_rt_controller/hand_udp_receiver.hpp"` → `#include "ur5e_hand_udp/hand_udp_receiver.hpp"`
  - `#include "ur5e_rt_controller/hand_udp_sender.hpp"` → `#include "ur5e_hand_udp/hand_udp_sender.hpp"`
  - `#include "ur5e_rt_controller/rt_controller_interface.hpp"` → `#include "ur5e_rt_base/types.hpp"` (내부 헤더)
  - `#include "ur5e_rt_controller/thread_config.hpp"` → `#include "ur5e_rt_base/thread_config.hpp"` (내부 헤더)
  - `#include "ur5e_rt_controller/thread_utils.hpp"` → `#include "ur5e_rt_base/thread_utils.hpp"` (내부 헤더)
- 헤더 가드 변경: `UR5E_RT_CONTROLLER_*` → `UR5E_HAND_UDP_*`

### 참고

이 패키지는 다음 v4.4.0 `ur5e_rt_controller` 파일에서 추출되었습니다:
- `include/ur5e_rt_controller/hand_udp_receiver.hpp`
- `include/ur5e_rt_controller/hand_udp_sender.hpp`
- `src/hand_udp_receiver_node.cpp`
- `src/hand_udp_sender_node.cpp`
- `launch/hand_udp.launch.py`
- `config/hand_udp_receiver.yaml`

기능 변경 없이 패키지 분리만 수행되었습니다.
