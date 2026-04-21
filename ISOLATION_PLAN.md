# RTC Workspace Dependency Isolation Plan

> **상태**: ✅ **실행 완료 (2026-04-21)** — 모든 단계 + §8 A-D 정리 + rtc_mujoco_sim 재빌드 통과
> **작성일**: 2026-04-21 (Rev 3 — Option A 반영: PyTorch 제외, `pip freeze` lock 방식)
>
> **실행 결과 요약**:
> - colcon 20 packages build 통과 (4m50s)
> - rtc_tsid 107 tests pass (ros-jazzy-proxsuite 0.6.5 호환 확인)
> - 최종 격리 audit: ELF /usr/local 누수 0건, /opt/openrobots + /opt/rti.com + ~/libs 전부 제거, robotpkg 0개
> - MuJoCo: `/opt/mujoco-3.2.4/lib/libmujoco.so.3.2.4` 로 resolve
> - 후속 작업: (1) `rtc_tsid/test/*` 와 `rtc_mpc/test/*` 의 `/usr/local/share/example-robot-data/...` 하드코딩 리팩토링 (현재 심볼릭 링크로 우회); (2) CI 파이프라인 도입 시 §9.1 스니펫 참고
>
> **파일 위치 (최종 커밋 기준)** — 본 문서 § 의 명령어 안에 `<rtc_ws>/{deps.repos, requirements.*, scripts/, .colcon/}` 로 표기된 경로는 실행 직후 경로이며, git push 를 위해 **전부 `src/rtc-framework/` 하위로 이동**, 이후 shell 스크립트는 **`rtc_scripts/scripts/` 패키지로 통합**되었습니다:
> - `<rtc_ws>/ISOLATION_PLAN.md` → `src/rtc-framework/ISOLATION_PLAN.md`
> - `<rtc_ws>/deps.repos` → `src/rtc-framework/deps.repos`
> - `<rtc_ws>/requirements.{in,lock}` → `src/rtc-framework/requirements.{in,lock}`
> - `<rtc_ws>/scripts/{build_deps,setup_env,verify_isolation,uninstall_system_deps}.sh` → `src/rtc-framework/rtc_scripts/scripts/*.sh` (rtc_scripts 패키지로 통합 — `CMakeLists.txt` 의 `install(PROGRAMS ...)` 에 등록)
> - `<rtc_ws>/.colcon/defaults.yaml` → `src/rtc-framework/.colcon/defaults.yaml` (setup_env.sh 가 `COLCON_DEFAULTS_FILE` 로 주입)
> - **삭제됨**: `recovery_after_c1_abort.sh` (일회성 복구 스크립트)
> - **유지**: `<rtc_ws>/deps/{src,build,install}/` (빌드 아티팩트 + git clones — 커밋 안 함), `<rtc_ws>/.venv/`
>
> **install.sh · build.sh 통합** — `RTC_DEPS_PREFIX` 미설정 시 `rtc_scripts/scripts/setup_env.sh` 를 자동 source. 사용자는 `./build.sh` / `./install.sh` 를 바로 실행 가능 (대화형 `ros2 launch` / `colcon test` 만 수동 source 필요).
>
> **대상 워크스페이스**: `/home/junho/ros2_ws/rtc_ws` (개발용 PC)
> **대상 저장소**: `src/rtc-framework` (19 ROS 2 패키지)
> **실제 RT 제어 PC**: 별도 머신 — PREEMPT_RT 커널 설치는 본 작업 범위 밖

## 사용자 결정 반영 (2026-04-21)

| Q | 결정 | 계획 반영 |
|---|---|---|
| Q1 | robotpkg 14개 전체 삭제 | §8 단계 C — 조건부 없이 모두 purge |
| Q2 | `/opt/rti.com` 제거 | §8 단계 D |
| Q3 | `~/libs/` 삭제 | §8 단계 B — manifest uninstall 후 `rm -rf ~/libs/{fmt,mimalloc,aligator}` |
| Q4 | **Option A — 워크스페이스 미사용 확인, 제거** (정적 분석: `import torch` 0건) | Step 4 에서 `~/.local/lib/python3.12/site-packages/torch*` 등 제거, `requirements.in` 에 미포함 |
| Q5 | 본 PC 개발용 · 실 RT 는 별도 PC | PREEMPT_RT 커널 설치 제외 · 실 RT PC 로의 배포 재현성은 `deps.repos`+`requirements.lock` 로 확보 |
| Q6 | CI 파이프라인 재사용 | §9.1 CI 가이드라인 섹션 추가 |
| Q7 | `ros-jazzy-proxsuite` 사전 확인 완료 | **결과: 존재** (0.6.5-1noble). robotpkg-proxsuite (0.7.2) 와 버전 차이 있음 — §8 단계 C 에서 ROS 판으로 전환 후 빌드·테스트 통과 확인 |
| 추가 | **`uv` 미사용, `venv`+`pip` 사용** | Step 4 를 `python3 -m venv` + `pip` + `pip-tools` (선택) 로 재작성 |

### Q7 사전 확인 로그 (2026-04-21)

```
$ apt-cache policy ros-jazzy-proxsuite
  Installed: (none)
  Candidate: 0.6.5-1noble.20260225.060049
```
- ROS 저장소에 **존재** (버전 0.6.5, robotpkg 의 0.7.2 보다 낮음).
- **0.6.5 가 `rtc_tsid` 의 WQP/HQP API 와 호환되는지**는 단계 C 실행 전 Step 6 의 `colcon build --packages-select rtc_tsid` 로 사전 검증 필요.
- 비호환 시 fallback: `robotpkg-proxsuite 0.7.2` 만 보존 (다른 robotpkg 는 제거).

### 추가 사전 확인 결과

- `/opt/onnxruntime` 은 `/opt/onnxruntime-linux-x64-1.17.1` 로의 **심볼릭 링크** — 복사본 아님. 그대로 유지.
- `libspdlog-dev` 는 `ros-jazzy-spdlog-vendor`, `ros-jazzy-rcl-logging-spdlog` 가 역의존 → **제거 불가**. §8 단계 D 에서 제외.

---

## 0. TL;DR

- **격리 레벨**: **Level 3 확정** — colcon overlay + 표준 `python3 -m venv` + ABI 민감 C++ 의존성을 `deps/install/` 로 소스 빌드. **`uv` 미사용** (사용자 요구).
- **근거 (3줄)**:
  1. 이미 *현장 발견된* 세 가지 dual-install ABI 충돌(fmt 9 vs 11, hpp-fcl ColMajor vs RowMajor, pinocchio 3.0 stale) 이 시스템 전역에 존재하며, `rtc_mpc/CMakeLists.txt`·`install.sh` 가 `HINTS/CACHE PATH` 해킹으로 이를 우회 중.
  2. RT hot path (500 Hz SCHED_FIFO 90, `mlockall`) 가 Eigen/Pinocchio/Aligator/fmt/mimalloc 에 직결되므로, 시스템 업데이트로 소리 없이 교체되면 결정적 동작이 깨질 수 있음.
  3. Level 4(전체 소스 빌드) 는 ROS Jazzy 가 공식 지원하는 `pinocchio 3.9`·`hpp-fcl 2.4.5`·(신규) `proxsuite 0.6.5` 패키지 품질을 버리는 비용이 크고, Level 2(venv만) 는 근본 C++ 충돌을 해결하지 못함.
- **부가 작업** (사용자 승인 완료):
  - §8 A: `/usr/local` 의 2024-07 stale 잔재 (hpp-fcl · pinocchio 3.0 · gtsam · teaser) 삭제.
  - §8 B: `/usr/local` 의 fmt/mimalloc/aligator uninstall + `~/libs/` 삭제 (Q3).
  - §8 C: robotpkg 14 개 전량 제거 + `ros-jazzy-proxsuite 0.6.5` 로 전환 + `/opt/openrobots` 제거 (Q1, Q7).
  - §8 D: `/opt/rti.com` 제거 (Q2).
- **범위 밖**: PREEMPT_RT 커널 (본 PC 는 개발용, 실 RT PC 는 별도 — §9.2).

---

## 1. Phase 1 — 워크스페이스 분석 결과

### 1.1 환경 정보

| 항목 | 값 | 메모 |
|---|---|---|
| OS | Ubuntu 24.04.4 LTS (Noble) | ✓ |
| Kernel | `6.17.0-22-generic` | **⚠ PREEMPT_RT 아님** (`CONFIG_PREEMPT_RT is not set`). `-rt` 접미사 없음. 하드 RT 보장 없음 — Level 3 격리와 **별개로** RT 커널 필요 시 별도 설치 필요. |
| ROS 2 | Jazzy | `/opt/ros/jazzy` |
| Python | 3.12.3 (`/usr/bin/python3`) | |
| `ulimit -r / -l` | `99 / unlimited` | ✓ 사용자 이미 `@realtime` 그룹 |
| realtime limits | `/etc/security/limits.conf` 에 `@realtime - rtprio 99 / memlock unlimited` | ✓ |
| Env (문제 있음) | `CMAKE_PREFIX_PATH=/opt/openrobots:` · `LD_LIBRARY_PATH=/opt/openrobots/lib:` · `PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:` | **← 격리의 최대 적**. `~/.bashrc` L129-133 에서 export. robotpkg 설치 잔재. |

### 1.2 워크스페이스 구조

```
/home/junho/ros2_ws/rtc_ws/
├── build/        (colcon)
├── install/      (colcon overlay — 존재)
├── log/          (colcon)
├── logging_data/ (runtime logs)
├── src/
│   └── rtc-framework/        ← 단일 저장소, 19 ROS 2 패키지 + 1 meta
└── .venv/        ← 현재: `--system-site-packages`, Cython + pip 만 설치 (사실상 미사용)
```

**패키지 인벤토리 (20개)**:
| 유형 | 개수 | 패키지 |
|---|---|---|
| `ament_cmake` | 18 | rtc_base, rtc_communication, rtc_controller_interface, rtc_controller_manager, rtc_controllers, rtc_inference, rtc_mpc, rtc_msgs, rtc_mujoco_sim, rtc_scripts, rtc_tsid, rtc_urdf_bridge, ur5e_bringup, ur5e_bt_coordinator, ur5e_description, ur5e_hand_driver, shape_estimation, shape_estimation_msgs |
| `ament_python` | 2 | rtc_digital_twin, rtc_tools |

외부 submodule/vendored 라이브러리: **없음** (모든 외부 의존성은 시스템 또는 `~/libs/`).

### 1.3 C++ 의존성 추출

**`find_package(...)` 호출 전수** (주요 것만):

| 의존성 | 버전 요구 | 시스템 실설치 | 출처 | ABI 민감 |
|---|---|---|---|---|
| `Eigen3` | (암묵적 ≥ 3.4) | `libeigen3-dev 3.4.0-4build0.1` | apt | **★** (header-only지만 ABI 전파) |
| `pinocchio` | (버전 제약 없음) | `ros-jazzy-pinocchio 3.9.0-1noble` · **잔재 3.0.0 @ /usr/local** | apt + stale source | **★★★** |
| `proxsuite` | (버전 제약 없음) | `robotpkg-proxsuite 0.7.2 @ /opt/openrobots` · **ROS 미설치** | robotpkg | **★★** |
| `aligator` | (버전 제약 없음) | `0.19.0 @ /usr/local` | source build via `install.sh` | **★★★** |
| `fmt` | `≥ 10` (HINT: `/usr/local/lib/cmake/fmt`) | `libfmt-dev 9.1.0` (apt) + `11.1.4 @ /usr/local` | **이중** | **★★★** (아래 §1.7 충돌 참조) |
| `hpp-fcl` | (pinocchio 의 transitive) | `ros-jazzy-hpp-fcl 2.4.5` + **stale 2024 RowMajor @ /usr/local** | **이중 ABI 비호환** | **★★★** |
| `yaml-cpp` | — | `libyaml-cpp-dev` (apt) | apt | 낮음 |
| `tinyxml2` | QUIET | `libtinyxml2-dev` (apt) | apt | 낮음 |
| `onnxruntime` | QUIET | `/opt/onnxruntime/` + `/opt/onnxruntime-linux-x64-1.17.1/` | **바이너리 드롭** | **★** |
| `mujoco` | QUIET | `/opt/mujoco-3.2.4/` + `/usr/local/lib/libmujoco.so` (중복) | **이중** | 낮음(RT 경로 외) |
| `behaviortree_cpp` | — | `ros-jazzy-behaviortree-cpp` | apt | 낮음 |
| `rclcpp` / `rclcpp_lifecycle` / `sensor_msgs` etc. | — | `ros-jazzy-*` | apt (ROS) | ROS 관리 |
| `rosidl_default_generators` | — | ROS | apt | ROS 관리 |

**참고**: `robotpkg-py312-eigenpy 3.12.0` 와 `ros-jazzy-eigenpy 3.12.0` 가 **동시** 설치되어 있어 PYTHONPATH 선순위에 따라 다른 eigenpy 가 로드됨.

### 1.4 Python 의존성 추출

`setup.py` / `requirements.txt` 선언 + 정적 `import` 분석 결과 (ROS·stdlib 제외):

| 패키지 | 선언 | 실 사용 위치 | 설치 상태 |
|---|---|---|---|
| `numpy` | `requirements.txt ≥ 1.24.3`, `setup.py install_requires=[setuptools]` (미선언) | rtc_tools, rtc_digital_twin | `python3-numpy 1.26.4` (apt) |
| `scipy` | `≥ 1.10.1` | rtc_tools | `python3-scipy 1.17.1` (apt) |
| `matplotlib` | `≥ 3.5.3` | rtc_tools/plotting | `python3-matplotlib 3.6.3` (apt) |
| `pandas` | `≥ 1.5.3` | rtc_tools | `python3-pandas 2.1.4+dfsg` (apt) |
| `mujoco` | `≥ 3.0.0` | rtc_tools, launch | `mujoco 3.2.0` (`~/.local/lib/python3.12`) |
| `PyQt5` | 미선언 (package.xml 에 `python3-pyqt5`) | rtc_tools/gui | `PyQt5 5.15.10` (apt) |
| `Cython` | (install.sh 에서 venv 에 pip) | eigenpy configure-time | venv 와 system local 양쪽 |
| `tkinter` | 미선언 | rtc_tools/gui (일부) | `python3-tk` (apt, stdlib 확장) |

**데이터 기반 제어 관련**: `torch 2.11.0` 이 `~/.local/lib/python3.12` 에 설치되어 있으나 **현재 워크스페이스 코드는 사용하지 않음** (정적 분석 결과). 추후 RL/IL 통합 시 검증 필요.

### 1.5 실시간 영향 요소

**RT hot path 핵심 경로**:
- `rtc_controller_manager/src/rt_controller_main_impl.cpp:45` — `mlockall(MCL_CURRENT | MCL_FUTURE)`
- `rtc_controller_manager/src/rt_controller_node_rt_loop.cpp` — 500 Hz `clock_nanosleep` 루프, SCHED_FIFO 90 (Core 2)
- `rtc_base/include/rtc_base/threading/thread_config.hpp` — SCHED_FIFO 스케줄링 (sensor_executor 70, udp_recv 65, MPC 60)

**RT hot path 가 링크하는 라이브러리 (최우선 격리/검증 대상)**:
`libeigen3` (headers), `libpinocchio_default`, `libproxsuite`, `libaligator`, `libfmt.so.11`, `libmimalloc.so.2`, `libhpp-fcl`, `libyaml-cpp`, `libtinyxml2`, `libonnxruntime`, `librtc_*` (자기 패키지).

CycloneDDS 설정 파일: `rtc_controller_manager/config/cyclone_dds.xml` (기존 경로 유지 필요).

### 1.6 현재 시스템 설치물 — *stale/conflicting* 판정

| 경로 / 패키지 | 설치일 | 현재 용도 | 판정 |
|---|---|---|---|
| `/usr/local/lib/libaligator.so.0.19.0` | 2026-04-19 | rtc_mpc (활성) | **이동** → `deps/install/` |
| `/usr/local/lib/libfmt.so.11.1.4` | 2026-04-19 | aligator, rtc_mpc | **이동** → `deps/install/` |
| `/usr/local/lib/libmimalloc.so.2.1` | 2026-04-19 | aligator | **이동** → `deps/install/` |
| `/usr/local/lib/libhpp-fcl.so` (193 MB) | **2024-07-05** | 과거 `~/git/simple-mpc` 용 (RowMajor) | **삭제 대상** (ABI 비호환 — ROS 판이 win 함) |
| `/usr/local/lib/libpinocchio_*.so.3.0.0` (5 개 SO) | **2024-07-05** | 과거 simple-mpc 잔재 | **삭제 대상** (Jazzy 3.9.0 이 win 하지만 혼란 유발) |
| `/usr/local/lib/libpinocchio_casadi.so.3.0.0` | 2024-07-05 | 사용 안 함 | **삭제 대상** (131 MB) |
| `/usr/local/lib/libgtsam*.so`, `libteaser*.so`, `libmetis-gtsam.so`, `libCppUnitLite.a` | 2024 | **워크스페이스 미사용** | **삭제 대상** |
| `~/libs/{fmt,mimalloc,aligator}` | 2026-04-19 | 위 /usr/local 의 소스 + `install_manifest.txt` | **보존** (uninstall 용) 후 `deps/src/` 로 병합 검토 |
| `robotpkg-proxsuite 0.7.2` @ `/opt/openrobots` | 과거 | `rtc_tsid` 가 사용 | **판단 필요** — ROS Jazzy 에 `proxsuite` apt 패키지 제공되면 전환 권장. 현재 `apt-cache show ros-jazzy-proxsuite` 확인 필요 |
| `robotpkg-py312-eigenpy 3.12.0` + `ros-jazzy-eigenpy 3.12.0` | 중복 | Pinocchio Python 바인딩 | **robotpkg 판 제거**; ROS 판 보존 |
| `robotpkg-py312-casadi`, `robotpkg-casadi`, `robotpkg-blasfeo`, `robotpkg-fatrop`, `robotpkg-qpoases+doc`, `robotpkg-simde`, `robotpkg-visit-struct`, `robotpkg-collada-dom`, `robotpkg-openscenegraph`, `robotpkg-py312-pythonqt`, `robotpkg-py312-qt5-gepetto-viewer`, `robotpkg-qt5-osgqt`, `robotpkg-qt5-qgv`, `robotpkg-jrl-cmakemodules` | 과거 | **워크스페이스 전체에서 `find_package(casadi\|blasfeo\|fatrop\|qpoases\|openscenegraph\|gepetto)` 사용 없음** (grep 확인 완료) | **삭제 대상** (14 개 패키지) |
| `~/.bashrc` L129-133 `/opt/openrobots` env exports | install.sh 가 주입 | robotpkg 경로 등록 | **주석/삭제** — `deps/install/` 로 대체 |
| `/opt/mujoco-3.2.4`, `/opt/onnxruntime*`, `/opt/rti.com` | 바이너리 드롭 | 각각 sim, hand_driver, (미사용 DDS) | **보존** (바이너리 드롭, ABI 이슈 없음). `/opt/rti.com` 는 RTI Connext 라이선스; 현 워크스페이스 미사용. 제거 권장. |
| `libfmt-dev:amd64 9.1.0` | apt (ROS 간접 의존) | ROS 패키지들 사용 | **보존** (ROS 가 의존) |
| `libspdlog-dev:amd64 1.12.0` | apt | **워크스페이스 미사용** (grep 확인) | **삭제 가능** (BT.CPP 가 간접 사용 가능성 확인 필요) |

### 1.7 식별된 세 가지 ABI 충돌 (이미 수동 우회중)

1. **fmt 9 (apt) vs fmt 11.1.4 (source)**: `rtc_mpc/CMakeLists.txt` L35 에 `find_package(fmt 10 REQUIRED HINTS /usr/local/lib/cmake/fmt)` 하드코딩.
2. **hpp-fcl ColMajor (ROS 2.4.5) vs RowMajor (/usr/local 2024)**: pinocchio transitive `find_dependency(hpp-fcl)` 가 /usr/local 먼저 선택 → `rtc_mpc` 는 `set(hpp-fcl_DIR /opt/ros/jazzy/... CACHE PATH "" FORCE)` 주입 (memory 확인).
3. **pinocchio 3.0 stale (/usr/local) vs 3.9.0 (ROS)**: 현재 ROS 가 resolver 에서 이김 (ROS 소싱 시 LD_LIBRARY_PATH prepend). 시스템 전역 영향 없지만 혼란 요인.

→ 이 세 가지는 **시스템 설치물 제거(§8) + `deps/install/` 재빌드** 로 한 번에 해소 가능.

---

## 2. Phase 2 — 격리 전략

### 2.1 레벨 선택: **Level 3**

| 레벨 | 설명 | 채택? |
|---|---|---|
| L1: Workspace overlay 만 | 현재 상태. 시스템 ABI 충돌 영구 잔존. | ❌ |
| L2: + Python venv (uv) | C++ 문제 미해결. | ❌ |
| **L3: + 핵심 C++ deps 소스 빌드 (`deps/install`)** | ABI 민감 deps 전부 프로젝트 소유. ROS 의 pinocchio/proxsuite/hpp-fcl 는 **distro 판 유지**. | **✓** |
| L4: 전체 소스 빌드 | Pinocchio·ProxSuite·hpp-fcl 까지 소스 빌드. Jazzy 의 공식 3.9.0 빌드 품질 버림. | ❌ (보류) |

**근거 보강**:
- **버전 충돌 발견**: 3건 (§1.7) — Level 2 이하로는 해결 불가.
- **재현성 요구**: 현재 배포 대상은 1대. CI 없음. 그러나 향후 로봇 추가·PREEMPT_RT 머신 셋업 재현성 확보 필요.
- **빌드 시간**: Aligator `~3-5 분`, fmt `<1 분`, mimalloc `<1 분`. Pinocchio/ProxSuite 까지 포함하면 `~20-30 분` 추가 — ROI 낮음.

### 2.2 제안 디렉토리 레이아웃

```
/home/junho/ros2_ws/rtc_ws/
├── .venv/                       ← uv 로 재생성 (현재 것 백업 후 제거)
├── .colcon/
│   └── defaults.yaml            ← colcon build/test 고정 옵션 (신규)
├── build/  install/  log/       ← colcon 산출물 (기존, 재빌드)
├── deps/                        ← 신규: 격리된 비ROS C++ 의존성
│   ├── src/                     ← vcs import 결과 (git clone)
│   │   ├── fmt/                 (11.1.4 tag)
│   │   ├── mimalloc/            (v2.1.7)
│   │   └── aligator/            (v0.19.0)
│   ├── build/                   ← 임시 빌드 트리
│   └── install/                 ← deps prefix — rtc_* 가 find_package 로 읽음
│       ├── bin/  include/  lib/
│       └── lib/cmake/{fmt,mimalloc,aligator}
├── scripts/                     ← 신규
│   ├── build_deps.sh            (의존성 위상 빌드)
│   ├── setup_env.sh             (sourcing 래퍼)
│   └── uninstall_system_deps.sh (§8 의 안전한 제거 스크립트)
├── deps.repos                   ← vcstool 매니페스트 (신규)
├── requirements.lock            ← uv pip compile 결과 (신규)
├── uv.lock                      ← uv native lock (신규, 택1)
├── ISOLATION_PLAN.md            ← 이 문서
└── src/rtc-framework/           ← 기존, 일부 CMakeLists 수정 (§4.6)
```

**기존 대비 변경**:
- **추가**: `deps/`, `scripts/`, `.colcon/`, `deps.repos`, `requirements.lock`.
- **재생성**: `.venv/`, `build/`, `install/`.
- **삭제 예정 (시스템)**: §8 의 14 개 robotpkg + 5 개 stale /usr/local 라이브러리.

### 2.3 단계별 실행 계획

각 단계: **명령 · 예상 시간 · 롤백 · 검증**.

---

#### Step 0 — 백업 (3 분, safe)

```bash
cd /home/junho/ros2_ws/rtc_ws
TS=$(date +%Y%m%d_%H%M%S)
tar -czf ../rtc_ws_backup_${TS}.tar.gz --exclude='build' --exclude='log' install/ .venv/ src/rtc-framework/CMakeLists.txt.in 2>/dev/null || true
dpkg -l | awk '/^ii/ {print $2" "$3}' > ../rtc_ws_apt_snapshot_${TS}.txt
/home/junho/ros2_ws/rtc_ws/.venv/bin/pip freeze > ../rtc_ws_venv_snapshot_${TS}.txt 2>/dev/null || true
ls -laR /usr/local/lib/ > ../rtc_ws_usr_local_snapshot_${TS}.txt
cp ~/.bashrc ../rtc_ws_bashrc_backup_${TS}
```
- **롤백**: `tar -xzf` + `~/.bashrc` 복사 + robotpkg 재설치 가능.
- **검증**: `ls -la ../rtc_ws_*${TS}*` 로 4 개 파일 존재 확인.

---

#### Step 1 — realtime limits 재확인 (1 분)

```bash
ulimit -r && ulimit -l
grep -E 'rtprio|memlock' /etc/security/limits.conf /etc/security/limits.d/*.conf 2>/dev/null
groups | grep -q realtime && echo OK || echo "MISSING"
```
- 출력 기대: `99 / unlimited / rtprio 99 + memlock unlimited / OK`.
- **조치 불필요** (이미 설정됨). 롤백 없음.

---

#### Step 2 — `/opt/openrobots` 환경 변수 오염 제거 (5 분, reversible)

```bash
# ~/.bashrc L125-133 주석 처리 (env 오염 제거)
sed -i.bak '/^export RTI_LICENSE_FILE=.*\/opt\/rti/d;/^alias ros2rti=.*\/opt\/rti/d' ~/.bashrc
sed -i '/^export .*\/opt\/openrobots/s/^/# ISOLATED /' ~/.bashrc
# 현 쉘에서 즉시 제거
unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
exec bash -l   # 재로그인 없이 새 쉘
```
- **롤백**: `cp ~/.bashrc.bak ~/.bashrc && exec bash -l`.
- **검증**: `env | grep -E 'openrobots|rti.com'` → 빈 출력.

---

#### Step 3 — 시스템 의존성 정리 (15 분, §8 상세)

§8 의 `scripts/uninstall_system_deps.sh` 실행. **각 단계마다 사용자 확인 프롬프트** (대화형) 제공.

---

#### Step 4 — Python venv 재구성 (`python3 -m venv` + `pip`, 10-20 분)

> 사용자 지침: **`uv` 미사용, 표준 `venv` + `pip`**. 재현성 확보를 위해 `pip-tools` 의 `pip-compile` 로 lock 파일 생성 (선택) 또는 `pip freeze` 로 간이 lock.

```bash
cd /home/junho/ros2_ws/rtc_ws

# ① ~/.local 의 무관한 대형 패키지 먼저 정리 (venv 우선순위 충돌 방지 · Option A)
#    — venv 활성화 **전에** 시스템 python 으로 실행해야 ~/.local 의 user-scope 설치를 건드림
python3 -m pip uninstall --yes \
  torch torchvision cuda-bindings cuda-pathfinder cuda-toolkit \
  functorch triton \
  2>/dev/null || true
# mujoco 는 requirements.in 에서 다시 설치하므로 먼저 제거
python3 -m pip uninstall --yes mujoco mujoco-logger glfw 2>/dev/null || true

# ② venv 재생성 (표준 stdlib, uv 미사용)
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python -m pip install --upgrade pip setuptools wheel

# ③ 요구 패키지 선언 (Option A — PyTorch 제외)
cat > requirements.in <<'EOF'
# rtc_tools/requirements.txt 의 상위 집합
# (PyTorch 는 현재 워크스페이스 미사용 — 정적 분석 확인. 사용 시점에 재추가.)
numpy>=1.24.3
scipy>=1.10.1
matplotlib>=3.5.3
pandas>=1.5.3
mujoco>=3.0.0
Cython
# PyQt5 · rclpy · ament_* 는 system-site-packages 로 끌어옴 — venv 재설치 불요.
EOF

# ④ Lock 생성 (pip freeze 방식 — 외부 도구 미사용)
pip install -r requirements.in
pip freeze --exclude-editable > requirements.lock

# (향후 적용 시: pip install --require-hashes 를 쓰려면 pip-compile 로 재생성 필요.
#  현재는 간이 pin 으로 충분.)
```

**주의 — `--system-site-packages` 누수 방지**:
venv 내부에 `numpy` 등이 재설치되므로 import 우선순위에서 venv 가 이기지만, **`~/.local` 에 같은 이름의 패키지가 남아 있으면** sys.path 순서에 따라 잡힐 수 있음. ①에서 이미 제거했으나 추가 확인:
```bash
python -c "import sys; [print(p) for p in sys.path]"
python -c "import numpy, mujoco; print(numpy.__file__); print(mujoco.__file__)"
# numpy/mujoco 는 .venv/lib/python3.12/site-packages/... 경로여야 함
```

- **롤백**: `rm -rf .venv && tar -xzf ../rtc_ws_backup_*.tar.gz .venv/` + `pip install --user <list>` 로 ~/.local 복원 (백업 대상이 아니므로 완전 복원 불가 — 제거 전 `~/.local` 목록 스냅샷을 Step 0 에 추가).
- **검증**:
  ```bash
  python -c "import numpy, scipy, matplotlib, pandas, mujoco, PyQt5, rclpy; \
             print([m.__file__ for m in (numpy, mujoco, PyQt5, rclpy)])"
  # numpy/mujoco 는 .venv/..., PyQt5 는 /usr/lib/python3/dist-packages, rclpy 는 /opt/ros/jazzy/...
  ```

---

#### Step 5 — `deps/` 디렉토리 및 `deps.repos` 작성 (10 분)

```bash
mkdir -p /home/junho/ros2_ws/rtc_ws/deps/{src,build,install}
cat > /home/junho/ros2_ws/rtc_ws/deps.repos <<'EOF'
repositories:
  fmt:
    type: git
    url: https://github.com/fmtlib/fmt.git
    version: 11.1.4
  mimalloc:
    type: git
    url: https://github.com/microsoft/mimalloc.git
    version: v2.1.7
  aligator:
    type: git
    url: https://github.com/Simple-Robotics/aligator.git
    version: v0.19.0
EOF

cd /home/junho/ros2_ws/rtc_ws/deps/src
vcs import . < ../../deps.repos
# aligator submodules (eigenpy, example-robot-data)
(cd aligator && git submodule update --init --recursive --depth 1)
```
- **롤백**: `rm -rf deps/`.
- **검증**: `ls deps/src/{fmt,mimalloc,aligator}/.git` → 존재.

---

#### Step 6 — 의존성 위상 빌드 (`scripts/build_deps.sh`, 15-20 분)

위상 순서: `fmt → mimalloc → aligator` (aligator 가 fmt + pinocchio + hpp-fcl + mimalloc 요구).

```bash
cat > /home/junho/ros2_ws/rtc_ws/scripts/build_deps.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
WS=/home/junho/ros2_ws/rtc_ws
DEPS_PREFIX="${WS}/deps/install"
# ROS 소싱 — hpp-fcl_DIR resolve 용 (Step 8 후에는 /opt/ros/jazzy 만)
source /opt/ros/jazzy/setup.bash
HPPFCL_DIR="/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/hpp-fcl"

build_one() {
  local name="$1"; shift
  local src="${WS}/deps/src/${name}"
  local bld="${WS}/deps/build/${name}"
  cmake -S "$src" -B "$bld" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$DEPS_PREFIX" \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib:$DEPS_PREFIX/lib" \
        -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
        -DBUILD_SHARED_LIBS=ON \
        "$@"
  cmake --build "$bld" --parallel
  cmake --install "$bld"
}

build_one fmt -DFMT_TEST=OFF
build_one mimalloc -DMI_BUILD_TESTS=OFF
build_one aligator \
    -DBUILD_TESTING=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_WITH_PINOCCHIO_SUPPORT=ON \
    -Dhpp-fcl_DIR="$HPPFCL_DIR" \
    -Dfmt_DIR="${DEPS_PREFIX}/lib/cmake/fmt"
echo "deps built at $DEPS_PREFIX"
EOF
chmod +x /home/junho/ros2_ws/rtc_ws/scripts/build_deps.sh
/home/junho/ros2_ws/rtc_ws/scripts/build_deps.sh
```
- **롤백**: `rm -rf deps/build deps/install`.
- **검증**:
  ```bash
  ls deps/install/lib/libaligator.so.0.19.0
  readelf -d deps/install/lib/libaligator.so.0.19.0 | grep -E 'RPATH|RUNPATH'
  # 기대: RUNPATH = $ORIGIN/../lib:...
  ```

---

#### Step 7 — `rtc_mpc` 의 하드코딩 HINTS 교체 (5 분)

`src/rtc-framework/rtc_mpc/CMakeLists.txt` L35 현재:
```cmake
find_package(fmt 10 REQUIRED HINTS /usr/local/lib/cmake/fmt)
```
→ 교체 (`${RTC_DEPS_PREFIX}` 는 아래 setup_env.sh 의 `CMAKE_PREFIX_PATH` 를 통해 주입):
```cmake
find_package(fmt 10 REQUIRED)  # deps/install via CMAKE_PREFIX_PATH
# hpp-fcl workaround 는 ROS 판 존속이므로 유지
set(hpp-fcl_DIR /opt/ros/$ENV{ROS_DISTRO}/lib/x86_64-linux-gnu/cmake/hpp-fcl CACHE PATH "" FORCE)
```

`install.sh` 의 `install_fmt_from_source` / `install_mimalloc_from_source` / `install_aligator_from_source` 는 `scripts/build_deps.sh` 로 완전 대체 → install.sh 에서 해당 함수 호출 제거.

- **롤백**: `git checkout src/rtc-framework/rtc_mpc/CMakeLists.txt install.sh`.
- **검증**: `git diff` 리뷰 후 Step 8 의 colcon 재빌드가 성공하는지 확인.

---

#### Step 8 — colcon 재빌드 (`CMAKE_PREFIX_PATH` 주입, 20 분)

```bash
cat > /home/junho/ros2_ws/rtc_ws/scripts/setup_env.sh <<'EOF'
#!/usr/bin/env bash
# Source order: system limits → ROS → deps → venv → workspace
source /opt/ros/jazzy/setup.bash

export RTC_DEPS_PREFIX="/home/junho/ros2_ws/rtc_ws/deps/install"
export CMAKE_PREFIX_PATH="${RTC_DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${RTC_DEPS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${RTC_DEPS_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# venv (system-site-packages 로 ROS Python 유지)
source /home/junho/ros2_ws/rtc_ws/.venv/bin/activate

# workspace overlay (재빌드 후에만 유효)
if [[ -f /home/junho/ros2_ws/rtc_ws/install/setup.bash ]]; then
  source /home/junho/ros2_ws/rtc_ws/install/setup.bash
fi
EOF
chmod +x /home/junho/ros2_ws/rtc_ws/scripts/setup_env.sh

cat > /home/junho/ros2_ws/rtc_ws/.colcon/defaults.yaml <<'EOF'
build:
  symlink-install: true
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    - -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
test:
  event-handlers:
    - console_cohesion+
EOF

rm -rf build/ install/ log/
source scripts/setup_env.sh
colcon build
```
- **롤백**: `git checkout` + 이전 `install/` 복원.
- **검증**:
  ```bash
  # 하나의 대표 바이너리가 deps/install 에서 aligator 를 끌어오는지
  readelf -d install/rtc_mpc/lib/librtc_mpc*.so | grep -E 'NEEDED|RUNPATH'
  ldd install/rtc_mpc/lib/librtc_mpc*.so | grep -E 'aligator|fmt'
  # 기대: libaligator.so.0.19.0 => /home/junho/ros2_ws/rtc_ws/deps/install/lib/...
  ```

---

#### Step 9 — RPATH · 격리 검증 (5 분)

```bash
for so in install/*/lib/lib*.so; do
  echo "=== $so ==="
  readelf -d "$so" | grep -E 'RPATH|RUNPATH' || echo "(none)"
  ldd "$so" 2>/dev/null | grep -vE '/opt/ros/jazzy|/home/junho/ros2_ws/rtc_ws/(deps/install|install)|linux-vdso|/lib(64)?/x86_64-linux-gnu' | sort -u
done
```
기대: 마지막 ldd 필터에서 결과 없음 — 모든 의존이 *ROS / deps / 워크스페이스 / libc* 로만 resolve.

---

#### Step 10 — RT 성능 회귀 테스트 (30-60 분)

```bash
# cyclictest (rt-tests 패키지 필요: sudo apt install rt-tests)
sudo cyclictest -p 90 -t 1 -n -i 1000 -D 60s -l 60000 > ../cyclictest_after_isolation.txt

# 워크스페이스 RT 루프 jitter
source scripts/setup_env.sh
ros2 launch ur5e_bringup sim.launch.py &
LAUNCH_PID=$!
sleep 30
# rtc_controller_manager 의 jitter_us 메트릭은 로그로 출력됨
grep 'jitter_us' /home/junho/ros2_ws/rtc_ws/logging_data/*.log | tail -100
kill $LAUNCH_PID
```
기준: jitter 99.9 %ile 가 격리 **이전 대비 악화 없음**. 악화 시 RPATH 누락 의심.

---

### 2.4 RT 성능 보존 체크리스트

| 검증 항목 | 명령 | 통과 조건 |
|---|---|---|
| `ulimit -r 99`, `ulimit -l unlimited` 유지 | `ulimit -r; ulimit -l` | `99 / unlimited` |
| `mlockall` 호출 성공 | `grep -A1 'mlockall failed' logging_data/*.log \|\| echo OK` | 경고 없음 |
| hot path 라이브러리가 `deps/install` 에서 로드 | `pmap $(pidof rt_controller_node) \| grep -E 'libfmt\|libaligator\|libmimalloc'` | 경로가 `deps/install/lib/...` |
| pinocchio/hpp-fcl 은 ROS 경로 | 위 pmap 에서 `/opt/ros/jazzy/...` | ROS 경로 유지 |
| CycloneDDS XML 경로 | `ros2 run rtc_controller_manager rt_controller_node --ros-args --log-level debug \| grep cyclone_dds.xml` | `install/.../config/cyclone_dds.xml` |
| `-march=native`, LTO | `scripts/build_deps.sh` 에 `-DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native"` 추가 | bin 에 `-mavx2` 등 symbol 존재 (`objdump -d` 샘플) |
| isolcpus / nohz_full 충돌 | `cat /proc/cmdline` | (현재 설정 없음, 격리 작업과 무관) |
| **PREEMPT_RT 커널** | `grep -c PREEMPT_RT /proc/version \|\| echo 0` | **⚠ 현재 0** — 별도 커널 설치 필요 (격리 계획과 독립) |

### 2.5 산출물 체크리스트

| 파일 | 내용 | Step |
|---|---|---|
| `deps.repos` | vcstool 매니페스트 (fmt/mimalloc/aligator + 버전) | 5 |
| `requirements.in` | Python 요구 선언 (torch 포함) | 4 |
| `requirements.lock` | pip 의존성 잠금 (pip-compile 해시 포함) | 4 |
| `scripts/build_deps.sh` | 의존성 자동 빌드 | 6 |
| `scripts/setup_env.sh` | sourcing 순서 캡슐화 | 8 |
| `.colcon/defaults.yaml` | colcon 빌드 옵션 고정 | 8 |
| `scripts/uninstall_system_deps.sh` | §8 의 대화형 정리 스크립트 | 3 |
| `ISOLATION_PLAN.md` | 이 문서 | — |
| (선택) `docs/isolation_rationale.md` | 결정 배경 (memory 의 dual-install 기록 요약) | — |
| (Q6 → §9.1) `.github/workflows/ci.yml` 또는 `Dockerfile.ci` | `deps.repos` + `requirements.lock` 로 재현 가능 빌드 | 후속 작업 |

### 2.6 위험 요소 (미해결 질문 해소됨)

1. **aligator 가 ROS hpp-fcl 에 여전히 의존** — 완전 격리가 아니므로, ROS 업그레이드 (Jazzy → Kilted 등) 시 재빌드 필요. `deps.repos` 에 hpp-fcl 핀을 추가해 Level 3.5 로 확장 가능 — 결정 보류.
2. **`system-site-packages` venv 의 누수** — `~/.local/lib/python3.12` 에 있던 `torch 2.11.0`, `cuda-*`, `cv2` 등이 venv 보다 우선 로드될 수 있음. Step 4 의 `~/.local` 정리 단계로 완화하되, 완전 차단하려면 `pyvenv.cfg` 의 `include-system-site-packages = false` 로 전환해야 함 (그 경우 `rclpy`·`ament_*`·`python_qt_binding`·`python3-pyqt5` 를 venv 에 별도 노출해야 하는 복잡성 발생). → **현 단계는 `include-system-site-packages = true` 유지 + `~/.local` 정리** 로 타협.
3. **ROS 자체가 `/opt/ros/jazzy/lib/python3.12/site-packages` 를 추가** — venv 의 `torch` 와 ROS 의 `torch`-무관 모듈은 충돌 없음. 단 `numpy` 버전이 ROS 가 요구하는 범위 밖이면 `rclpy` 임포트 실패 가능 → `requirements.lock` 은 ROS Jazzy 의 numpy 하한(1.24) 이상을 유지. 검증: `python -c "import rclpy"` 가 Step 8 후 성공해야 함.
4. **`ros-jazzy-proxsuite 0.6.5` 가 `rtc_tsid` 와 호환되지 않을 위험** — robotpkg 의 0.7.2 와 마이너 버전 차이. 단계 C 실행 **전에** Step 6 이후 `colcon test --packages-select rtc_tsid` 로 검증. 실패 시 `robotpkg-proxsuite 0.7.2` 1개만 보존 (다른 13개는 제거).
5. **robotpkg-py312-eigenpy 제거 시 Pinocchio Python 바인딩 영향** — ROS 판 `ros-jazzy-eigenpy 3.12.0` 이 동일 버전이므로 무영향 예상. 검증: `python3 -c "import pinocchio; m=pinocchio.buildSampleModelManipulator(); print(m.nq)"`.
6. **ament_export_dependencies 효과 범위** — `rtc_mpc` 가 fmt/mimalloc/aligator 를 ament_export 하므로, downstream 에서 `find_package(rtc_mpc)` 시 `RTC_DEPS_PREFIX` 가 CMAKE_PREFIX_PATH 에 없으면 실패. → `setup_env.sh` 를 반드시 source 하는 문화를 `CLAUDE.md` 에 추가 (Step 7 에 포함).
7. **개발 PC → 실 RT PC 재현성** (Q5 연관) — 본 계획은 개발 PC 용. 실 RT PC 에 동일 환경을 구축하려면 다음 산출물이면 충분:
   - `deps.repos` (Git 태그 핀)
   - `requirements.lock` (해시 포함)
   - `install.sh` (apt 의존성 설치)
   - `scripts/{build_deps,setup_env}.sh`
   - 실 RT PC 는 추가로 `linux-image-rt-generic` 또는 커널 패치 + `rtc_scripts/` 의 IRQ affinity / isolcpus 설정 필요 (§9.2).

---

## 8. 시스템 설치물 삭제 상세 (`scripts/uninstall_system_deps.sh`)

> **원칙**: (a) 워크스페이스 재빌드 전에 삭제하면 기존 바이너리 실행 불가 → Step 6 성공 **이후** 실행. (b) 각 항목은 **사용자 확인** 후 제거. (c) 삭제 전 `dpkg -L` / `install_manifest.txt` 로 파일 리스트 보존.

### 8.1 단계 A — `/usr/local` stale 잔재 제거 (2024-07-05 설치물)

**대상 (삭제)**:
- `libhpp-fcl.so` (193 MB)
- `libpinocchio_casadi.so.3.0.0` (137 MB)
- `libpinocchio_default.so.3.0.0` (145 MB)
- `libpinocchio_collision.so.3.0.0` · `libpinocchio_extra.so.3.0.0` · `libpinocchio_parsers.so.3.0.0`
- `libproxsuite-nlp.so`
- `libgtsam.so` · `libgtsam_unstable.so` · `libmetis-gtsam.so` · `libCppUnitLite.a` · `libteaser_io.so` · `libteaser_registration.so`
- `/usr/local/lib/cmake/{hpp-fcl,pinocchio,GTSAM,GTSAM_UNSTABLE,GTSAMCMakeTools,teaserpp,proxsuite-nlp,example-robot-data,jrl-cmakemodules}`
- `/usr/local/lib/python3.12/dist-packages/{hppfcl,pinocchio,proxsuite_nlp,example_robot_data}` (→ ROS 판으로 대체)

**명령**:
```bash
# 먼저 현재 상태 저장
sudo find /usr/local -maxdepth 6 \
  \( -name 'libhpp-fcl*' -o -name 'libpinocchio*3.0.0*' -o -name 'libgtsam*' \
     -o -name 'libteaser*' -o -name 'libCppUnitLite*' -o -name 'libmetis-gtsam*' \
     -o -name 'libproxsuite-nlp*' \) \
  -printf '%p\n' > /tmp/rtc_usr_local_stale_removal.txt
less /tmp/rtc_usr_local_stale_removal.txt   # 사용자 검토

# 수동 확인 후 삭제 (스크립트에 대화형 [y/N])
xargs -a /tmp/rtc_usr_local_stale_removal.txt sudo rm -i
sudo rm -rf /usr/local/lib/cmake/{hpp-fcl,pinocchio,GTSAM,GTSAM_UNSTABLE,GTSAMCMakeTools,teaserpp,proxsuite-nlp,example-robot-data,jrl-cmakemodules}
sudo rm -rf /usr/local/lib/python3.12/dist-packages/{hppfcl,pinocchio,proxsuite_nlp,example_robot_data}
sudo rm -rf /usr/local/share/example-robot-data
sudo ldconfig
```
- **롤백**: 현재 복원 소스 없음 → **삭제 전 `sudo tar -czf /tmp/usr_local_backup_${TS}.tar.gz -T /tmp/rtc_usr_local_stale_removal.txt` 필수**.
- **검증**: `ls /usr/local/lib/libhpp-fcl.so` → No such file. 워크스페이스 재빌드/실행 성공.

### 8.2 단계 B — `/usr/local` 의 fmt/mimalloc/aligator uninstall + `~/libs/` 삭제 (Q3)

```bash
# 순서 중요: aligator → mimalloc → fmt (역의존)
for pkg in aligator mimalloc fmt; do
  manifest="$HOME/libs/$pkg/build/install_manifest.txt"
  if [[ -f "$manifest" ]]; then
    echo "--- $pkg manifest ---"; cat "$manifest"
    xargs -a "$manifest" sudo rm -f
  fi
done
sudo ldconfig

# Q3 반영: 소스 디렉토리도 완전 삭제 (이미 deps/src/ 로 대체됨)
rm -rf "$HOME/libs/fmt" "$HOME/libs/mimalloc" "$HOME/libs/aligator"
# ~/libs/ 에 다른 항목이 없으면 디렉토리도 제거
rmdir "$HOME/libs" 2>/dev/null || ls "$HOME/libs"
```
- **롤백**: 진짜 롤백은 없음 (소스까지 삭제). `deps/src/` 에 git clone 상태로 동일 소스 존재하므로 필요 시 `scripts/build_deps.sh` 재실행.
- **Step 6 성공 후** 실행 — 그 전에 삭제하면 `install/` 의 기존 바이너리가 실행 불가.
- **검증**:
  ```bash
  ls /usr/local/lib/libaligator* /usr/local/lib/libfmt.so.11* 2>&1  # "No such file"
  ls ~/libs 2>&1                                                    # "No such file"
  ldd install/rtc_mpc/lib/librtc_mpc*.so | grep -E 'fmt|aligator|mimalloc'
  # → 모두 deps/install/lib/... 로 resolve
  ```

### 8.3 단계 C — robotpkg 15 개 제거 + ROS 판 proxsuite 로 전환 (Q1, Q7)

> **사전 조건**: Q7 사전 확인에서 `ros-jazzy-proxsuite 0.6.5` 존재 확정 (§"Q7 사전 확인 로그"). `rtc_tsid` 호환성은 단계 C-1 에서 검증.

**단계 C-0 — 사용 여부 재확인 (읽기 전용 · 공백 출력이어야 함)**:
```bash
grep -rE 'find_package\((casadi|blasfeo|fatrop|qpOASES|openscenegraph|gepetto|visit_struct|collada-dom|simde|jrl-cmakemodules|pythonqt|qgv|osgqt)' \
     /home/junho/ros2_ws/rtc_ws/src
# 출력이 있다면 해당 패키지는 보존 필요 — 사용자에게 보고 후 계획 수정
```

**단계 C-1 — ROS 판 proxsuite 선 설치 + rtc_tsid 호환성 검증**:
```bash
# robotpkg 경로 오염이 있는 상태에서 ROS 판만 설치 → rtc_tsid 가 어느 판을 잡는지는
# CMAKE_PREFIX_PATH 우선순위에 달림. 이 시점에서는 /opt/openrobots 가 여전히 올라가 있으므로
# 먼저 ros-jazzy-proxsuite 설치 후 robotpkg-proxsuite 를 제거하고 재빌드 테스트.
sudo apt-get install -y ros-jazzy-proxsuite
sudo apt-get remove --purge -y robotpkg-proxsuite
sudo apt-get autoremove --purge -y

# 재빌드·테스트 — 여기서 실패하면 단계 C-2 중단하고 복구
source /home/junho/ros2_ws/rtc_ws/scripts/setup_env.sh
colcon build --packages-select rtc_tsid --cmake-clean-cache
colcon test  --packages-select rtc_tsid --event-handlers console_direct+
colcon test-result --verbose
```
**실패 시 복구**:
```bash
sudo apt-get install -y robotpkg-proxsuite   # 원상 복구
# → 이 경우 robotpkg-proxsuite 와 robotpkg-jrl-cmakemodules (transitive) 는 보존
#   단계 C-2 의 목록에서 이 두 개만 제외
```

**단계 C-2 — 나머지 13~14 개 robotpkg 제거 (Q1 전체 삭제)**:
```bash
sudo apt-get remove --purge -y \
  robotpkg-blasfeo \
  robotpkg-casadi \
  robotpkg-collada-dom \
  robotpkg-fatrop \
  robotpkg-jrl-cmakemodules \
  robotpkg-openscenegraph \
  robotpkg-py312-casadi \
  robotpkg-py312-eigenpy \
  robotpkg-py312-pythonqt \
  robotpkg-py312-qt5-gepetto-viewer \
  robotpkg-qpoases+doc \
  robotpkg-qt5-osgqt \
  robotpkg-qt5-qgv \
  robotpkg-simde \
  robotpkg-visit-struct

sudo apt-get autoremove --purge -y
```

**단계 C-3 — robotpkg apt source 자체 삭제 + `/opt/openrobots` 잔재 제거**:
```bash
# apt source 제거 (다음 apt update 에서 robotpkg 재설치 방지)
sudo rm -f /etc/apt/sources.list.d/robotpkg.list
sudo rm -f /usr/share/keyrings/robotpkg-archive-keyring.gpg
sudo apt-get update

# autoremove 후에도 남는 /opt/openrobots 빈 디렉토리 정리
if [[ -d /opt/openrobots ]]; then
  find /opt/openrobots -maxdepth 2 -ls   # 잔여 파일 검토
  sudo rm -rf /opt/openrobots            # 전체 제거 (robotpkg 소유권만 남아야 함)
fi

# ~/.bashrc 의 /opt/openrobots export 는 Step 2 에서 이미 주석화됨 — 이제 삭제
sed -i '/^# ISOLATED export .*\/opt\/openrobots/d' ~/.bashrc
```

- **롤백**: Step 0 의 `rtc_ws_apt_snapshot_*.txt` 로부터 `grep robotpkg ... | awk '{print $1}' | xargs sudo apt-get install -y` 재설치 가능. robotpkg apt source 재등록 필요.
- **검증**:
  ```bash
  dpkg -l | grep -c '^ii  robotpkg-'       # 기대: 0
  ls /opt/openrobots 2>&1                  # "No such file or directory"
  env | grep -E 'openrobots'               # 빈 출력
  python3 -c "import pinocchio, eigenpy; print(pinocchio.__file__); print(eigenpy.__file__)"
  # 기대: /opt/ros/jazzy/lib/python3.12/site-packages/... 두 줄
  ros2 pkg list | grep -E '^(rtc_tsid|rtc_mpc)$'  # workspace 오버레이 작동 확인
  ```

### 8.4 단계 D — `/opt/rti.com` 제거 (Q2)

```bash
sudo rm -rf /opt/rti.com
# ~/.bashrc L125-126 의 RTI 관련 줄 제거
sed -i '/^export RTI_LICENSE_FILE=.*\/opt\/rti/d' ~/.bashrc
sed -i '/^alias ros2rti=.*\/opt\/rti/d' ~/.bashrc
```
- **검증**: `ls /opt/rti.com 2>&1 | grep -q "No such"` + `env | grep RTI_LICENSE_FILE` → 빈 출력.
- **`libspdlog-dev` 는 제거 불가** (사전 확인 완료): `ros-jazzy-spdlog-vendor` 및 `ros-jazzy-rcl-logging-spdlog` 가 역의존. 보존.

### 8.5 단계 E — `install.sh` 내 robotpkg/source-install 분기 제거 (코드 정리)

`install.sh` 에서:
- `install_pinocchio()` 의 robotpkg fallback 삭제 (ROS 판 강제)
- `install_proxsuite()` 의 robotpkg fallback 삭제 (단계 C 의 조건부와 동기화)
- `install_mpc_deps()` / `install_fmt_from_source()` / `install_mimalloc_from_source()` / `install_aligator_from_source()` 전체 → `scripts/build_deps.sh` 호출로 치환
- `~/.bashrc` 에 `/opt/openrobots` export 주입 로직 **삭제**

**별도 커밋** 으로 분리 — diff 가 커서 독립 리뷰 필요.

---

## 9. 실행 순서 요약 (Happy path)

```
Step 0   백업 (apt snapshot, ~/.bashrc, /usr/local tree, venv freeze)
Step 1   RT limits 확인 (변경 없음, 이미 OK)
Step 2   ~/.bashrc 의 /opt/openrobots + RTI env 제거
Step 4   python3 -m venv 재생성 + requirements.lock (pip-compile)
         + ~/.local 충돌 패키지 정리
Step 5   deps.repos 작성 + vcs import → deps/src/{fmt,mimalloc,aligator}
Step 6   scripts/build_deps.sh → deps/install/ 생성
Step 7   rtc_mpc/CMakeLists.txt 의 /usr/local HINTS 삭제
         install.sh 의 source-build 함수 → scripts/build_deps.sh 로 치환
         CLAUDE.md 에 "source scripts/setup_env.sh 먼저" 명시
Step 8   colcon build --cmake-clean-cache (새 env 로)
Step 9   RPATH + ldd 검증 (deps/install, /opt/ros/jazzy, libc 만 허용)
         ★ 워크스페이스 정상 동작 확인 후 다음 단계 진행
§8 A     /usr/local stale 2024 잔재 제거 (hpp-fcl, pinocchio 3.0, gtsam 등)
§8 B     /usr/local 의 fmt/mimalloc/aligator uninstall + ~/libs 삭제 (Q3)
§8 C-1   ros-jazzy-proxsuite 설치 + rtc_tsid 호환 테스트 (Q7)
§8 C-2   robotpkg 14개 purge (Q1)
§8 C-3   robotpkg apt source 삭제 + /opt/openrobots 제거
§8 D     /opt/rti.com 제거 (Q2)
§8 E     install.sh 정리 커밋 (code review)
Step 10  RT 성능 회귀 테스트 (cyclictest + 워크스페이스 jitter)
         → 결과를 본 문서에 추가 (Before/After)
```

**왜 §8 삭제 단계를 Step 9 뒤에 두는가**:
삭제 전에 `deps/install` 기반의 새 환경이 완전히 작동함을 증명해야 롤백 여지가 남음. 순서가 뒤집히면 워크스페이스가 빌드/실행 불가 상태에 진입했을 때 복구 경로가 없음.

### 9.1 CI 파이프라인 가이드라인 (Q6 재사용)

본 격리 산출물은 그대로 CI 에서 재사용 가능. 예시 GitHub Actions 스니펫:

```yaml
# .github/workflows/ci.yml (참고용, 실제 구현은 후속 작업)
jobs:
  build:
    runs-on: ubuntu-24.04
    container: ros:jazzy-ros-base
    steps:
      - uses: actions/checkout@v4
        with: { path: src/rtc-framework }

      - name: Install apt deps (subset of install.sh)
        run: |
          apt-get update
          apt-get install -y python3-venv python3-pip git \
            libeigen3-dev libyaml-cpp-dev libtinyxml2-dev \
            ros-jazzy-pinocchio ros-jazzy-proxsuite ros-jazzy-hpp-fcl \
            ros-jazzy-eigenpy ros-jazzy-behaviortree-cpp \
            ros-jazzy-ament-cmake-gtest python3-colcon-common-extensions \
            python3-vcstool

      - name: Import + build deps
        run: |
          mkdir -p deps/src
          vcs import deps/src < src/rtc-framework/deps.repos
          (cd deps/src/aligator && git submodule update --init --recursive --depth 1)
          bash src/rtc-framework/scripts/build_deps.sh

      - name: Python venv + lock
        run: |
          python3 -m venv --system-site-packages .venv
          . .venv/bin/activate
          pip install --upgrade pip
          pip install -r src/rtc-framework/requirements.lock

      - name: colcon build + test
        run: |
          . /opt/ros/jazzy/setup.bash
          . .venv/bin/activate
          export CMAKE_PREFIX_PATH="$PWD/deps/install:$CMAKE_PREFIX_PATH"
          export LD_LIBRARY_PATH="$PWD/deps/install/lib:$LD_LIBRARY_PATH"
          colcon build
          colcon test
          colcon test-result --verbose
```

핵심: `deps.repos` + `requirements.lock` + `install.sh` 의 apt 목록만 있으면 어디서든 동일 환경 재현. CI 캐시에는 `deps/install/` 전체를 올리면 재빌드 시간 단축 (`actions/cache`).

### 9.2 실 RT PC 로의 배포 (Q5 연관 · 본 작업 범위 밖, 참고)

실 RT 제어 PC 는 별도 머신. 개발 PC 에서 완성된 본 격리를 옮길 때:

1. **커널**: 실 RT PC 는 `linux-image-rt-generic` (Ubuntu 22.04/24.04 의 경우 `linux-image-<ver>-rt-generic`) 또는 소스 패치 커널 설치. `uname -r` 에 `-rt` 접미사 확인.
2. **저장소 동기화**: `git clone rtc-framework` + `vcs import deps/src < deps.repos` + `scripts/build_deps.sh`.
3. **Python**: `python3 -m venv --system-site-packages` + `pip install -r requirements.lock`. **GPU 없는 RT PC 라면 PyTorch 는 `+cpu` 휠로 교체** (`requirements.in` 에 `--extra-index-url https://download.pytorch.org/whl/cpu` 라인 추가 후 재컴파일).
4. **RT 권한**: `rtc_scripts/scripts/` 의 기존 스크립트 사용 — `@realtime` 그룹 + `limits.d` 설정 + (원하면) `isolcpus=2,3` 커널 파라미터 + IRQ affinity.
5. **CycloneDDS**: `rtc_controller_manager/config/cyclone_dds.xml` 은 상대 경로 의존 없음, 그대로 사용 가능.
6. **검증**: `cyclictest -p 90 -t 1 -n -i 1000 -D 300s` → 99.99 %ile jitter 를 로그 기준선으로 보관. 그 후 워크스페이스 RT 루프 실행.

**개발 PC (본 작업) 와 실 RT PC 의 차이점**:
- 개발 PC: vanilla 커널, soft-RT, jitter 수~수십 μs 가능. 빌드·디버그 용도.
- RT PC: PREEMPT_RT, cyclictest 기준 수 μs max. 실 로봇 연결, E-STOP 정상 동작 검증.

---

## 10. 커밋 전략 제안

| # | 커밋 메시지 | 파일 |
|---|---|---|
| 1 | `[isolation] add ISOLATION_PLAN.md (analysis, Level 3)` | `ISOLATION_PLAN.md` (이 문서) |
| 2 | `[isolation] add deps.repos + scripts/build_deps.sh` | `deps.repos`, `scripts/build_deps.sh`, `.gitignore` 에 `deps/{src,build,install}` |
| 3 | `[isolation] uv venv + requirements.lock` | `requirements.in`, `requirements.lock`, `.venv` 제외 |
| 4 | `[isolation] setup_env.sh + .colcon/defaults.yaml` | `scripts/setup_env.sh`, `.colcon/defaults.yaml` |
| 5 | `[rtc_mpc] drop /usr/local hardcode → use CMAKE_PREFIX_PATH` | `rtc_mpc/CMakeLists.txt`, `install.sh` |
| 6 | `[isolation] add uninstall_system_deps.sh` | `scripts/uninstall_system_deps.sh` |
| 7 | `[docs] CLAUDE.md: source scripts/setup_env.sh before colcon` | `CLAUDE.md` |

---

## 11. 열린 질문 — **모두 해소됨** (2026-04-21)

| Q | 답변 | 반영 위치 |
|---|---|---|
| Q1 | 전체 삭제 | §8 단계 C-2 (14개 robotpkg purge) |
| Q2 | 제거 | §8 단계 D |
| Q3 | 삭제 | §8 단계 B (`~/libs` 포함) |
| Q4 | venv 에 포함 | Step 4 `requirements.in` |
| Q5 | 본 PC 는 개발용, 실 RT PC 는 별도 | §9.2 (참고 섹션만 제공, 본 작업 범위 밖) |
| Q6 | CI 재사용 전제 | §9.1 GitHub Actions 스니펫 |
| Q7 | 사전 확인 완료 (ros-jazzy-proxsuite 0.6.5 존재) | §"Q7 사전 확인 로그", §8 단계 C-1 호환성 검증 포함 |
| 추가 | **`uv` 미사용, `venv`+`pip` 사용** | Step 4 재작성 (`python3 -m venv` + `pip-tools`) |

**남은 한 가지 의사결정** (실행 중 발생 가능):
- 단계 C-1 에서 `ros-jazzy-proxsuite 0.6.5` 가 `rtc_tsid` 와 비호환으로 드러날 경우, `robotpkg-proxsuite 0.7.2` 1개만 보존하는 부분 롤백을 허용할지 → 기본 동작은 **허용** (테스트 실패 시 자동 보존). 다르게 원하면 알려주세요.
