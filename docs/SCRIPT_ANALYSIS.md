# Shell Script Analysis Report

> `ur5e_rt_controller/scripts/` 폴더 내 전체 스크립트 정적 분석 결과

분석 대상:
- `build_rt_kernel.sh` — PREEMPT_RT 커널 빌드 및 설치
- `setup_irq_affinity.sh` — NIC IRQ 친화성 설정
- `setup_udp_optimization.sh` — RT UDP / ROS 2 NIC 최적화

## 1. 공통 평가

| 항목 | 결과 |
|------|------|
| Bash 문법 검사 (`bash -n`) | 3개 모두 통과 |
| ShellCheck 정적 분석 | `build_rt_kernel.sh`: warning 2건 / 나머지: clean |
| 실행 권한 (`chmod +x`) | 3개 모두 설정됨 |
| Shebang (`#!/bin/bash`) | 3개 모두 존재 |
| `set -euo pipefail` | 3개 모두 적용 (엄격 모드) |
| Root 권한 확인 | 3개 모두 구현 |

## 2. 스크립트별 상세 분석

### 2.1 `build_rt_kernel.sh`

**ShellCheck 경고 (2건)**

| 라인 | 코드 | 심각도 | 내용 |
|------|------|--------|------|
| 243 | SC2010 | warning | `ls \| grep` 대신 glob 또는 for 루프 사용 권장 |
| 244 | SC2012 | info | `ls` 대신 `find` 사용 권장 |

**기능적 이슈**

1. **`--build-dir` 인자 누락 시 크래시** (라인 63)
   - `--build-dir`을 마지막 인자로 전달하고 값을 생략하면, `$2`가 unbound variable이 되어 `set -u`에 의해 스크립트가 중단됨
   - 수정 제안: `shift 2` 전에 `[[ $# -ge 2 ]] || error "--build-dir requires a value"` 검증 추가

2. **`lsb_release` 미설치 시 실패** (라인 82)
   - `lsb_release`가 없으면 error 함수로 종료하지만, 메시지가 불분명할 수 있음
   - Ubuntu minimal 설치에서는 `lsb-release` 패키지가 없을 수 있음

3. **커널/패치 버전 하드코딩** (라인 87-94)
   - Ubuntu 22.04: `6.6.127` / `6.6.127-rt69`
   - Ubuntu 24.04: `6.8.2` / `6.8.2-rt11`
   - 미러에서 해당 버전이 제거되거나 URL 구조가 변경되면 다운로드 실패 가능

4. **`ls | grep` 파이프라인** (라인 243-244)
   - 파일명에 공백이나 특수 문자가 있으면 오동작 가능
   - 수정 제안:
     ```bash
     DEB_IMAGE=$(find . -maxdepth 1 -name "linux-image-${KERNEL_VERSION}*.deb" ! -name "*dbg*" -print -quit)
     ```

5. **GRUB 설정 `sed -i` 수정** (라인 269)
   - `GRUB_CMDLINE_LINUX_DEFAULT` 형식이 예상과 다르면 패턴 불일치로 수정이 적용되지 않음
   - 백업 없이 직접 수정 (`sed -i`에 백업 suffix 없음)

6. **대화형 menuconfig** (라인 217)
   - `--batch` 없이 실행 시 `make menuconfig`가 터미널 인터랙션을 요구함
   - CI/CD 또는 자동화 환경에서는 반드시 `--batch` 플래그 필요

### 2.2 `setup_irq_affinity.sh`

**ShellCheck**: Clean (경고 없음)

**기능적 이슈**

1. **NIC 자동 감지 시 가상 인터페이스 선택 가능** (라인 43-46)
   - `docker0`, `veth*`, `br-*` 등 가상 인터페이스가 물리 NIC보다 먼저 감지될 수 있음
   - 수정 제안: `/sys/class/net/<NIC>/device` 존재 여부로 물리 NIC 필터링 추가

2. **짧은 NIC 이름으로 `/proc/interrupts` grep 시 오탐** (라인 56)
   - 예: `eth` → `ethernet-controller` 등 관련 없는 문자열까지 매칭
   - 수정 제안: `grep -w "${NIC}"` 또는 더 구체적인 패턴 사용

3. **Core 개수 하드코딩** (라인 36-37)
   - `AFFINITY_MASK="3"` (Core 0-1)은 6코어 이상 시스템 기준
   - 2코어 시스템에서는 모든 코어가 IRQ를 처리하게 되어 RT 보호 효과 없음
   - 수정 제안: `nproc`으로 코어 수 확인 후 동적으로 마스크 계산

4. **비영속적 설정**
   - 재부팅 시 IRQ affinity가 초기화됨
   - 스크립트 자체에서 안내 메시지를 출력하지만, systemd service 파일은 제공하지 않음

### 2.3 `setup_udp_optimization.sh`

**ShellCheck**: Clean (경고 없음)

**기능적 이슈**

1. **NIC 자동 감지 — 동일한 가상 인터페이스 문제** (라인 41-44)
   - `setup_irq_affinity.sh`와 동일한 로직 및 동일한 문제

2. **sysctl 설정 파일 무조건 덮어쓰기** (라인 99)
   - `/etc/sysctl.d/99-ros2-udp.conf`를 매 실행 시 새로 생성
   - 사용자가 수동으로 수정한 설정이 있으면 유실됨
   - 수정 제안: 기존 파일 존재 시 백업 생성 (`cp ... .bak`)

3. **ethtool 설정 비영속성** (라인 146-148)
   - 재부팅 시 NIC coalescing, offload, ring buffer 설정이 초기화됨
   - sysctl 설정만 `/etc/sysctl.d/`를 통해 영속됨 — 스크립트에서 이를 안내하지만 자동화 수단은 미제공

4. **UDP 버퍼 크기 2GB** (라인 106-109)
   - `rmem_max = 2147483647` (≈2GB)는 ROS 2 DDS 권장 값이지만
   - 메모리가 적은 시스템(8GB 이하)에서는 과도할 수 있음

## 3. 실행 순서 및 의존성

```
[1] build_rt_kernel.sh   →  (재부팅 필요)
[2] setup_irq_affinity.sh   →  (재부팅 불필요, 부팅 후 매번 실행)
[3] setup_udp_optimization.sh  →  (재부팅 불필요, 부팅 후 매번 실행)
```

- `build_rt_kernel.sh`는 최초 1회만 실행 (커널 빌드 → 설치 → 재부팅)
- `setup_irq_affinity.sh`와 `setup_udp_optimization.sh`는 매 부팅 후 실행 필요
- `install.sh`에서 2, 3번 스크립트를 순차적으로 호출하는 함수가 존재 (라인 562, 584)

## 4. 위험도 요약

| 이슈 | 스크립트 | 위험도 | 실행 실패 가능성 |
|------|----------|--------|------------------|
| `--build-dir` 인자 누락 크래시 | build_rt_kernel.sh | **중** | 사용자 입력 오류 시 발생 |
| 가상 NIC 잘못 감지 | irq_affinity / udp_opt | **중** | Docker 설치된 시스템에서 발생 가능 |
| Core 수 하드코딩 (2코어 시스템) | irq_affinity | **중** | 소형 시스템에서 RT 보호 무효화 |
| `ls \| grep` 파이프라인 | build_rt_kernel.sh | **하** | 특수 문자 파일명에서만 발생 |
| 커널 버전 하드코딩 | build_rt_kernel.sh | **하** | 미러 URL 변경 시 발생 |
| sysctl 설정 덮어쓰기 | udp_optimization | **하** | 수동 수정 값 유실 |
| GRUB sed 패턴 불일치 | build_rt_kernel.sh | **하** | 비표준 GRUB 설정에서 발생 |

## 5. 결론

전체적으로 3개 스크립트 모두 **문법적으로 정상**이며, 의도된 환경(Ubuntu 22.04/24.04, 6코어 이상, 물리 NIC)에서는 **정상 실행 가능**합니다.

주요 개선 권장 사항:
1. `--build-dir` 인자 검증 추가 (build_rt_kernel.sh)
2. NIC 자동 감지 시 물리 인터페이스 필터링 (irq_affinity, udp_opt)
3. CPU 코어 수에 따른 동적 affinity mask 계산 (irq_affinity)
4. `ls | grep` → `find` 또는 glob 패턴으로 교체 (build_rt_kernel.sh)
