# compare_mjcf_urdf 읽는 법

`compare_mjcf_urdf` 가 MJCF 와 URDF 의 무엇을 어떻게 비교하는지, 옵션이 판정을 어떻게 바꾸는지의 레퍼런스다. 실행 방법과 종료 코드는 [README](../README.md#compare_mjcf_urdf) 에 있다. `robot_descriptions/robots/model_pairs.yaml` 의 모든 쌍은 `test/test_real_model_pairs.py` 가 자동 검사한다.

## 비교 항목

| 항목 | MJCF 소스 | URDF 소스 |
|---|---|---|
| Link mass | `<inertial mass>` | `<mass value>` |
| Link COM (world frame) | `<inertial pos>` + body FK | `<inertial><origin xyz>` + link FK |
| 관성 (주모멘트) | `diaginertia`, 또는 `fullinertia` 의 고유값 | `ixx … iyz` 의 고유값 |
| 관성 텐서 (world frame) | `fullinertia`, 또는 `diaginertia` 를 inertial 자세로 돌린 것 | `ixx … iyz` 를 `<inertial><origin rpy>` 로 돌린 것 |
| Joint position limits | 걸려 있는 `range` (hinge 는 radian) | `<limit lower/upper>` |
| Joint effort limits | 아래 "관절 토크 한계" | `<limit effort>` |
| Joint axis (world frame) | `axis` (FK 변환) | `<axis xyz>` (FK 변환), 평행 검사 (anti-parallel 허용) |
| Joint position (world frame) | body `pos` + 자세 (FK 누적) | `<joint origin>` (FK 누적) |
| Armature | `<joint armature>` | 없음 (MJCF 전용, 참고 표시) |

## 구조 비교는 venv 가 필요하다

console script 의 shebang 은 `/usr/bin/python3` 이고 `mujoco` 는 workspace `.venv` 의 pip 패키지다. 그 인터프리터에서는 body 수 · 전체 질량 · 빠진 link 검사를 건너뛰고 `UNVERIFIED: 1` 을 찍는다 — 통과가 아니다. 구조 비교까지 보려면 `source repo_scripts/scripts/setup_env.sh` 뒤에 `python3 -m rtc_tools.validation.compare_mjcf_urdf …` 로 부른다. `--fail-on-unverified` 는 실행되지 못한 검사가 있으면 exit 1 로 끝낸다 — 게이트에는 필수다.

## 옵션

- **`--align-frames <MJCF_FRAME> <URDF_FRAME>`**: MJCF 는 로봇 루트 body 를 씬 작성자가 정한 자리에 mount 하고 URDF 의 world 는 루트 link 다. 두 world 가 다르면 world FK 비교가 로봇 전체 오프셋을 내는데 이는 모델 발산이 아니라 mounting 규약이다. 물리적으로 같은 프레임을 양쪽에서 하나씩 **선언**하면 갭이 닫힌다. 추정이 아니라 선언인 이유는 최소자승 fit 이 진짜 발산을 흡수하기 때문이다. 이름이 엇갈릴 수 있다 (ur5e 의 MJCF body `base` 는 URDF `base_link` 에 대응). 미지정이면 두 world 가 일치한다고 가정한다.
- **`--link-map`** (YAML/JSON): 이름이 엇갈리는 쌍의 **예외 목록**이다. 나머지 동명 쌍은 그대로 비교되고, 충돌하면 선언이 이긴다. 리포트의 `Link pairs compared: N` 과 짝을 못 찾은 목록으로 비교 범위가 좁아졌는지 볼 수 있다. link 존재 판정은 이 map 을 거친다 (ur5e 의 massless `base` 프레임과 4 kg `base_link_inertia` 처럼 같은 이름이 다른 것을 가리킬 수 있다).
- **`fuse:`** (link_map 의 structured form): MJCF 가 fixed joint 자식을 부모 body 에 접은 경우의 선언이다. `links:` (이름이 엇갈리는 쌍) 와 `fuse:` (MJCF body ← 접힌 URDF link 목록) 를 쓰고, flat form 도 그대로 동작한다 (값이 mapping 인지로 구분). 선언된 자식은 비교 전에 합성한다 — 질량 합, 질량가중 COM, 평행축 정리로 옮긴 관성 합 (URDF world 의 zero configuration 에서 합성). 합성 질량 · COM · 주모멘트는 그대로 비교되고, 자식으로 가는 경로에 fixed 가 아닌 관절이 있으면 선언 자체가 mismatch 다.
- **`--tip-frames <MJCF_FRAME> <URDF_FRAME>`**: tool 프레임을 직접 비교한다. 관절 비교는 축 직선 기준이라 마지막 관절 이후의 오프셋 (DH `d6`) 을 못 본다. MJCF 쪽은 body 또는 site 이름을 받는다. ur5e 는 `--tip-frames attachment_site tool0`.
- **`--tolerance`** (기본 1e-4): 절대 한계이며 link 질량 · 관성의 상대 한계로도 쓰인다.
- **`--joints`**: 비교할 관절 목록. 생략하면 MJCF ∩ URDF 를 자동 탐지한다.

## 판정 규칙

- **massless URDF 프레임**은 mismatch 로 세지 않고 `[NOTE]` 로 알린다. 면제는 MuJoCo 가 실제로 body 를 만들지 않은 것에 한정된다. 질량을 가진 link 의 소실은 mismatch 다.
- **질량 · 주모멘트는 크기에 맞춘 허용오차**로 비교한다. `|Δ| ≤ min(tolerance, tolerance × 크기)` 이고 크기는 질량이면 두 값 중 큰 쪽, 관성이면 그 link 의 가장 큰 주모멘트다. 1 kg · 1 kg·m² 이상에서는 절대 한계가, 그 아래에서는 상대 한계가 걸린다 (작은 link 가 절대 한계에 가려지지 않게). 양쪽이 정확히 0 이면 통과, 한쪽만 0 이면 불일치다.
- **관성은 두 번 비교한다**. 주모멘트는 회전 불변이라 어느 모멘트가 틀렸는지 짚지만 주축의 방향은 못 본다. 그래서 주모멘트가 맞은 link 는 텐서를 world frame 에서 성분별로 다시 비교하고, 틀리면 `INERTIA TENSOR MISMATCH (world)` 를 찍는다. 주모멘트 둘이 같은 link 는 그 평면 안의 주축이 어디든 같은 텐서라 불일치가 아니다. MuJoCo 의 URDF 로더는 `<inertial><origin rpy>` 를 읽지 않으므로 rpy 가 있는 URDF 를 변환한 MJCF 가 여기서 걸릴 수 있다.
- **COM 은 world frame** 에서 비교한다. 로컬 COM 은 body 원점 규약 (visual mesh 기준 대 DH 기준) 때문에 같은 물리 COM 이 다르게 읽힌다. 이 검사가 없으면 질량과 주모멘트가 그대로인 채 COM 만 옮겨진 발산이 통과한다.
- **관절 위치는 축 직선**으로 비교한다. 원점이 축 위 어디에 있든 물리가 같으므로 두 축 직선의 수직 거리만 mismatch 로 세고 축 방향 성분은 `[NOTE]` 다. prismatic 은 원점이 zero position 이라 점 비교를 유지한다.

## MJCF 를 읽는 규칙

도구는 MJCF 를 텍스트로 파싱하되 `range` 의 의미는 MuJoCo 가 컴파일하는 대로 읽는다.

- **default class 는 tree** 다. 요소의 class 는 자신의 `class` → 가장 가까운 body/frame 의 `childclass` → `main` 순이고 부모 class 의 값을 물려받는다. actuator 는 body tree 밖이라 자신의 `class` 만 본다. class 가 없는 관절은 `main` 만 받는다.
- **걸리지 않는 range 는 한계가 아니다**. `limited` · `forcelimited` · `ctrllimited` · `actuatorfrclimited` 가 `false` 면 읽지 않고, 비워 두면 `<compiler autolimits>` (기본 true) 에서 range 가 있을 때만 걸린 것으로 본다.
- **각도 기본 단위는 degree** (`<compiler angle>`). hinge `range` 와 body `euler` · `axisangle` 을 변환하고 slide 와 힘 · ctrl range 는 변환하지 않는다.
- **관절 토크 한계**: `joint` 전달 actuator 마다 순수 gain (고정 gain · bias 없음 · dynamics 없음 — `<motor>`) 이고 `ctrllimited` 면 gain × `ctrlrange`, `forcelimited` 면 `forcerange` 로 clamp, `gear` 를 곱한다. 여러 개면 더하고, 관절의 `actuatorfrcrange` 가 걸려 있으면 합을 다시 clamp 한다. 어느 쪽도 없으면 한계 없음이고 `0` 으로 읽는다.
- **clamp 는 교집합이 아니다**. 두 범위가 겹치지 않으면 MuJoCo 는 뒤에 거는 범위의 가까운 끝 한 값을 남긴다 (`forcerange="5 9"` + `ctrlrange="-2 3"` → `[5, 5]`).
- **actuator 가 없는 관절**은 class 의 `<general forcerange>` 를 한계로 읽지 않는다. URDF 에 effort 가 있으면 `MJCF=0` 불일치가 난다 — sim 이 그 관절을 구동할 수 없다는 뜻이다.
- **비대칭 힘 범위** (`-30 50`) 는 `EFFORT MISMATCH: MJCF=[-30, 50] (asymmetric)` 로 보고한다. URDF effort 는 양방향 같은 한계다.
- **body 자세**: `quat` · `euler` (`<compiler eulerseq>`) · `axisangle` · `xyaxes` · `zaxis`. `<inertial>` 자세도 같다.

## 읽지 않는 것

`model_pairs.yaml` 의 로봇 MJCF 에는 아래가 없다. 씬 · 물체 MJCF 에는 `<include>` · ball / free 관절 · geom 관성이 있다.

- **`<include>` 를 따라가지 않는다**. include 로만 들어오는 관절 · body 는 비교 대상이 아니고, root 에 적힌 관절도 include 된 `<compiler>` · `<default>` · `<actuator>` 를 못 읽어 틀린 값으로 읽힐 수 있다 (거짓 불일치 또는 거짓 OK).
- **`joint` 전달이 아닌 actuator** (`jointinparent` · tendon · site · slider-crank · body) 를 읽지 않는다. 파일에 있으면 `[WARN]` 이 나열한다.
- `<frame>` 의 pose (frame 의 `childclass` 는 읽는다), ball / free 관절의 `range`, dynamics 가 있는 actuator 의 `ctrlrange`.
- **관성은 `<inertial>` 에 적힌 값**이다. `inertiafromgeom` · `settotalmass` · `boundmass` · `balanceinertia` 는 적용하지 않는다. `<inertial>` 없이 geom 으로 관성을 얻는 body 는 link 별 비교에서 빠진다.
- `<default><site>` 의 pose 는 site 에 적용하지 않는다.
- `<option><flag clampctrl="disable"/>` · `<flag actuation="disable"/>` 를 읽지 않는다.
- 한쪽만 걸린 한계 (`forcerange="-3 inf"`) 는 한계 없음 (`0`) 이고, 한계 없음과 한계 0 을 구별하지 못한다.
- `<default>` 안의 `<position>` · `<velocity>` · `<damper>` shortcut 은 "순수 gain 이 아니다" 로만 기록한다 (`biastype` 에 따라 MuJoCo 의 실제 gain 과 어긋날 수 있다).
- MuJoCo 가 거부하는 파일 (정의되지 않은 class 등) 도 텍스트로는 읽는다. 구조 비교가 돌 때만 `MJCF failed to compile under MuJoCo` 로 알린다.
