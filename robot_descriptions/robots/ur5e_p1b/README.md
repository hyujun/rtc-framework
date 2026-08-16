# ur5e_p1b — UR5e + proto_1b hand (closed-chain, kinematics-only test fixture)

이 디렉토리는 **테스트 전용 fixture** 다 (`panda/` 와 같은 성격). `robot_descriptions`
안에서 **유일하게 진짜 폐쇄 체인**인 모델이며, 그 경로를 CI 에서 태우기 위해 존재한다
(#457).

## 왜 이것이 필요한가 — iiwa7_leap 이 대체하지 못한다

폐쇄 체인 경로 (`GetActuatedModel()` non-null → reduced-dynamics provider, 다열
`ScatterFrameJacobian`, fingertip FK 공유) 는 **분기 자체가 검증 대상**이다.
`iiwa7_leap` 은 serial/mimic 이라 `GetActuatedModel()` 이 null 이고, repo 의 다른 폐쇄
체인인 `rtc_urdf_bridge/test/urdf/{crank_rocker,four_bar}` 는 합성 1-DoF (`control.nv == 1`)
라 다열 scatter 를 한 열도 안 태운다. 그래서 셋 다 서로를 대체하지 못한다 —
합성 fixture 는 **은퇴시키지 않는다** (1-DoF 경계·특이 케이스·residual floor 담당, #250).

이 모델이 실제로 만족하는 것:

| 요구 | 실측 |
|---|---|
| extended URDF + closure sidecar | 5 loop 로드, `GetActuatedModel()` non-null |
| `control.nv > 1` | `n_a = 16` (UR5e 6 + proto_1b 10) |
| arm sub-model + fingertip 4개 | `sub_models: ur5e(base→tool0)`, tree tip 4 |
| arm tip 이 loop 상류 | `tool0` 는 loop 밖 — 폐쇄 체인에서도 arm TCP 가 정확 |
| multi-loop | `contact_3d` × 5 (index/ring_pip/ring_tip/thumb/middle) |

## 출처 (vendored)

`hand_description` (별도 프로젝트, MIT, maintainer junho5.park@lge.com) 에서 복사했다.
그 패키지는 이 워크스페이스의 의존이 **아니고** 개발자 머신에만 있으므로, 이 fixture 를
쓰는 테스트는 CI 에서 전부 조용히 red 였다 (#452 → #454 에서 정직한 skip → #457 에서 이관).

| 파일 | 출처 | 상태 |
|---|---|---|
| `urdf/proto_1b.urdf` | `hand_description/robots/proto_1b/urdf/proto_1b.urdf` (2026-07-14, sha256 `5328e96c…`) | **바이트 동일** |
| `urdf/ur5e_with_proto_1b.closure.yaml` | `hand_description/robots/ur5e_p1b/urdf/…` (2026-07-14, sha256 `eb6a2a1b…`) | 후행 공백 1곳 제거 + 파일 끝 개행 추가 외 동일 |
| `urdf/ur5e_with_proto_1b.urdf.xacro` | 동명 파일에서 **재작성** | include 를 상대경로 → `$(find robot_descriptions)` 로, arm 을 `robots/ur5e/urdf/ur5e.urdf` 로 |

UR5e 팔은 복사하지 않았다 — 이 repo 의 `robots/ur5e/urdf/ur5e.urdf` 가 upstream 사본과
**운동학·관성 바이트 동일**하고 차이는 mesh `package://` prefix 뿐이라, xacro 가 repo 것을
그대로 include 한다.

### upstream 이 바뀌면 (drift)

vendored 사본은 조용히 갈린다. 대조는 위 표의 sha256 을 upstream 파일에 다시 돌리는 것으로
충분하다 — 바이트 동일을 유지한 이유가 그것이다. `.closure.yaml` 만 정규화 2건이 있으므로
`diff` 가 그 2줄을 낸다.

## meshes 미포함 (의도)

소비자는 `pinocchio::urdf::buildModel` / `buildModelFromXML` 만 쓴다
(`rtc_urdf_bridge/src/pinocchio_model_builder.cpp`; `buildGeom`·`GeometryModel` 참조 0건).
`proto_1b.urdf` 의 `package://hand_description/robots/proto_1b/meshes/...` 42개 참조는
**dangling 이지만 buildModel 이 무시한다** — `panda/` 가 `package://example-robot-data/...`
로 같은 상태다. prefix 를 `robot_descriptions` 로 고쳐 쓰지 않은 이유는 두 가지다: 그러면
실제로 존재하지 않는 경로를 가리켜 *거짓말*이 되고, upstream 과의 바이트 동일성이 깨져 위
drift 대조가 어려워진다. 원래 mesh 는 63 MB 이고 한 바이트도 필요 없다.

> **geometry/collision 모델 (RViz, `buildGeomFromUrdf`, MuJoCo) 이 필요한 소비자는 이
> fixture 를 쓰면 안 된다** — mesh 까지 vendor 하거나 `hand_description` 을 직접 설치할 것.

## 런타임 프로파일과의 관계

`integrated_bringup/config/ur5e_p1b/` 는 **실제 별도 로봇**의 출하 프로파일이고 여전히
`hand_description` 을 가리킨다 (mesh·MJCF scene 이 필요하다). 이 fixture 는 그 로봇의
운동학·closure 만 CI 로 들여온 것이다 — 같은 모델이지만 소비 목적이 다르다.
