# robots/ur5e_p1b

UR5e + proto_1b hand 의 URDF · closure sidecar. **유형**: 데이터 폴더 (kinematics-only 폐쇄 체인 테스트 fixture)

## 개요

`robot_descriptions` 안에서 유일한 실규모 폐쇄 체인 (5 loop, `n_a = 16`) 이다. 폐쇄 체인 경로 (`GetActuatedModel()` non-null → reduced-dynamics provider, 다열 `ScatterFrameJacobian`, fingertip FK 공유) 는 분기 자체가 검증 대상인데, `iiwa7_leap` 은 serial/mimic 이라 이 경로를 타지 않고 `rtc_urdf_bridge/test/urdf/{crank_rocker,four_bar}` 는 1-DoF 합성 모델이라 다열 scatter 를 태우지 못한다. 그래서 서로 대체되지 않는다.

이 사본은 rtc-framework 가 소유한 **테스트 입력**이다. 모델의 소유자는 별도 저장소 `hand_description` 이고, 여기서는 URDF · closure 를 load 해서 **이 저장소의 코드**(폐쇄 체인 경로 · 포구 supervisor · 계획기 · inference) 를 검증한다. 관성 유효성이나 모델 차원 같은 **모델 자체에 대한 단언은 두지 않는다**.

`integrated_bringup/config/ur5e_p1b/` 는 같은 모델의 실제 출하 프로파일이며 mesh · MJCF scene 때문에 여전히 `hand_description` 을 가리킨다.

## 디렉토리

```
ur5e_p1b/
└── urdf/   proto_1b.urdf · xacro · closure.yaml (mesh 없음)
```

| 파일 | 출처 (`hand_description`) | 상태 |
|---|---|---|
| `urdf/proto_1b.urdf` | `robots/proto_1b/urdf/proto_1b.urdf` (2026-07-14, sha256 `5328e96c…`) | 바이트 동일 |
| `urdf/ur5e_with_proto_1b.closure.yaml` | `robots/ur5e_p1b/urdf/…` (2026-07-14, sha256 `eb6a2a1b…`) | 후행 공백 1곳 제거 + 파일 끝 개행 추가 외 동일 |
| `urdf/ur5e_with_proto_1b.urdf.xacro` | 동명 파일에서 재작성 | include 를 `$(find robot_descriptions)` 로, arm 을 `robots/ur5e/urdf/ur5e.urdf` 로 |

UR5e 팔은 복사하지 않았다. 이 저장소의 `robots/ur5e/urdf/ur5e.urdf` 가 upstream 사본과 운동학·관성이 바이트 동일하고 차이는 mesh `package://` prefix 뿐이라 xacro 가 그대로 include 한다. `hand_description` 은 이 워크스페이스의 의존이 아니고 개발자 머신에만 있다.

upstream 과의 drift 는 위 표의 sha256 을 upstream 파일에 다시 돌려 대조한다 (`.closure.yaml` 만 `diff` 가 정규화한 2줄을 낸다). 사본을 upstream 에 맞춰 두는 것은 테스트가 요구하지 않는다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| RViz · `buildGeomFromUrdf` · MuJoCo 에서 mesh 를 못 찾는다 | mesh 를 vendor 하지 않았다 (원본 63 MB). `proto_1b.urdf` 의 `package://hand_description/...` mesh 참조는 dangling 이고 `buildModel` 만 이를 무시한다 | 이 fixture 를 쓰지 않는다. mesh 까지 vendor 하거나 `hand_description` 을 직접 설치한다 |
| prefix 를 `robot_descriptions` 로 고쳐 쓰고 싶다 | 존재하지 않는 경로를 가리키게 되고 upstream 과의 바이트 동일성이 깨져 drift 대조가 어려워진다 | 고치지 않는다 |
