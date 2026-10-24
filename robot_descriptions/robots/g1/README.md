# robots/g1

Unitree G1 상체 (고정 base, 오른손 없음) 와 거기에 합성 손을 붙인 테스트 모델. **유형**: 데이터 폴더 (kinematics/dynamics 테스트 fixture)

## 개요

`g1_p1b` 프로파일의 테스트가 여는 모델이다. 출하 모델은 별도 저장소 `hand_description` 에 있고 손 파일은 공개하지 않으므로, G1 본체는 그대로 vendor 하고 손은 [`p1b_surrogate`](../p1b_surrogate/README.md) (**합성 — 실제 손이 아니다**) 로 대신한다.

| 파일 | 내용 |
|---|---|
| `urdf/g1_without_right_hand_fixed_base.urdf` | Unitree `g1_29dof_rev_1_0.urdf` 에서 다리 관절을 고정하고 오른손 palm 을 뺀 모델. `hand_description` 의 같은 이름 파일과 바이트 동일 (sha256 `a73d7872…`) |
| `urdf/g1_with_p1b_surrogate_fixed_base.urdf.xacro` | world → pelvis (G1 MJCF 의 선 자세 높이) + 오른쪽 `right_wrist_yaw_link` 에 합성 손. 장착 변환은 `g1_p1b/_base.yaml` 이 적은 값 |
| `urdf/g1_with_p1b_surrogate_fixed_base.closure.yaml` | 손의 loop 5 + actuated 27 (G1 17 · 손 10) |
| `LICENSE` | Unitree 의 BSD-3-Clause |

## 디렉토리

```
g1/
├── LICENSE
└── urdf/   G1 URDF · 결합 xacro · closure.yaml (mesh 없음)
```

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| RViz · `buildGeomFromUrdf` · MuJoCo 에서 G1 mesh 를 못 찾는다 | mesh 를 vendor 하지 않았다. URDF 의 `package://hand_description/...` mesh 참조는 dangling 이고 `buildModel` 만 이를 무시한다 | 이 fixture 를 형상 용도로 쓰지 않는다 |
| G1 URDF 의 mesh prefix 를 고치고 싶다 | 바이트 동일성이 깨져 upstream 대조가 어려워진다 | 고치지 않는다 |
