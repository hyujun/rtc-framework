# robots/g1

Unitree G1 29-DoF 휴머노이드. 공개 원본 (전신) 과 그 파생 부동 베이스 모델, 그리고 상체 고정 + 합성 손 테스트 모델. **유형**: 데이터 폴더

## 개요

`unitree_g1_29dof/` 는 Unitree 가 공개한 `g1_29dof_rev_1_0` 의 URDF · MJCF · mesh 35 · LICENSE 를 **바이트 동일**로 둔 사본이다 (`MANIFEST.sha256` 이 기록이고 `rtc_tools/test/test_vendored_manifests.py` 가 대조한다). 고치지 않는다 — upstream 갱신은 폴더째 바꾸고 MANIFEST 를 다시 만든다.

`mjcf/g1_29dof_floating.xml` · `urdf/g1_29dof_floating.urdf` 는 보행 MPC 의 plant 로 쓰는 파생본이다. 원본과의 차이는 각 파일 머리 주석이 적는다. 요지:

- 손목 6 관절을 없애 강체로 (actuated 23). URDF 는 root 가 pelvis 그대로다 — 부동 관절은 loader 가 붙인다 (`rtc_urdf_bridge` `urdf.root_joint_type`)
- 발 sphere 4 점 → sole box 1 개, `<motor>` → `<position>` (다리 · 허리 kp 200 / kv 5, 팔 100 / 2), pelvis 의 framepos · framequat · framelinvel · frameangvel (`objtype="xbody"` — body 프레임; `body` 는 관성 프레임), keyframe `stand`, `implicitfast` · timestep 0.002/3
- 두 원본이 서로 어긋난 곳 (pelvis · torso 질량 1 g, 발목 · 허리 effort 50 vs 35) 은 URDF 를 SSoT 로 따랐다

`g1_without_right_hand_fixed_base.urdf` 는 `g1_p1b` 프로파일의 테스트가 여는 상체 모델이다. 출하 손은 공개하지 않으므로 [`p1b_surrogate`](../p1b_surrogate/README.md) (**합성 — 실제 손이 아니다**) 를 붙인다.

| 파일 | 내용 |
|---|---|
| `unitree_g1_29dof/` | Unitree 원본 (BSD-3-Clause). `MANIFEST.sha256` 과 바이트 동일 |
| `mjcf/g1_29dof_floating.xml` | 부동 베이스 plant (바닥 포함). mesh 는 `../unitree_g1_29dof/meshes` |
| `urdf/g1_29dof_floating.urdf` | 같은 모델의 URDF (손목 fixed). `model_pairs.yaml` 의 짝 |
| `g1.link_map.yaml` | 원본 MJCF 가 부모에 접은 fixed 자식 5 개의 `fuse:` 선언 |
| `urdf/g1_without_right_hand_fixed_base.urdf` | 상체 고정 모델. `hand_description` 의 같은 파일과 바이트 동일 (sha256 `a73d7872…`) |
| `urdf/g1_with_p1b_surrogate_fixed_base.urdf.xacro` | 상체 + 합성 손. 장착 변환은 `g1_p1b/_base.yaml` 의 값 |
| `urdf/g1_with_p1b_surrogate_fixed_base.closure.yaml` | 손의 loop 5 + actuated 27 (G1 17 · 손 10) |

## 디렉토리

```
g1/
├── LICENSE              Unitree BSD-3-Clause
├── g1.link_map.yaml
├── unitree_g1_29dof/    원본 (meshes/, MANIFEST.sha256)
├── mjcf/                파생 부동 MJCF
└── urdf/                파생 부동 URDF · 상체 fixture
```

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `test_vendored_manifests` 가 실패한다 | 원본 폴더의 파일이 바뀌었거나 추가됐다 | 원본은 고치지 않는다. upstream 갱신이면 MANIFEST 를 다시 만들고 파생본을 다시 유도한다 |
| `test_real_model_pairs` 가 `g1_29dof_floating` 에서 실패한다 | 파생 MJCF · URDF 중 한쪽만 고쳤다 | 둘을 같이 고친다. URDF 가 SSoT |
| 상체 fixture 에서 mesh 를 못 찾는다 | `package://hand_description/...` 참조가 dangling 이다 (`buildModel` 만 무시) | 형상이 필요하면 `urdf/g1_29dof_floating.urdf` 를 쓴다 |

## 더 읽기

- 파생 규칙 · 게이트 수치의 출처: #975 (W0-F01), 레이아웃 결정: #974 (WD-13)
