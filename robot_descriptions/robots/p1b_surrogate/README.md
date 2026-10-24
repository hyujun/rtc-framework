# robots/p1b_surrogate

**합성 — 실제 손이 아니다.** 폐쇄 체인 테스트용 가상 손과 UR5e 결합 모델. **유형**: 데이터 폴더 (kinematics/dynamics 테스트 fixture)

## 개요

출하 프로파일 `ur5e_p1b` · `g1_p1b` 의 손 (proto_1b) 은 별도 저장소 `hand_description` 에 있고 그 파일은 공개하지 않는다. 테스트는 그 대신 이 모델을 연다. 링크 · 관절 이름과 loop 구조 (손가락 4개, actuated 10, `contact_3d` loop 5 — 엄지 · 검지 · 중지에 4절 링크 하나, 약지에 두 개가 직렬) 만 실제 손과 같고, **길이 · 오프셋 · 질량 · 관성은 이 파일을 위해 지어낸 값**이다. 실제 손 파일에서 가져오거나 유도한 수치는 없다.

출하 config 가 읽는 규약은 그대로 둔다: 관절 축, `integrated_bringup/config/ur5e_p1b/_base.yaml` 의 actuated 위치 한계, palm 프레임 (손가락 +y, 굽힘 +z, flange 에 rpy (90°, 0, 90°)), fingertip bracket 프레임의 회전. 그래서 출하 config 의 catch frame (`l_palm_link` 기준) 과 fingertip 이름이 이 모델에서 그대로 성립한다. 손 치수나 질량에 기대는 테스트 수치는 실제 로봇에 대해 아무것도 말하지 않는다.

각 4절 링크는 부모 링크 평면의 (s, t) — s 는 손가락 방향, t 는 굽힘 쪽 — 에서 xacro 식으로 배치했다. 두 constraint frame 이 관절과 같은 식으로 놓이므로 q = 0 에서 모든 loop 가 구성상 닫힌다 (이 모델의 reference 자세). 치수는 actuated 범위 양끝에서 0.3 rad 더 나간 구간까지 조립되고, 전달각이 직각에서 크게 벗어나지 않으며 (5 loop 모두 sin μ ≥ 0.94), 다른 조립 분기가 2 rad 넘게 떨어지도록 골랐다. 치수를 바꾸면 이 성질을 다시 확인한다.

## 디렉토리

```
p1b_surrogate/
└── urdf/   손 · UR5e 결합 xacro, closure.yaml
```

G1 에 붙인 결합 모델은 [`robots/g1/`](../g1/README.md) 에 있다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 실제 손과 fingertip 위치 · 질량이 다르다 | 합성 모델이다 | 실제 값이 필요하면 `hand_description` 을 직접 쓴다 (이 저장소의 테스트에서는 쓰지 않는다) |
| 치수를 바꾼 뒤 closure 사영이 수렴하지 않는다 | 4절 링크가 범위 안에서 조립 불가 · 특이 자세가 됐다 | 범위 전체에서 조립 · 전달각 · 분기 거리를 다시 잰다 |
