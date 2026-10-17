# robots/panda

Franka Emika Panda 7-DoF 의 URDF 한 개. **유형**: 데이터 폴더 (kinematics-only 테스트 fixture)

## 개요

`rtc_tsid` · `rtc_mpc` · `integrated_bringup` 의 gtest 가 generic fixed-base 7-DOF 모델로 `pinocchio::buildModel` 에 쓰는 **테스트 입력**이다. 출처는 `example-robot-data` (Gepetto, BSD-2-Clause) 의 `robots/panda_description/urdf/panda.urdf` 이며 그대로 복사했다. 소스 트리가 없는 환경에서도 테스트가 돌도록 `robot_descriptions` 의 ament share 경로에 vendor 했다.

## 디렉토리

```
panda/
└── urdf/   panda.urdf (mesh 없음)
```

mesh 는 의도적으로 넣지 않았다. 소비자가 `pinocchio::urdf::buildModel` (kinematics/dynamics 파싱) 만 쓰므로, URDF 의 `package://example-robot-data/...` mesh 참조는 dangling 이어도 무시된다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| RViz · `buildGeomFromUrdf` 에서 mesh 를 못 찾는다 | geometry/collision 용 자산이 없다 | 이 fixture 를 쓰지 않는다. mesh 까지 vendor 하거나 `example-robot-data` 를 직접 설치한다 |
