# panda — Franka Emika Panda (kinematics-only test fixture)

이 디렉토리는 **테스트 전용 fixture** 다. rtc_tsid / rtc_mpc 의 gtest 가 generic
fixed-base 7-DOF 모델로 사용하는 `urdf/panda.urdf` 만 vendor 한다.

## 출처

`example-robot-data` (Gepetto, BSD-2-Clause) 의
`robots/panda_description/urdf/panda.urdf` 를 그대로 복사한 것이다.
이전에는 vcs-imported `deps/src/example-robot-data` 또는 legacy `/usr/local` 에서
참조했으나, deps 아티팩트가 `deps/install` 만 보존(소스 트리 삭제)하여 CI 에서
부재 → 테스트 fixture 의 `pinocchio::buildModel` 이 throw → 미실행. 이를
robot_descriptions 의 ament share 경로로 vendor 하여 로컬·CI 일관성을 확보한다.

## meshes 미포함 (의도)

소비자는 `pinocchio::urdf::buildModel` (kinematics/dynamics 파싱) 로만 사용하므로
visual/collision mesh 가 필요 없다. URDF 의 `package://example-robot-data/...` mesh
참조는 dangling 이지만 buildModel 이 무시한다. **geometry/collision 모델(RViz,
buildGeomFromUrdf)이 필요한 소비자는 이 fixture 를 쓰면 안 된다** — 다른 로봇처럼
meshes/ 까지 vendor 하거나 example-robot-data 를 직접 설치할 것.
