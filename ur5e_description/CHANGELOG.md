# 변경 이력 — ur5e_description

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.17.0] - 2026-03-25

### 변경 (Changed)

- 워크스페이스 전체 버전 v5.17.0 통일
- 문서 최신화 (README.md, package.xml)

---

## [5.14.0] - 2026-03-16

### 변경 (Changed) — 문서 업데이트

- README.md: 버전 v5.14.0 업데이트
- README.md: 관절 사양 (위치 한계, 속도, 토크) 및 링크 질량 테이블 추가

---

## [5.8.0] - 2026-03-14

### 변경
- 워크스페이스 전체 버전 (v5.8.0) 통일

---

## [5.7.0] - 2026-03-11

### 추가
- `compare_mjcf_urdf` 검증 도구가 `ur5e_tools`에 추가되어 이 패키지의 MJCF(`ur5e.xml`)와 URDF(`ur5e.urdf`) 물리 파라미터 동일성을 비교 검증 가능
- `README.md`에 MJCF vs URDF 파라미터 비교 섹션 및 실행 예제 추가

---

## [5.3.0] - 2026-03-08

### 변경
- 워크스페이스 전체 버전 (v5.3.0) 통일

---

## [5.2.2] - 2026-03-07

### 추가
- **초기 생성**: `ur5e_mujoco_sim` 패키지에서 로봇 모델(MJCF, URDF, meshes)을 독립 패키지로 분리
- **메시 통합**: Universal Robots 공식 ROS2 Description 패키지에서 visual(DAE) 및 collision(STL) 메시 파일 다운로드 및 배치
- **URDF 생성**: `ur_description` xacro를 사용하여 UR5e 전용 URDF 생성 (`robots/ur5e/urdf/ur5e.urdf`)
- **MJCF 모델**: `robots/ur5e/mjcf/` 내에 MuJoCo 전용 로봇 모델 및 씬 파일 구성
- **CMake 빌드**: 빌드 시 xacro를 실행하여 URDF를 자동 생성하는 로직 추가
