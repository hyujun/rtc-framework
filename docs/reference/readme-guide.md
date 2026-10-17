# README 작성 가이드

README 를 새로 쓰거나 다시 쓸 때 보는 예시 모음이다. **계약 (무엇을 담고 무엇을 담지 않는가 · H2 골격 · 형태 · 언어) 은 [conventions.md](../../agent_docs/conventions.md#readme) 가 갖는다** — 이 문서는 그 계약을 적용하는 방법을 보일 뿐이고, 여기 문장이 계약과 어긋나면 계약이 옳다. 크기 예산과 형태 검사는 `repo_scripts/config/readme_budget.yaml` 과 `validate_docs.py` D14 · D15 다.

## 유형별 골격

H2 이름은 고정이고 순서도 고정이다. 유형에 따라 쓰는 절만 다르다 (— 는 생략).

| # | H2 | 노드 패키지 | 라이브러리 패키지 | 메시지 패키지 | 데이터 · 스크립트 폴더 |
|---|---|---|---|---|---|
| 1 | (H1) 이름 + 한 문장 (≤120자) + 유형 | 필수 | 필수 | 필수 | 필수 |
| 2 | `개요` — 무엇을 하는가 (≤8줄) · 누가 쓰는가 | 필수 | 필수 | 필수 | 필수 |
| 3 | `빠른 시작` — 복사 가능한 명령 | launch · run | `find_package` 스니펫 | `ros2 interface show` | 스크립트 실행 |
| — | `패키지 구성` | 루트 README 만 | | | |
| 4 | `인터페이스` | 실행파일 · launch 인자 · 토픽 · 서비스 · 파라미터 | 공개 헤더 · 주요 클래스 | 타입 색인 | — |
| 5 | `설정` — YAML 위치 + 핵심 키 | 필수 | 선택 | — | 선택 |
| 6 | `디렉토리` — 2단계 | 선택 | 선택 | — | 필수 |
| 7 | `문제 해결` — 증상 → 원인 → 조치 | 선택 | 선택 | — | 선택 |
| 8 | `더 읽기` — docs · 헤더 · rationale · 이슈 | 필수 | 필수 | 선택 | 선택 |
| 9 | `의존성` — `package.xml` 링크 · 라이선스 | 필수 | 필수 | 필수 | 선택 |

- 라이브러리와 노드가 섞인 패키지는 노드 열을 따르고, `인터페이스` 안에서 노드 표 다음에 공개 헤더 표를 둔다.
- 컨트롤러 패키지의 `인터페이스` 는 `config_key` 와 전환 명령을 갖는다.
- 빌드 명령은 패키지마다 반복하지 않고 루트 README 의 빌드 절을 링크한다.
- 100줄을 넘는 README 는 H1 아래에 H2 목차를 둔다.

## 목적지 표

README 에서 덜어낸 내용이 갈 곳이다. 옮기기 전에 목적지에 **이미 있는지** 먼저 본다 — 있으면 README 쪽을 지우기만 한다 (헤더 · `.msg` 주석에 이미 있는 내용을 다시 쓰면 그 패키지와 downstream 이 다시 빌드된다).

| README 에 있던 것 | 목적지 |
|---|---|
| 규칙 · invariant · 게이트 문장 | 이미 `agent_docs/` · `.claude/rules/` 에 있으면 삭제하고 ID 를 링크. 여러 패키지에 걸친 에이전트 판정인데 아직 없으면 소유 `agent_docs/` 문서에 추가하고 예산을 `# raised from N: <why>` 로 올린다 |
| 한 API 에 묶인 구현자 계약 (role 문자열 · lifecycle pass · 토픽 소유권) | 그 API 의 헤더 Doxygen 또는 `<pkg>/docs/<topic>.md`. `agent_docs/` 가 README 절을 SSoT 로 링크하고 있으면 README 에 남기고 압축한다 |
| 근거 · 실측 · 기각한 대안 · 결정 로그 | `docs/reference/*-rationale.md` |
| 이슈 번호 서사 · 마이그레이션 노트 | git log · 이슈 (README 는 `더 읽기` 에 링크) |
| 구현 내부 · 스레드 내부 · 클래스 구조 | 헤더 Doxygen · 코드 주석 |
| 테스트 설계 · 테스트 목록 | 테스트 파일 헤더 · [testing-rationale.md](testing-rationale.md) |
| `.msg` / `.srv` 필드 표 · YAML 키 전부 · 헤더 시그니처 | 원본 파일 (README 는 색인 + 링크) |
| 파일마다 설명이 붙은 트리 | 각 파일의 헤더 상단 주석 (README 는 디렉토리 단위) |
| 사용자용이지만 README 에 넘치는 레퍼런스 (컨트롤러별 파라미터 전표 · BT 노드 포트 표) | `<pkg>/docs/<topic>.md` — 선례: [compliance-conventions.md](../../rtc_controllers/docs/compliance-conventions.md) · [configuration.md](../../ur5e_bt_coordinator/docs/configuration.md) |
| 사람이 따라가는 여러 패키지에 걸친 how-to | `docs/<topic>.md` |

## 샘플 — 노드 패키지

````markdown
# my_node_pkg

센서 X 를 읽어 `/my/topic` 으로 내보내는 lifecycle 노드. **유형**: 노드 패키지

## 개요

한 단락으로 무엇을 하는지. 소비자: [integrated_bringup](../integrated_bringup/README.md).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다.

```bash
ros2 launch my_node_pkg my_node.launch.py use_sim:=true
```

## 인터페이스

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| launch 인자 | `use_sim` | bool | sim clock 사용 |
| publish | `/my/topic` | `rtc_msgs/msg/Foo` | 100 Hz |
| parameter | `rate_hz` | double | 발행 주기 |

## 설정

[config/my_node.yaml](config/my_node.yaml) — 기본값은 파일이 갖는다.

| 키 | 의미 |
|---|---|
| `rate_hz` | 발행 주기 [Hz] |

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 토픽이 안 나온다 | 노드가 inactive | `ros2 lifecycle set /my_node activate` |

## 더 읽기

- 설계 근거: `docs/reference/<topic>-rationale.md`

## 의존성

[package.xml](package.xml) · Apache-2.0
````

## 샘플 — 라이브러리 패키지

````markdown
# my_lib

joint-space 보간 유틸리티 (header-only). **유형**: 라이브러리 패키지

## 개요

무엇을 제공하는지 한 단락. 소비자: [rtc_controllers](../rtc_controllers/README.md).

## 빠른 시작

```cmake
find_package(my_lib REQUIRED)
target_link_libraries(my_target PRIVATE my_lib::my_lib)
```

## 인터페이스

| 헤더 | 주요 타입 · 함수 |
|---|---|
| `my_lib/interp.hpp` | `QuinticSpline` · `SampleAt()` |

## 더 읽기

- API 상세는 헤더 Doxygen

## 의존성

[package.xml](package.xml) · Apache-2.0
````

## 샘플 — 메시지 패키지

````markdown
# my_msgs

my_* 노드가 주고받는 메시지 · 서비스. **유형**: 메시지 패키지

## 개요

필드 의미 · 판정 기준은 각 `.msg` / `.srv` 주석이 SSoT 다.

## 빠른 시작

```bash
ros2 interface show my_msgs/msg/Foo
```

## 인터페이스

| 타입 | 용도 | 토픽 · 서비스 | 발행 · 서버 노드 |
|---|---|---|---|
| `msg/Foo` | 센서 X 상태 | `/my/topic` | `my_node` |

## 의존성

[package.xml](package.xml)
````

## 샘플 — 데이터 · 스크립트 폴더

````markdown
# robots/my_robot

my_robot 의 URDF · MJCF · mesh. **유형**: 데이터 폴더

## 개요

출처 · 라이선스 · 단위와 frame 규약 한 줄. 소비자: `robot_descriptions` 를 통해 로드하는 launch.

## 빠른 시작

```bash
check_urdf urdf/my_robot.urdf
```

## 디렉토리

```
my_robot/
├── urdf/     URDF · xacro
├── mjcf/     MuJoCo 모델
└── meshes/   visual · collision mesh
```
````

## 트리 쓰는 법

- 디렉토리만 2단계까지, 한 줄 60자 이내다. 파일마다 붙던 설명은 그 파일의 헤더 상단 주석으로 옮긴다 — 트리는 파일이 추가 · 삭제될 때마다 어긋났다.
- Stop hook 의 README 알림은 디렉토리가 생기거나 없어질 때 · 공개 헤더 · launch · config · `package.xml` 이 바뀔 때 뜬다. 파일 하나가 추가된 것만으로는 README 를 고칠 일이 없다.

## 다시 쓸 때 체크리스트

- [ ] 첫 문단만 읽고 무엇인지 · 누가 쓰는지 안다
- [ ] `빠른 시작` 의 명령을 복사해 그대로 실행했거나 `--help` 로 확인했다 (경로 · 실행파일 · launch 인자가 실재)
- [ ] 인터페이스 · 설정 표의 값은 YAML · 코드에서 읽었다 (C++ 기본값과 YAML 값이 다르면 YAML 을 적는다)
- [ ] 다른 파일이 절 이름이나 앵커로 인용하는 절이 살아 있다 — `validate_docs.py` D2 · D15
- [ ] 생성 블록 (`<!-- BEGIN/END GENERATED -->`) 은 손대지 않았다 (`gen_thread_layout.py --check`)
- [ ] 지운 단락은 목적지 diff 로 추적되거나, 중복 · 낡은 내용 · 이력이라서 지웠다
- [ ] `readme_budget.yaml` 에 `strict` 를 달고 예산을 새 크기로 내렸다

## 출처

README 계약을 정할 때 참고한 외부 자료 (요약과 원문 인용은 이슈 #869 의 리서치 코멘트):

- [GitHub Docs — About READMEs](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-readmes): 무엇 · 왜 · 시작법 · 도움 · 관리자. 긴 문서는 밖으로
- [Google styleguide — READMEs](https://google.github.io/styleguide/docguide/READMEs.html): "a short summary of a directory's contents", 복사 가능한 사용 명령과 관련 문서 링크
- [standard-readme](https://github.com/RichardLitt/standard-readme): 고정된 절 순서, 120자 미만 한 줄 설명, 100줄을 넘으면 목차
- [Diátaxis — how-to guides](https://diataxis.fr/how-to-guides/): how-to 는 "action and only action", 설명 · 완전성을 위한 나열은 reference 로 링크
- [agents.md](https://agents.md): 에이전트용 빌드 · 테스트 · 규약은 AGENTS.md 로 — "keeps READMEs concise and focused on human contributors"
- ROS 2 모노레포 실례: [ros2_control](https://github.com/ros-controls/ros2_control) · [ros2_controllers](https://github.com/ros-controls/ros2_controllers) · [navigation2](https://github.com/ros-navigation/navigation2) · [moveit2](https://github.com/moveit/moveit2) — 루트는 몇 화면의 진입점, 패키지 README 는 수십~수백 단어 + docs 링크. 이들은 docs 사이트가 따로 있어서, 그것이 없는 이 저장소의 패키지 README 는 명령과 인터페이스 표를 직접 갖는다
