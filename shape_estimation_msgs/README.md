# shape_estimation_msgs

ToF 기반 형상 추정 노드가 쓰는 메시지 · action 정의. **유형**: 메시지 패키지

## 개요

[shape_estimation](../shape_estimation/README.md) 노드의 추정 결과(`ShapeEstimate`)와 능동 탐색 action(`ExploreShape`)을 정의한다. 런타임 ToF 데이터(`ToFSnapshot`)는 로봇 독립성을 위해 `rtc_msgs` 가 갖는다. 필드의 의미 · 단위는 각 `.msg` / `.action` 주석이 SSoT 다.

## 빠른 시작

```bash
ros2 interface show shape_estimation_msgs/msg/ShapeEstimate
ros2 interface show shape_estimation_msgs/action/ExploreShape
```

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다.

## 인터페이스

| 타입 | 용도 | 토픽 · action | 노드 |
|---|---|---|---|
| [`msg/ShapeEstimate`](msg/ShapeEstimate.msg) | 추정된 형상 프리미티브 (종류 · 신뢰도 · 중심 · 축 · 치수 · 곡률 · 돌출 구조) | `/shape/estimate` | `shape_estimation_node` (발행) |
| [`action/ExploreShape`](action/ExploreShape.action) | 탐색 모션 + 형상 추정 통합 action. goal 의 `0` 값(`confidence_threshold` · `max_time_sec`)은 YAML 기본값을 쓴다 | `/shape/explore` | `shape_estimation_node` (server) |
| [`msg/ToFReadings`](msg/ToFReadings.msg) | ToF 6개 거리 + 유효성. 예약 — 소비자 없음 | — | — |
| [`msg/TipPoses`](msg/TipPoses.msg) | 핑거팁 3개 pose. 예약 — 소비자 없음 | — | — |

## 의존성

[package.xml](package.xml) — `std_msgs` · `geometry_msgs` · `builtin_interfaces` · `action_msgs`. MIT License ([LICENSE](../LICENSE)).
