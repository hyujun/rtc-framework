# L1 — IO: `PointCloud2` 예측 궤적 수신·파싱·검증, 스냅샷 브리지, 로봇·센서 상태

이 문서는 현재 구현의 입력 층 (vision 예측 궤적의 수신 · 검증 · 스냅샷 게시, 그리고 계획기로 가는 RT 상태) 을 표현한다.

- 배치 `[확정 D-1]`: `PointCloud2` 구독·필드 이름 파서는 `integrated_bringup` 바인딩 (`integrated_bringup/include/integrated_bringup/controllers/catching/traj_input.hpp`), ROS 비의존 판정 로직(D-2 시간 변환, 순서·`generation`·`validity`·stale 판정, 점 개수 검사)은 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`, `traj_ingress.hpp` · `time_types.hpp` · `trajectory.hpp`)
- 구성: 필드 이름 파서, 수신 콜백, 궤적 스냅샷 브리지(`rtc::SeqLock`), 계획기 공분산 버퍼 전달, 계획기로 가는 RT 상태 POD, 진단 카운터
- 의존: L0, L2 (공용 궤적 타입)

---

## 1. 범위 / 비범위

범위: vision(ball_perception)의 `sensor_msgs/PointCloud2`(마스터 §5.1, D-4)를 검증해 공용 궤적 타입(L2 §5.1)으로 바꾸고 RT와 계획기에 전달한다. 계획기로 넘기는 RT 상태 POD 를 제공한다. 스냅샷 나이(stale), 시계 이상, 순서 역전·중복, 트랙 교체를 판정한다.

비범위:
- 좌표 변환 조회를 RT에서 수행하는 것(금지 — RT 경로에 tf2 없음). `frame_id`→`world` 가 다르면 configure 에서 한 번 읽어 캐시한 정적 변환을 nrt 콜백에서 적용한다. sim 에서는 불필요 — `frame_id` = `world`; 실기 카메라 프로파일이 다른 frame 을 내면 그때 켠다.
  - **`world` → 모델 world 는 별개이고 항상 필요하다 (L3 §4.2 의 frame 규약).** 계획기·CLIK 은 pinocchio universe (URDF 모델 root) 좌표를 쓰는데, ur5e_p1b 의 root 는 `base_link` 라 `world` (= `base`) 와 z 둘레 180° 다르다. 이 변환이 없으면 후보가 팔 뒤로 간다. nrt 수신 시 한 번 $p,v,a$ 와 6×6 공분산 ($R_6\Sigma R_6^\top$, 정확히 0 인 회전 계수는 건너뛰어 NaN(모름) 이 섞이지 않게) 에 적용하고, 그 뒤의 모든 소비자는 모델 world 를 본다. 변환은 `model_world_T_world` = (모델에서 읽은 `io.arm_base_frame` 배치) · `io.base_T_world` — 지도 도구의 `--arm-base-frame`·`--world-yaw-deg`·`--world-translation-m` 과 같은 분해. 항등이면 적용하지 않는다 (iiwa7_leap)
- 궤적 예측·전파(vision 노드, 마스터 §5.2).
- RT 원시형 구현 — `rtc::SeqLock`·`rtc::SpscQueue` 를 쓴다(G1-8).

입력 레이아웃은 ball_perception 의 레이아웃이다(D-4, G1-1). 시간은 수신 시 1회 절대 steady 시각으로 변환한다(D-2, §4.1). 트랙 식별은 vision 의 `generation` 이 맡고(§4.4), `validity`·`snapshot_sequence` 로 미평가 예측과 순서 역전을 거부한다. 공분산은 RT 스냅샷에서 분리해 계획기 버퍼로만 보낸다(A-3).

## 2. 코드 확인 게이트

본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 확인된 사실 |
|---|---|---|
| G1-1 | `PointField` 배열 실제 레이아웃 | `[확정 D-4]` — little-endian. 필드: `x,y,z,vx,vy,vz,ax,ay,az` FLOAT64, `covariance` FLOAT64×36 (row-major $p_x..v_z$, 모르면 NaN), `snapshot_sequence` · `generation` 은 UINT32×2 (uint64 low/high), `horizon_ns` UINT32, `validity` UINT8 (0 NOT_EVALUATED, 1 VALID). **offset 수치는 이 문서에 두지 않는다 — 코드 헤더 `integrated_bringup/include/integrated_bringup/controllers/catching/traj_input.hpp` 와 `BuildFieldMap` 이 갖고, 파서는 이름으로 찾는다** (§4.6). debug 토픽이라 stable ABI 아님 (W5-2) |
| G1-2 | 토픽 이름과 QoS | ball_perception `sim_estimator_node` 의 debug 예측 궤적 토픽 (`io.traj_topic`). QoS depth 는 `KEEP_LAST(1)` 고정(ARCH-6, invariants.md) — TBD 대상이 아니다. reliability 는 **`best_effort` 로 고정**한다 (코드 고정, 설정 키 아님 — publisher 는 RELIABLE/VOLATILE 이라 둘 다 호환하고, best_effort KEEP_LAST(1) 구독은 reliable 구독과 같은 메시지를 받는다) |
| G1-3 | 점 시각 필드 타입·기준 | `t` 필드는 없다. `horizon_ns` (UINT32, `header.stamp` 기준 상대 ns), `header.stamp` = 예측 원점 시각 |
| G1-4 | `header.frame_id`와 `world`의 관계 | **`world` 그대로** (TBD-VIS-06: 예측 토픽 `frame_id` = `world`, 프로파일 `input.frame_transform.source: identity`). 이 변환은 없다 — 모델 world 변환은 §1 |
| G1-5 | 트랙 식별·소실 판정 수단 | `generation` (트랙 epoch), `validity`. 유령 트랙은 **없다** (TBD-VIS-07: 공이 계속 날아도 입력이 끊기면 `VALID` 예측은 다음 발행 tick 한 건까지만 나오고 그 뒤 **침묵**). 단 INVALID 스냅샷은 발행되지 않는다 — 소실은 `track_status` (diagnostics) 나 수신 나이로만 안다. 그래서 `io.t_stale` 이 필수다 |
| G1-6 | subscription callback이 도는 executor / callback group | 컨트롤러 소유 구독은 LifecycleNode default group → `nrt_callback_executor` (단일 스레드, lifecycle 서비스와 공유) |
| G1-7 | RT tick 시각과 스탬프 clock 의 관계 (sim) | `[확정 D-2, D-3]` — RT tick 은 `RTControllerInterface::Compute(const ControllerState&)`, 시각은 steady. 스탬프는 wall (`rtc_mujoco_sim`·`sim_estimator_node` 모두 `use_sim_time=false`, `/clock` 없음). wall→steady 는 §4.1 변환 1회 |
| G1-8 | SeqLock/SPSC 원시형 API와 재시도 정책 | `rtc::SeqLock::Store`/`Load`/`sequence`. `Load` 는 **재시도 상한 없이** 일관된 사본을 얻을 때까지 반복한다(단일 writer·유한 쓰기 시간이 설계 불변식). 비-RT writer(nrt 콜백) → RT reader 는 backend 3종이 관절 상태에 이미 쓰는 경로이므로 새 primitive 가 아니다 — 최악 재시도 시간은 G1-C 로 측정한다(D-21). payload 는 trivially copyable (L0 §5.2) |
| G1-9 | 지문 센서·손 상태 경로와 규약 | 실기 P1b `HandSensorState` 250 Hz, sim `WrenchStamped`. 두 경로 모두 finger-on-object 부호. 센서 lane 의 freshness 는 **D-24 (a)**: `rtc_base` `DeviceState` 센서 lane 에 수신 시각 · sequence · valid 를 싣는다 (TBD-HAND-03) |

## 3. 참고자료

[R8] 공분산 좌표 변환, [R12] PTP. `sensor_msgs/PointCloud2` 필드 규약은 ROS 2 인터페이스 정의(1차 출처: `sensor_msgs` 패키지).

## 4. 수학적 이론

### 4.1 시각 변환과 스냅샷 나이 `[확정 D-2]`

시간 규약 (판정별 비교 축) 은 L0 §4.5 가 갖는다. 이 절은 수신 쪽 변환과 `header.stamp` 사용 계약이다. 수신 콜백(nrt)은 **도착 즉시** steady·wall 시각을 한 쌍으로 찍고, 원격 스탬프를 한 번만 절대 steady 시각으로 바꾼다.

$$t_{ref}^{steady}=t_{recv}^{steady}-\big(t_{recv}^{wall}-t_{stamp}\big),\qquad \mathrm{BallTime}_j=t_{ref}^{steady}+\texttt{horizon\_ns}_j$$

- 이후 RT·계획기는 `BallTime`(절대 steady ns)만 본다. 원점이 다른 상대시각을 섞지 않는다. 원격 stamp 를 시간 원점으로 쓰는 것은 D-2 가 명문화하는 E-1 예외다.
- **stale·나이는 steady 수신 나이** $t_{now}^{steady}-t_{recv}^{steady}$ 로만 판정한다(L0 §4.5). `header.stamp` 로 staleness 를 판정하지 않는다([invariants.md](../../../agent_docs/invariants.md)).
- 원점 지연 $t_{recv}^{wall}-t_{stamp}$ 는 **진단**(분포 기록)이다. 음수가 $T_{future}$ 보다 크면(미래 스탬프) 변환 결과가 틀리므로 시계 이상으로 거부한다.
- stale 임계 $T_{stale}$: 발행 주기 + 여유 (YAML `io.t_stale`).
- **`header.stamp` 사용 계약.** stamp 는 아래 한 곳에서만, 한 번만 쓴다.

| 용도 | 쓰는 값 | 비고 |
|---|---|---|
| 물리 샘플 시각 복원 | $t_{ref}^{steady}=t_{recv}^{steady}-(t_{recv}^{wall}-t_{stamp})$, 수신 콜백에서 1회 | E-1 기록된 예외의 유일한 대상 |
| freshness · stale · watchdog | $now_{steady}-recv_{steady}$ | stamp 를 쓰지 않는다 |
| 원점 지연 $t_{recv}^{wall}-t_{stamp}$ | 진단 발행 (분포 · 점프) | 양수 쪽 (오래된 stamp) 의 나이 거부는 두지 않는다 — 오래된 원점은 지평 검사가 거른다 |
| 미래 stamp ($t_{recv}^{wall}-t_{stamp}<-$`io.future_tol`) | 메시지 거부 + 카운터 | 변환 신뢰 불가 판정 (fail-closed) |
| 절대 시각 지평 검사 | 마지막 점 `BallTime` 대 `now_lead` | feasibility 판정이며 deadline 이 아니다 |

- **E-1 예외의 조건** (전문: [invariants.md](../../../agent_docs/invariants.md) §Clock 시간축 규칙). 다음을 모두 만족할 때만 허용하고 하나라도 깨지면 E-1 이다: ① freshness · stale · watchdog 은 수신 나이로만 판정, ② 미래 방향 보정항이 `future_tol` 을 넘으면 거부하고 센다, ③ 송 · 수신이 같은 호스트의 `CLOCK_REALTIME` 을 공유하거나 PTP 동기가 검증된 경우로 한정 (실기는 단계 진입 전에 재확인), ④ 보정항 분포를 진단으로 발행한다. 이 예외는 다른 토픽의 근거가 아니다. 변환이 $t_c$ · $t_{cmd}$ 같은 deadline 판정을 stamp 에서 파생시키므로 wall clock 점프는 그 판정 오차로 그대로 들어간다 — `header.stamp` 로 staleness · E-STOP 을 판단하는 것은 여전히 금지다.
- 지평 끝 소진: 마지막 점 `BallTime` 을 **now_lead** 와 비교한다(L0 §4.5 표의 "궤적 지평 끝 경고"). 이는 feasibility 판정이지 deadline 이 아니다 — 원점 오차는 지평을 짧게 보이게 하는 fail-closed 방향이다. 샘플링이 선행축으로 읽으므로 소진 판정도 같은 축이어야 한다 — 실제 나이를 지평 상대시각과 비교하면 두 축이 섞인다.
- **지평 요구 (D-15).** 수신 궤적의 지평(마지막 점 `horizon_ns`)이 제어기 요구 `io.horizon_min` 보다 짧으면 계획 후보에서 제외하고 진단한다. 요구값은 $R_1$ (commit 조건) 기준으로 정한다: $T_{freeze}+L=(T_{close,tot}+T_{arm}+T_{margin})+L$ 을 10 ms 로 올림한다 (내림하면 $R_1$ 을 미달하는 궤적을 통과시킨다). sim profile 의 지평 · 간격 · 점 수는 목표 분포 요구 $H_{req}$ (기구학 reachable 창 + $T_{det}$) 가 정하며, 값은 vision profile 파일과 `io.horizon_min` 에 있다 (D-15 · D-27). ball_perception 의 예시 profile (0.5 s / 최대 10 점) 은 그대로는 부족하다.

### 4.2 시계 오차의 영향

§4.1 변환은 전송 지연을 정확히 보상하고 두 PC 시계 오프셋 $\delta$ 만 남긴다. $\delta$ 는 보상 불가능한 위치 오차 $\approx\Vert v\Vert\,\delta$ 를 만든다(1차 근사). 6 m/s에서 1 ms는 6 mm다. $T_{future}$와 PTP offset 임계는 포구 허용오차 예산에서 역산해 정한다(L3 §4.6). sim 은 한 PC 의 wall clock 이라 $\delta=0$ 이다(D-3).

### 4.3 좌표 변환과 공분산

계획과 기준 생성은 `W`에서 수행한다. `header.frame_id`가 `W`가 아닌 경우만 변환한다(G1-4). **네 가지를 모두 변환해야 한다.**

$$p'=Rp+t,\qquad v'=Rv,\qquad a'=Ra,\qquad \Sigma'=T\Sigma T^\top,\quad T=\mathrm{diag}(R,R)$$

속도·가속도에는 평행이동이 들어가지 않는다. 공분산은 6×6이므로 $T$가 $\mathrm{blkdiag}(R,R)$ 이다. 변환은 configure에서 캐시한 $R,t$ 로 nrt 콜백에서 수행하고, 공분산 변환 결과는 계획기 버퍼로만 간다(A-3). NaN(모름) 원소는 변환하지 않고 NaN 그대로 둔다 — 행·열에 NaN 이 섞인 상태로 곱하면 알려진 원소까지 오염된다.

frame이 움직이는 경우(예: 카메라가 로봇에 달린 경우)는 본 설계 범위 밖이다.

### 4.4 트랙 식별과 순서 `[확정 D-4]`

- **`generation` (uint64) 이 트랙 epoch 다.** 값이 바뀌면 새 트랙이다. 스냅샷에 그대로 싣고, L3·L7 이 이 값의 변화를 트랙 교체로 해석한다(L3 의 `n_settle` 등). 제어 PC 는 자체 `track_epoch` 를 만들지 않는다.
- **`snapshot_sequence` (uint64)** 는 같은 generation 안에서 직전 수락값 이하이면 **중복·순서 역전으로 거부**한다 (늦게 도착한 옛 예측이 새 예측을 덮어쓰면 안 된다). generation 이 바뀌면 기대값을 리셋한다 (A-S5-4) — 새 epoch 은 번호를 아무 값에서나 다시 시작할 수 있다.
- **`validity`** 는 점 단위 필드다. `VALID`(1) 가 아닌 점(`NOT_EVALUATED` 또는 미지 값)이 하나라도 있으면 메시지를 거부한다 (C-1, 부분 수용 없음 — fail-closed).
- 두 uint64 필드는 UINT32×2 (low, high) 로 온다: $x=\mathrm{low}+2^{32}\cdot\mathrm{high}$. 모든 점이 같은 값을 가져야 하며 점 0 과 다르면 형식 오류로 거부한다.

**궤적 점프 (진단).** 같은 generation 의 직전 궤적과 새 궤적을 **같은 절대 시각** $t^\ast$ 에서 샘플링해 비교한다.

$$J=\big\Vert \hat p_{new}(t^\ast)-\hat p_{old}(t^\ast)\big\Vert,\qquad t^\ast=\max(t^{steady}_{ref,new},\,t^{steady}_{ref,old})+\Delta_{eval}$$

$J>J_{warn}$ 이면 경고만 기록한다(트랙 판정에는 쓰지 않는다). 이 값은 L3의 교체 히스테리시스(L3 §4.7)와 L4의 재예측 점프(L4 §4.3)가 감당해야 할 크기와 같은 양이므로 분포를 함께 본다.

### 4.5 예측 일관성 감시 `[논문 외 설계]`

**이 절은 구현하지 않았다** — $\bar\nu$ 의 생산자가 없고 `io.pred.nu_*` 키는 없다. 식은 설계로 남긴다.

연속 두 메시지를 같은 절대 시각에서 비교하면 혁신(innovation) 대용 통계를 만들 수 있다. §4.4의 $J$ 를 공분산으로 정규화한다.
$$\nu=J^\top\big(\Sigma_{pp,new}(t^\ast)+\Sigma_{pp,old}(t^\ast)+\lambda I\big)^{-1}J$$

- **정규화 역행렬 (NUM-1).** 합 공분산은 특이·악조건일 수 있으므로 $\lambda I$ 를 더한 뒤 Cholesky 로 푼다. 실패하거나 원소에 NaN(모름)이 있으면 그 쌍의 $\nu$ 는 계산하지 않는다(카운트만).
- **공분산의 시각 규칙.** $t^\ast$ 는 일반적으로 두 점 사이다. 계획기가 쓰는 규칙은 있다 — **가장 가까운 표본**의 $6\times6$ 을 등속 전이로 $t^\ast$ 까지 전파한다 ($F\Sigma F^\top$, `SampleBallNode`, L3 §5.3). 원소별 선형 보간은 쓰지 않는다. 이 절의 $\nu$ 는 구현하지 않았으므로 두 예측의 쌍에 그 규칙을 쓸지는 **정해지지 않았다.**
- 두 예측이 독립이고 공분산이 정직하면 $\nu\sim\chi^2_3$, $\mathbb E[\nu]=3$. 창 길이 $N$ 의 평균이 $\bar\nu\notin\big[\tfrac1N\chi^2_{3N}(\alpha/2),\ \tfrac1N\chi^2_{3N}(1-\alpha/2)\big]$ 이면 경고한다([R8], 경계는 configure 에서 미리 계산). 두 예측은 겹치는 관측을 쓰므로 독립이 아니다 — $\bar\nu$ 는 절대 임계보다 **추세**로 본다.
- 계산은 공분산이 아직 손에 있는 nrt 콜백에서 한다(RT 에는 공분산이 없다, A-3).

$\bar\nu$ 가 쓰였다면 세 곳이다: L3의 $\kappa_\sigma$ 보정 근거(L3 §4.4), L7의 `PRED_INCONSISTENT` 사유(L7 §4.2), L8 지표. 구현하지 않았으므로 L7 `PRED_INCONSISTENT` 는 발화하지 않는 명시 면제이고, 같은 목적은 추정기가 발행하는 innovation·NIS (`/ball_perception/debug/{innovation,nis}`) 를 오프라인으로 보아 얻는다.

**유령 트랙.** vision이 공을 놓치고 관성 예측만 계속 발행하면 스탬프는 신선하고 $J$ 와 $\bar\nu$ 는 오히려 작아진다. `generation`·`validity` 가 이 판별의 1차 수단이다. 측정 손실 뒤에는 `VALID` 예측이 더 나오지 않고 침묵한다(G1-5).


### 4.6 레이아웃 검증 `[확정 D-4, A-3]`

- **필드 이름으로 파싱한다.** `PointField` 배열에서 필수 필드를 이름으로 찾고, 각 필드의 datatype·count 가 기대와 같은지 검사한다(§5.1 표). offset 은 배열에서 읽는다 — 상수로 가정하지 않는다.
- **레이아웃 해시는 진단이다.** `(name, offset, datatype, count)` 목록 + `point_step` + `is_bigendian` 의 해시를 캐시하고, 해시가 바뀐 메시지에서만 필드 맵을 다시 만든다(평소 O(1)). 해시 변경은 진단으로 올린다. 재구성한 맵이 필수 필드 검사를 통과하면 수락한다 — offset 만 바뀐 레이아웃 변경은 그대로 따라간다.
- 해시·이름 검사는 **의미 변경**(단위·기준·순서)을 못 잡는다. 제어 PC 는 궤적의 내용을 물리로 검사하지 않는다 — 추정기의 궤적을 신뢰한다 (§7). $\bar\nu$ 추세 (§4.5) 는 구현하지 않았다.

## 5. C++ 구현

§5 는 인터페이스의 계약을 말하고, 시그니처 · 타입의 정의는 헤더가 갖는다. ros2_control·tf2 는 쓰지 않는다.

### 5.1 파서 (`integrated_bringup` 바인딩, nrt)

정의: `integrated_bringup/include/integrated_bringup/controllers/catching/traj_input.hpp` (`TrajFieldMap`, `BuildFieldMap`, `CloudReject`). 필수 필드 집합:

| 필드 | datatype | count |
|---|---|---|
| `x, y, z, vx, vy, vz, ax, ay, az` | FLOAT64 | 1 |
| `covariance` | FLOAT64 | 36 |
| `snapshot_sequence`, `generation` | UINT32 | 2 (low, high) |
| `horizon_ns` | UINT32 | 1 |
| `validity` | UINT8 | 1 |

이름으로 찾고 datatype·count·경계(offset + 크기 <= point_step)를 검사한다.

메시지 형식 검사 — **점을 복사하기 전에** 모두 통과해야 한다:

- `is_bigendian == false` (바이트 스왑 미지원)
- `height == 1`, `width` ∈ [`io.n_min`, `kCap`] (`n_max` 키는 없다 — 점 수 상한은 `kCap`, L2 §5.1) — 상한 검사를 복사 전에, **인덱싱 전에** 한다 (범위 밖 읽기 방지)
- **`width == 0` 은 거부가 아니라 "트랙 없음" 이다.** vision 은 예측이 없을 때 `height 1`·`width 0` 인 빈 cloud 를 제 주기로 발행한다 — 공이 없는 동안 계속. 이를 `no_track` (`CloudReject::kNoTrack`) 으로 따로 세고 (거부 histogram 의 마지막 bucket) 경고하지 않는다. 분류 자리는 byte order·`frame_id`·`height` 검사 **뒤**, `width` 범위 검사 **앞**: 다른 frame 이나 `height ≠ 1` 의 빈 메시지는 그 결함 그대로 거부하고, `0 < width < n_min` 은 계속 `shape` 다. "트랙 없음" 은 **빈 행**이다 — `width 0` 인데 `row_step ≠ 0` 이거나 `data` 가 비어 있지 않으면 헤더와 본문이 어긋난 메시지라 `size` 로 거부한다. 저장하는 것은 없다 — 수락 카운트·진단·sequence 기억·스냅숏은 마지막으로 **수락한** 예측의 것으로 남고, stale 판정은 그 예측의 수신 나이로 계속 흐른다
- `data.size() == point_step × width`, `row_step == point_step × width`
- 필드 값은 `std::memcpy` 로 읽는다(정렬·aliasing UB 방지)

### 5.2 수신 콜백 (nrt)

정의: `CatchingTrajInput::OnCloud` (`traj_input.hpp`). 순수 디코더다 — 수락(`CloudReject::kNone`)이면 호출자 (`controller.cpp`) 가 `snap`·`cov` 를 SeqLock 에 게시하고 계획기를 깨운다 (eventfd, `planner_thread.hpp`); 거부면 둘 다 건드리지 않고 사유를 센다. 컨트롤러 LifecycleNode 의 default group 구독 → `nrt_callback_executor` (G1-6). lifecycle 은 publisher 만 게이트하므로 이 구독은 컨트롤러가 비활성(inactive)인 동안에도 살아서 `OnCloud` 가 계속 불린다 (D-23). 할당은 configure 에서 끝낸 버퍼만 쓴다.

판정 순서 (앞의 거부가 뒤를 막는다):

1. 도착 즉시 steady·wall 시각을 한 쌍으로 찍는다 (D-2).
2. 레이아웃·형식 검사 (§4.6, §5.1 — 복사 전), 헤더 필드 읽기 (`generation`, `snapshot_sequence`, `validity`).
3. 순서 검사 (`CheckOrder`, §4.4 — 중복·역전·`NOT_EVALUATED`).
4. 미래 스탬프 검사: 원점 지연 $<-T_{future}$ 이면 거부 (§4.1). 원점 지연 분포는 진단만.
5. $t_{ref}$ 변환 1회 (D-2), 점 파싱 (NaN 거부, `frame_id`→모델 world 변환), 궤적 형식 검사 (`rtc::catching::Check`: 유한 · 단조 · 간격 하한 · 점 개수 — 실패는 `kMalformed`).
6. 지평 요구 진단 (`horizon_short`, D-15), provenance token 기록, 같은 generation 의 직전 예측과의 점프 $J$ 진단 (§4.4).

- `TrajectorySnapshot` 은 공용 궤적 타입(L2 §5.1)이며 trivially copyable POD 다(L0 §5.2). 점별 `BallTime`, `valid`, 그리고 provenance token `{activation_generation, generation, snapshot_sequence, traj_recv_ns}` (D-22) 을 싣는다 (`snap.token`).
- 계획기 쪽 공분산 버퍼 (`CovarianceSnapshot`, SeqLock) 도 같은 token 을 싣는다(A-3, D-22). 계획기는 계산 시작과 게시 직전에 최신 token 을 다시 조회해, 대체됐거나 짝이 안 맞는 조합(예: 궤적 N ↔ 공분산 N−1)을 버린다.
- **lifecycle 은 publisher 만 게이트한다** — 비활성 중에도 이 구독은 살아 있다(D-23). RT 소비자는 `snap.token.activation_generation` 이 현재 activation 과 다르면 그 스냅샷을 무효로 본다(§5.3).
- 공분산 대칭화 $\Sigma\leftarrow\tfrac12(\Sigma+\Sigma^\top)$ 는 NaN 이 없는 원소 쌍에만 적용한다.
- 문자열 비교(`frame_id`)와 해시 계산은 nrt 콜백이라 허용한다. 콜백 계산량은 lifecycle 서비스와 executor 를 공유하므로 작게 유지한다(D-7 이 계획 계산을 이 executor 에서 뺀 이유).

### 5.3 RT 측 읽기와 stale 판정

정의: `rtc_controllers/include/rtc_controllers/catching/traj_ingress.hpp` 의 `ReadTraj(snap, now, now_lead, t_stale_ns, current_activation, ConsumedToken&)` → `TrajView{stale, expired, is_new, age_ns}`. RTControllerInterface::Compute 안 — `noexcept`, 할당 없음.

- **D-21.** `SeqLock::Load()` 는 매 tick 무조건 한 번 하고, `SeqLock::sequence()` 와 짝지어 조건부로 복사하지 않는다 — 두 호출 사이에 writer 가 끼면 옛 payload 와 새 sequence 가 짝지어져 최신 스냅샷을 놓친다. 새 스냅샷 여부는 payload 안의 `snapshot_sequence` (D-22) 로만 판정한다.
- **`is_new` 의 판정 대상은 번호가 아니라 (epoch, 번호) 쌍 + `seen` 이다.** 새 track epoch 은 번호를 아무 값에서나 다시 시작할 수 있어 번호가 같다는 것만으로는 반복의 근거가 못 되고, 0 은 합법적인 `snapshot_sequence` 라 `last == 0` 하나로는 "아직 아무것도 안 봤다" 와 "0번을 봤다" 를 구별할 수 없다 (`ConsumedToken`).
- **stale** = 활성 세대 불일치 (D-23) ∨ 무효 ∨ steady 수신 나이 > `t_stale_ns`. 비양수 임계는 "제한 없음" 이 아니라 "쓸 수 없음" 으로 읽는다 (fail-closed). 수신 나이는 steady 시각 차다.
- **expired** = `now_lead` > 마지막 점의 `BallTime` (선행축, §4.1). stale 과 별개로 계산한다 — 신선한 스냅샷도 소진될 수 있고 감독자가 두 사유 (BALL_STALE, HORIZON_EXTRAP) 를 구분해 쓴다.
- **`age_ns` = −1 은 "수신 없음" 센티넬이다.** `traj_recv_ns` 가 첫 메시지 전에는 0 이라 `now − recv` 를 그대로 쓰면 steady 시계 uptime 이 나이로 읽힌다.
- **D-23.** 활성 세대가 현재와 다르면 비활성 중 받은 궤적이 재활성 첫 tick 에 그대로 쓰이는 것을 막기 위해 무효(stale)로 본다.
- `now` 는 매 tick steady 실측이다(tick × `dt` 아님, L0 §4.5).

### 5.4 RT 상태 POD (RT → 계획기)

계획기 입력용 RT 상태는 `rtc::SeqLock<rtc::catching::PlannerRtState>` (`demo_catching_controller.hpp` 의 `planner_rt_box_`, 정의는 `rtc_controllers/include/rtc_controllers/catching/planner_io.hpp`) 로 넘긴다. Eigen 멤버는 SeqLock 에 실을 수 없어 모두 `std::array<double, kMax…>` + 사용 차원이다.

- 필드: tick 의 활성 세대 · `rt_iteration` · `rt_state_ns` (D-22), 시험 리셋 epoch, 감독 모드 · arm 래치, 팔 명령 상태 `q_cmd`·`qd_cmd` (DEVICE 순서, `nv`), 대기 자세, L4 기준 상태 (closed_form 의 soft-catch 추종법이 만든 $x,\dot x,\gamma,\dot\gamma,\ddot\gamma$ 와 γ ramp), 추종 중인 계획 · 감속 구간의 id · 상태, 트랙 (`track_seen`, `track_generation`). 매 tick 새로 채운다 — 이번 tick 이 계산하지 않은 필드는 이전 값이 아니라 0/false 다 (PROC-7)
- 채우는 곳: `Compute` 안에서 `ControllerState` 로부터.
- **지문 센서 freshness (D-24 (a)).** 지문 wrench 의 수신 시각 · sequence · valid 는 `rtc_base` `DeviceState` 센서 lane 에 backend 3종이 채우는 값이며 (PROC-3), RT 감독이 `rtc::IsSensorGroupFresh` 로 읽는다 (`controller.cpp`, L7 §4.4). 지문 wrench 부호는 sim·실기 모두 finger-on-object 이고 S7.3 판정은 바이어스를 뺀 크기 ‖F − b‖ 만 써 부호에 의존하지 않는다 (G1-9)

## 6. YAML 파라미터

값 · 기본값 · 근거는 `integrated_bringup/config/<robot>/controllers/demo_catching_controller.yaml` 의 `io:` · `prediction:` 블록과 그 주석이 갖는다.

| 키 | 타입 | 단위 | 범위 | 뜻 |
|---|---|---|---|---|
| `io.traj_topic` | string | – | – | ball_perception debug 예측 궤적 토픽 (G1-2, D-4; stable ABI 아님) |
| `io.expected_frame` | string | – | – | vision frame (마스터 §3). 다르면 §4.3 변환 |
| `io.arm_base_frame` | string | – | 모델의 frame, root 에 강체 | 모델 world 변환의 팔 base frame (§1). 없으면 경고 후 vision frame 을 모델 world 로 취급. 모델에 없거나 움직이는 관절 뒤면 configure 거부 |
| `io.base_T_world` | map | deg, m | 유한 | $p_{base}=R_z(\text{yaw})\,p_{world}+t$. 실기는 카메라 보정. `arm_base_frame` 없이 주면 거부 |
| `io.n_min` | int | – | 2–`kCap` | 형식 검사 하한. **단일 키** — L2 검사도 이 값을 쓴다. 아래 유도 |
| `io.t_stale` | double | s | 0.02–0.2 | steady 수신 나이 임계 |
| `io.future_tol` | double | s | 1e-4–1e-2 | 원점 지연 음수 허용치 = 시계 동기 오차 예산 (§4.1, §4.2) |
| `io.horizon_min` | double | s | >0 | D-15 지평 요구 (§4.1) |
| `io.track.eval_offset` | double | s | 0–0.3 | §4.4 비교 시각 오프셋 |
| `io.track.j_warn` | double | m | >0 | §4.4 점프 경고. `TBD` 면 경고하지 않는다 (진단 전용이라 무장을 막지 않는다) |
| `sim.io.future_tol` | double | s | 1e-4–0.5 | sim 전용 overlay — sim config 에서만 활성이고 `io.future_tol` 을 덮는다 (`sim.ball.drag_k` 와 같은 활성 규칙). 부재는 실패가 아니다 — 엄격한 공용 키를 물려받는다 (fail-closed) |
| `prediction.dt_expected` | double | s | 1e-3–1.0 | vision profile 의 간격 |

- **`io.n_min` 의 유도.** $n_{min}=\lceil \texttt{io.horizon\_min}/\texttt{prediction.dt\_expected}\rceil+1$. **+1 은 여유가 아니다** — 간격 dt 로 놓인 n 점이 덮는 창은 $(n-1)\,dt$ 이므로 $\lceil h/dt\rceil$ 점은 요구 지평보다 짧고, 그러면 최소 길이 메시지마다 지평 경고가 뜬다. 검증기가 `horizon_min`·`dt_expected` 와의 정합을 검사한다. `io.n_max` 키는 없다 (상한은 `kCap`). 디코더의 간격 하한 (`TrajInputConfig::dt_min_ns`, 구조적 하한 1 ms — 실제 간격 gate 는 `n_min`) 은 `dt_expected` 와 별개의 상수다.
- **`io.t_stale`.** 발행 주기의 3 주기를 둔다 — 드롭 한 건은 stale 이 아니고, 유령 트랙 침묵 (마지막 VALID 뒤 한 건, 이후 침묵) 은 임계 안에 소실로 읽힌다.
- **`io.future_tol` 과 `sim.io.future_tol` 이 두 자릿수 다른 이유는 재는 대상이 다르기 때문이다.** sim 공 lane 의 stamp 는 발사 기준 sim 시간축을 wall 에 얹은 값이라 비행 중 위상 오차만큼 wall 을 앞서고 (D-3, `rtc_mujoco_sim` README §Projectile Ball stamp), 실기 카메라는 capture 시각이라 시계 동기 오차뿐이다 (실기 값은 PTP/NTP 실측 후 — TBD-NET-01). 한 키에 한 범위로는 둘 중 하나만 지킬 수 있고, sim 값을 실기 범위 안에 넣으면 하드웨어에서 큰 시계 오차를 조용히 수락한다.
- **`io.horizon_min`.** 값은 §4.1 의 식으로 정한다. $L$ 은 가정값이라 provisional 이다.

## 7. 단위 기술 구현 순서

구현한 것: 형식 · 레이아웃 검사 (§4.6, §5.1), 수신 콜백과 거부 사유 카운터 (§5.2), 원점 지연 · 수신 나이 · 점 수 · 지평 · 점프 진단 (nrt, 1 Hz, `PublishRole` 없이), RT 상태 POD (§5.4).

**물리 일관성 검사는 두지 않는다.** 궤적은 추정기가 항력을 고려해 낸다 — 점마다의 가속도는 상수 $g$ 가 아니라 profile 이 정하는 값이다 (L2 G2-3). 제어 PC 는 그 궤적을 신뢰하고 쓴다: 가속도의 크기, $v_j$ 와 위치의 정합, 공분산의 부호는 검사하지 않는다. 수신 경로가 보는 것은 `frame_id` 의 매 메시지 비교와 궤적 형식 (유한 · 시각 단조 · 간격 · 점 수) 이다. 그래서 레이아웃이 그대로인 **의미 변경** (단위 · 기준 · 순서, §4.6) 은 이 층에서 드러나지 않는다 — 맞추는 것은 추정기 쪽 규약의 몫이다. 공분산이 양의 준정부호가 아니거나 유한하지 않은 점도 거부되지 않는다 — 읽는 쪽이 처리한다 (segment MPC 의 포구 위치 가중 $W_p$ 는 음의 고유값을 0 으로 보고, 유한하지 않으면 상수 가중을 쓴다 — `mpc_multiframe_clik_formulation.md` §1.6 의 "포구 위치 가중").

## 8. 디버깅 방법

- 파싱이 통째로 실패: `ros2 topic echo --once --field fields` 로 실제 `PointField` 배열을 덤프해 필수 필드 표(§5.1)와 비교한다. 레이아웃 해시 변경 진단이 먼저 떴는지 본다.
- 값이 그럴듯하지만 틀림: endianness, `horizon_ns` 해석(상대 ns), uint64 low/high 순서, cov 순서 $(p,v)$ 를 의심한다. 한 점을 손으로 바이트 단위로 읽어 대조한다.
- 모든 메시지가 stale: stale 은 steady 수신 나이이므로 발행이 멈췄거나 거부되고 있는 것이다 — 거부 사유 카운터를 먼저 본다. `no_track` 만 오르고 있으면 lane 은 살아 있고 vision 에 트랙이 없는 것이다 (공이 시야에 없거나 추정기가 초기화 전).
- `kFutureStamp` (`StampStatus`·`CloudReject`) 발생: 실기에서 PTP offset, sim 에서 누군가 `use_sim_time=true` 로 떠서 stamp 가 sim time 인지 확인한다(G1-7).
- 순서 역전 거부가 계속됨: vision 노드 재시작으로 `snapshot_sequence` 가 되감긴 것인지 `generation` 과 함께 본다(§4.4).
- 트랙이 자주 새로 잡힘: vision 쪽 `generation` 변화 빈도를 직접 기록한다(제어 PC 가 만들지 않는다).
- 궤적이 계속 이상: `/sim/ball/ground_truth`(sim)와 수신 궤적을 같은 steady 축에 그린다.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G1-A | 거부 사유별 단위 테스트 (필수 필드 누락·datatype·count, shape, size, `width` > `kCap` (ASan 무오류), 미래 스탬프, `NOT_EVALUATED`, `snapshot_sequence` 중복·역전, 비단조 시각, 간격 하한 미만, NaN) 전부 통과. offset 만 바뀐 레이아웃은 수락 | `[SIM-ANY]` |
| G1-B | 실제 ball_perception 메시지 1건을 파싱해 점 개수·필드 값·uint64 필드가 `ros2 topic echo` 출력과 일치 | `[SIM-ANY]` |
| G1-C | writer 부하 + RT reader (`control_rate` 100·500·5000 Hz)에서 찢어진 스냅샷 0 (필드 체크섬 비교), TSAN 경고 0, 최악 재시도 시간 기록 (D-21, G1-8) | `[SIM-ANY]` |
| G1-D | RT 읽기 경로 할당 0 (`ScopedNoMalloc`·`ScopedAllocGate`), 최악 실행시간 기록 | `[SIM-ANY]` |
| G1-E | 인위적 지연·누락 주입 시 stale 전이가 steady 기준 기대 시각 ±1 틱 이내. $T_{arm}\ne0$ 에서 지평 소진은 now_lead 기준 | `[SIM-ANY]` |
| G1-F | 좌표 변환 왕복 테스트: $p,v,a,\Sigma$ 를 변환 후 역변환해 원값 복원 (< 1e-12), NaN 원소 보존 | `[SIM-ANY]` |
| G1-G | 실기 연결에서 수신 나이·원점 지연 분포(평균, 99%), 점프 분포 기록 → `t_stale`, `future_tol` 확정 | `[HW-P1B]` |
| G1-H | writer `Store` 를 reader step 사이에 주입하는 결정적 테스트에서 최신 스냅샷이 누락되지 않음을 보인다 (D-21) | `[SIM-ANY]` |
| G1-I | nrt 콜백을 막아 DDS backlog 를 만든 뒤 최신 `snapshot_sequence` 만 수락됨을 확인, 구독 QoS `KEEP_LAST(1)` (ARCH-6) | `[SIM-ANY]` |
| G1-J | 컨트롤러가 비활성인 동안 받은 궤적이 재활성 후 첫 tick 에 소비되지 않음 (`activation_generation`, D-23) | `[SIM-ANY]` |

## 10. 미확정 항목

- `io.track.j_warn` — 점프 분포를 측정하지 않았다 (진단 전용이라 무장을 막지 않는다)
- 실기의 `io.future_tol` · `io.t_stale` — 실기 연결의 수신 나이 · 원점 지연 분포(G1-G)로 확정한다 (TBD-NET-01)
- §4.5 (ν) — 구현하지 않았다
