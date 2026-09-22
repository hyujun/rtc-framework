# L1 — IO: `PointCloud2` 예측 궤적 수신·파싱·검증, 스냅샷 브리지, 로봇·센서 상태

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: `PointCloud2` 구독·필드 이름 파서는 `integrated_bringup` 바인딩, ROS 비의존 판정 로직(D-2 시간 변환, 순서·`generation`·`validity`·stale 판정, 점 개수 검사)은 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`)
- 단계: **S1** 순수 조각 (S1.2 점 개수 검사, S1.3 시간 변환) + **S5.2** 구독 → 필드 이름 파서 → SeqLock 스냅샷
- 선행: 단계 W (완료, plan §2), L0, L2 (공용 궤적 타입, S1.2)
- 산출물: 필드 이름 파서, 수신 콜백, 궤적 스냅샷 브리지(`rtc::SeqLock`), 계획기 공분산 버퍼 전달, RT 상태 POD, 진단 카운터

---

## 1. 범위 / 비범위

범위: vision(ball_perception)의 `sensor_msgs/PointCloud2`(마스터 §5.1, D-4)를 검증해 공용 궤적 타입(L2 §5.1)으로 바꾸고 RT와 계획기에 전달한다. 손·지문 센서 상태를 계획기로 넘기는 RT 상태 POD 를 제공한다. 스냅샷 나이(stale), 시계 이상, 순서 역전·중복, 트랙 교체를 판정한다.

비범위:
- 좌표 변환 조회를 RT에서 수행하는 것(금지 — RT 경로에 tf2 없음). `frame_id`→`world` 가 다르면 configure 에서 한 번 읽어 캐시한 정적 변환을 nrt 콜백에서 적용한다. sim 에서는 불필요 — 실측 `frame_id` = `world` (TBD-VIS-06 닫힘, S3.4 2026-09-20); 실기 카메라 프로파일이 다른 frame 을 내면 그때 켠다.
- 궤적 예측·전파(vision 노드, 마스터 §5.2).
- RT 원시형 구현 — `rtc::SeqLock`·`rtc::SpscQueue` 를 쓴다(G1-8).

**v0.5 변경.** 입력 레이아웃이 ball_perception 의 실제 레이아웃으로 확정됐다(D-4, G1-1). 시간은 수신 시 1회 절대 steady 시각으로 변환한다(D-2, §4.1). 트랙 식별은 vision 의 `generation` 이 맡고(§4.4), `validity`·`snapshot_sequence` 로 미평가 예측과 순서 역전을 거부한다. 공분산은 RT 스냅샷에서 분리해 계획기 버퍼로만 보낸다(A-3).

## 2. 코드 확인 게이트

단계 W에서 처리했다(plan §2, `WORKSPACE_ANALYSIS.md` W2, W4, W5). 본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 기록 |
|---|---|---|
| G1-1 | `PointField` 배열 실제 레이아웃 | 닫힘 `[확정 D-4]` — little-endian, `point_step` 384: `x,y,z,vx,vy,vz,ax,ay,az` FLOAT64 (offset 0–64), `covariance` FLOAT64×36 @72 (row-major $p_x..v_z$, 모르면 NaN), `snapshot_sequence` UINT32×2 @360, `generation` UINT32×2 @368 (uint64 low/high), `horizon_ns` UINT32 @376, `validity` UINT8 @380 (0 NOT_EVALUATED, 1 VALID). **offset 은 참고값이고 파서는 이름으로 찾는다** (§4.6). debug 토픽이라 stable ABI 아님 (W5-2) |
| G1-2 | 토픽 이름과 QoS | 토픽 닫힘 — ball_perception `sim_estimator_node` 의 debug 예측 궤적 토픽 (`io.traj_topic`). QoS depth 는 `KEEP_LAST(1)` 고정(ARCH-6, invariants.md) — TBD 대상이 아니다. reliability 는 **`best_effort` 로 확정** (TBD-VIS-08 닫힘, S3.4 2026-09-20: best_effort KEEP_LAST(1) 구독이 reliable 구독과 identity 로 동일 — 30 Hz 에서 856/856, 50 ms 지연·30 % 드롭 주입에서도 편측 손실 0. publisher 는 RELIABLE/VOLATILE 이라 둘 다 호환). 선례: `integrated_bringup` inference 컨트롤러가 `KeepLast(1)` 로 구독한다 (W5-1) |
| G1-3 | 점 시각 필드 타입·기준 | 닫힘 — `t` 필드는 없다. `horizon_ns` (UINT32, `header.stamp` 기준 상대 ns), `header.stamp` = 예측 원점 시각 (W5-3) |
| G1-4 | `header.frame_id`와 `world`의 관계 | 닫힘 — **`world` 그대로** (TBD-VIS-06, S3.4 2026-09-20: 예측 토픽 `frame_id` = `world` ×856, 입력 카메라 lane 도 `world`, 프로파일 `input.frame_transform.source: identity`). 변환 없음 (W5-5) |
| G1-5 | 트랙 식별·소실 판정 수단 | 대부분 닫힘 — `generation` (트랙 epoch), `validity`. 유령 트랙은 **없다** (TBD-VIS-07 닫힘, S3.4 2026-09-20 드롭 주입: 공이 계속 날아도 입력이 끊기면 `VALID` 예측은 다음 30 Hz tick 한 건(≤34 ms)까지만 나오고 그 뒤 **침묵**). 단 INVALID 스냅샷은 발행되지 않는다 — 소실은 `track_status` (diagnostics, +100 ms COASTING·+500 ms LOST) 나 수신 나이로만 안다. 그래서 `io.t_stale` 이 필수다 (W5-7) |
| G1-6 | subscription callback이 도는 executor / callback group | 닫힘 — 컨트롤러 소유 구독은 LifecycleNode default group → `nrt_callback_executor` (단일 스레드, lifecycle 서비스와 공유) (W2-5) |
| G1-7 | RT tick 시각과 스탬프 clock 의 관계 (sim) | 닫힘 `[확정 D-2, D-3]` — RT tick 은 `RTControllerInterface::Compute(const ControllerState&)`, 시각은 steady. 스탬프는 wall (`rtc_mujoco_sim`·`sim_estimator_node` 모두 `use_sim_time=false`, `/clock` 없음). wall→steady 는 §4.1 변환 1회. D-3 은 S3.1a 검증 후 재검토 (W2-3) |
| G1-8 | SeqLock/SPSC 원시형 API와 재시도 정책 | 닫힘 — `rtc::SeqLock::Store`/`Load`/`sequence`. `Load` 는 **재시도 상한 없이** 일관된 사본을 얻을 때까지 반복한다(단일 writer·유한 쓰기 시간이 설계 불변식). 비-RT writer(nrt 콜백) → RT reader 는 backend 3종이 관절 상태에 이미 쓰는 경로이므로 새 primitive 가 아니다 — 최악 재시도 시간은 G1-C 로 측정한다(D-21). payload 는 trivially copyable (L0 §5.2) (W2-2) |
| G1-9 | 지문 센서·손 상태 경로와 규약 | 닫힘 — 실기 P1b `HandSensorState` 250 Hz, sim `WrenchStamped`. 두 경로 모두 finger-on-object 부호 (0fcc1d23 이후; `FingertipSensor.msg` 주석도 PR [#538](https://github.com/hyujun/rtc-framework/pull/538) 로 같은 부호로 고쳐졌고 이 브랜치에 병합됨, plan §7.3). 센서 lane 에는 수신 시각·sequence 가 없다 — freshness 경로는 **D-24 결정 대기**(S5 착수 전) (W4-6, TBD-HAND-03) |

## 3. 참고자료

[R8] 공분산 좌표 변환, [R12] PTP. `sensor_msgs/PointCloud2` 필드 규약은 ROS 2 인터페이스 정의(1차 출처: `sensor_msgs` 패키지).

## 4. 수학적 이론

### 4.1 시각 변환과 스냅샷 나이 `[확정 D-2]`

시간 규약의 SSoT 는 plan §3 이다. 수신 콜백(nrt)은 **도착 즉시** steady·wall 시각을 한 쌍으로 찍고, 원격 스탬프를 한 번만 절대 steady 시각으로 바꾼다.

$$t_{ref}^{steady}=t_{recv}^{steady}-\big(t_{recv}^{wall}-t_{stamp}\big),\qquad \mathrm{BallTime}_j=t_{ref}^{steady}+\texttt{horizon\_ns}_j$$

- 이후 RT·계획기는 `BallTime`(절대 steady ns)만 본다. 원점이 다른 상대시각을 섞지 않는다. 원격 stamp 를 시간 원점으로 쓰는 것은 D-2 가 명문화하는 E-1 예외다.
- **stale·나이는 steady 수신 나이** $t_{now}^{steady}-t_{recv}^{steady}$ 로만 판정한다(plan §3). `header.stamp` 로 staleness 를 판정하지 않는다([invariants.md](../../agent_docs/invariants.md)). v0.4 의 `age = clock.now − stamp` 거부(`TooOld`)는 이 규칙 위반이라 삭제했다.
- 원점 지연 $t_{recv}^{wall}-t_{stamp}$ 는 **진단**(분포 기록)이다. 음수가 $T_{future}$ 보다 크면(미래 스탬프) 변환 결과가 틀리므로 시계 이상으로 거부한다.
- stale 임계 $T_{stale}$: 발행 주기 + 여유(예시 profile ≤ 30 Hz). YAML, S3.4·S8 실측 후.
- 지평 끝 소진: 마지막 점 `BallTime` 을 **now_lead** 와 비교한다(plan §3 "궤적 지평 끝 경고 = now_lead"). 샘플링이 선행축으로 읽으므로 소진 판정도 같은 축이어야 한다 — v0.4 `readTraj` 는 실제 나이를 지평 상대시각과 비교해 두 축을 섞었다.
- **지평 요구 (D-15).** 수신 궤적의 지평(마지막 점 `horizon_ns`)이 제어기 요구 `io.horizon_min` 보다 짧으면 계획 후보에서 제외하고 진단한다. 요구값은 S3.6 이 정하고 sim profile 을 그에 맞춘다 — `io.horizon_min` 0.51 s (R1, §6), sim profile 권장 0.95 s / 0.05 s / 19 점 (기구학 reachable 창 기준, plan D-27·§4.4 S3.6 결과; 현 설정 0.8 s / 16 점은 계획기가 보는 창 끝이 0.78 s 라 늦게 잡는 후보가 빠진다. ball_perception 의 예시 profile 0.5 s / 최대 10 점은 그대로는 부족하다).

### 4.2 시계 오차의 영향

§4.1 변환은 전송 지연을 정확히 보상하고 두 PC 시계 오프셋 $\delta$ 만 남긴다. $\delta$ 는 보상 불가능한 위치 오차 $\approx\Vert v\Vert\,\delta$ 를 만든다(1차 근사). 6 m/s에서 1 ms는 6 mm다. $T_{future}$와 PTP offset 임계는 포구 허용오차 예산에서 역산해 정한다(L3 §4.6). sim 은 한 PC 의 wall clock 이라 $\delta=0$ 이다(D-3).

### 4.3 좌표 변환과 공분산

계획과 기준 생성은 `W`에서 수행한다. `header.frame_id`가 `W`가 아닌 경우만 변환한다(G1-4, S3.4 에서 필요 여부 확인). **네 가지를 모두 변환해야 한다.**

$$p'=Rp+t,\qquad v'=Rv,\qquad a'=Ra,\qquad \Sigma'=T\Sigma T^\top,\quad T=\mathrm{diag}(R,R)$$

속도·가속도에는 평행이동이 들어가지 않는다. 공분산은 6×6이므로 $T$가 $\mathrm{blkdiag}(R,R)$ 이다. 변환은 configure에서 캐시한 $R,t$ 로 nrt 콜백에서 수행하고, 공분산 변환 결과는 계획기 버퍼로만 간다(A-3). NaN(모름) 원소는 변환하지 않고 NaN 그대로 둔다 — 행·열에 NaN 이 섞인 상태로 곱하면 알려진 원소까지 오염된다.

frame이 움직이는 경우(예: 카메라가 로봇에 달린 경우)는 본 설계 범위 밖이다.

### 4.4 트랙 식별과 순서 `[확정 D-4]`

- **`generation` (uint64) 이 트랙 epoch 다.** 값이 바뀌면 새 트랙이다. 스냅샷에 그대로 싣고, L3·L7 이 이 값의 변화를 트랙 교체로 해석한다(L3 의 `n_settle` 등). v0.4 의 제어 PC 자체 `track_epoch` 생성(스탬프 간격·점프 임계)은 삭제한다.
- **`snapshot_sequence` (uint64)** 는 같은 generation 안에서 직전 수락값 이하이면 **중복·순서 역전으로 거부**한다(v0.4 M1 — 늦게 도착한 옛 예측이 새 예측을 덮어쓰던 문제). vision 노드 재시작으로 sequence 가 되감기는 경우의 처리(generation 변화 동반 여부)는 S5.2 에서 확인한다.
- **`validity`** 는 점 단위 필드다. `VALID`(1) 가 아닌 점(`NOT_EVALUATED` 또는 미지 값)이 하나라도 있으면 메시지를 거부한다. 부분 수용은 S5.2 에서 재검토한다.
- 두 uint64 필드는 UINT32×2 (low, high) 로 온다: $x=\mathrm{low}+2^{32}\cdot\mathrm{high}$. 모든 점이 같은 값을 가져야 하며 점 0 과 다르면 형식 오류로 거부한다.

**궤적 점프 (진단).** 같은 generation 의 직전 궤적과 새 궤적을 **같은 절대 시각** $t^\ast$ 에서 샘플링해 비교한다.

$$J=\big\Vert \hat p_{new}(t^\ast)-\hat p_{old}(t^\ast)\big\Vert,\qquad t^\ast=\max(t^{steady}_{ref,new},\,t^{steady}_{ref,old})+\Delta_{eval}$$

$J>J_{warn}$ 이면 경고만 기록한다(트랙 판정에는 쓰지 않는다). 이 값은 L3의 교체 히스테리시스(L3 §4.7)와 L4의 재예측 점프(L4 §4.3)가 감당해야 할 크기와 같은 양이므로 분포를 함께 본다.

### 4.5 예측 일관성 감시 `[논문 외 설계]`

연속 두 메시지를 같은 절대 시각에서 비교하면 혁신(innovation) 대용 통계를 만들 수 있다. §4.4의 $J$ 를 공분산으로 정규화한다.

$$\nu=J^\top\big(\Sigma_{pp,new}(t^\ast)+\Sigma_{pp,old}(t^\ast)+\lambda I\big)^{-1}J$$

- **정규화 역행렬 (NUM-1).** 합 공분산은 특이·악조건일 수 있으므로 $\lambda I$ 를 더한 뒤 Cholesky 로 푼다. 실패하거나 원소에 NaN(모름)이 있으면 그 쌍의 $\nu$ 는 계산하지 않는다(카운트만).
- **공분산의 시각 보간.** $t^\ast$ 는 일반적으로 두 점 사이다. 점마다 주어진 $\Sigma$ 를 $t^\ast$ 로 옮기는 규칙(인접 점 선택 / 원소별 선형 보간 등)이 **아직 정해지지 않았다 — S5.2 미결 항목.**
- 두 예측이 독립이고 공분산이 정직하면 $\nu\sim\chi^2_3$, $\mathbb E[\nu]=3$. 창 길이 $N$ 의 평균이 $\bar\nu\notin\big[\tfrac1N\chi^2_{3N}(\alpha/2),\ \tfrac1N\chi^2_{3N}(1-\alpha/2)\big]$ 이면 경고한다([R8], 경계는 configure 에서 미리 계산). 두 예측은 겹치는 관측을 쓰므로 독립이 아니다 — $\bar\nu$ 는 절대 임계보다 **추세**로 본다.
- 계산은 공분산이 아직 손에 있는 nrt 콜백에서 한다(RT 에는 공분산이 없다, A-3).

$\bar\nu$ 는 세 곳에서 쓴다: L3의 $\kappa_\sigma$ 보정 근거(L3 §4.4), L7의 `PRED_INCONSISTENT` 사유(L7 §4.2), L8 지표.

**유령 트랙.** vision이 공을 놓치고 관성 예측만 계속 발행하면 스탬프는 신선하고 $J$ 와 $\bar\nu$ 는 오히려 작아진다. v0.5 에서는 `generation`·`validity` 가 이 판별의 1차 수단이다. 측정 손실 뒤에도 `VALID` 예측이 계속 나오는지는 S3.4 드롭 주입으로 확인한다(G1-5).

### 4.6 레이아웃 검증 `[확정 D-4, A-3]`

- **필드 이름으로 파싱한다.** `PointField` 배열에서 필수 필드를 이름으로 찾고, 각 필드의 datatype·count 가 기대와 같은지 검사한다(§5.1 표). offset 은 배열에서 읽는다 — 상수로 가정하지 않는다.
- **레이아웃 해시는 진단이다.** `(name, offset, datatype, count)` 목록 + `point_step` + `is_bigendian` 의 해시를 캐시하고, 해시가 바뀐 메시지에서만 필드 맵을 다시 만든다(평소 O(1)). 해시 변경은 진단으로 올린다. 재구성한 맵이 필수 필드 검사를 통과하면 수락한다 — offset 만 바뀐 레이아웃 변경은 그대로 따라간다.
- 해시·이름 검사는 **의미 변경**(단위·기준·순서)을 못 잡는다. 그 역할은 물리 일관성 검사(§7 S5.2)와 $\bar\nu$ 추세가 한다.

## 5. C++ 구현

§5 의 스케치는 인터페이스 형태만 보인다. 실제 시그니처는 S5.2 에서 확정한다. ros2_control·tf2 는 쓰지 않는다.

### 5.1 파서 (`integrated_bringup` 바인딩, nrt)

필수 필드 집합:

| 필드 | datatype | count |
|---|---|---|
| `x, y, z, vx, vy, vz, ax, ay, az` | FLOAT64 | 1 |
| `covariance` | FLOAT64 | 36 |
| `snapshot_sequence`, `generation` | UINT32 | 2 (low, high) |
| `horizon_ns` | UINT32 | 1 |
| `validity` | UINT8 | 1 |

```cpp
// integrated_bringup 바인딩 — non-RT (nrt_callback_executor)
struct FieldSlot { std::uint32_t offset{0}; bool found{false}; };

struct FieldMap {                                  // 해시가 바뀔 때만 재구성
  std::array<FieldSlot, 9> pva;                    // x..az, FLOAT64 x1
  FieldSlot covariance, snapshot_sequence, generation, horizon_ns, validity;
  std::uint32_t point_step{0};
  std::uint64_t layout_hash{0};                    // 진단용
  bool ok{false};
};

// 이름으로 찾고 datatype·count·경계(offset + 크기 <= point_step)를 검사한다.
[[nodiscard]] bool BuildFieldMap(const sensor_msgs::msg::PointCloud2& m, FieldMap& out,
                                 ParseReject& why);
```

메시지 형식 검사 — **점을 복사하기 전에** 모두 통과해야 한다:

- `is_bigendian == false` (바이트 스왑 미지원)
- `height == 1`, `width` ∈ [`io.n_min`, `n_max`] (`n_max ≤ kCap`, S3.6) — 상한 검사를 복사 전에, **인덱싱 전에** 한다. v0.4 참조 구현은 `n > kMaxSamples` 에서 범위 밖 읽기가 있었다(ASan 확인, S1.2 에서 `kCap` 검사로 수정)
- `data.size() == point_step × width`, `row_step == point_step × width`
- 필드 값은 `std::memcpy` 로 읽는다(정렬·aliasing UB 방지)

### 5.2 수신 콜백 (nrt)

```cpp
// integrated_bringup 바인딩. 컨트롤러 LifecycleNode 의 default group 구독
// → nrt_callback_executor (G1-6). lifecycle 은 publisher 만 게이트하므로 이 구독은
// 컨트롤러가 비활성(inactive)인 동안에도 살아서 OnCloud 가 계속 불린다 (D-23).
// 할당은 configure 에서 끝낸 버퍼만 쓴다.
void CatchingTrajInput::OnCloud(const sensor_msgs::msg::PointCloud2& m) {
  const std::int64_t recv_steady = SteadyNowNs();  // 도착 즉시 한 쌍으로
  const std::int64_t recv_wall = WallNowNs();

  Reject r = CheckLayoutAndShape(m, fm_, cfg_);    // §4.6, §5.1 (복사 전)
  if (r == Reject::kNone) r = ReadHeaderFields(m, fm_, hdr_);  // generation, sequence, validity
  if (r == Reject::kNone) r = CheckOrder(hdr_, last_);          // §4.4 중복·역전·NOT_EVALUATED
  const std::int64_t origin_latency = recv_wall - StampNs(m.header.stamp);
  if (r == Reject::kNone && origin_latency < -cfg_.future_tol_ns) r = Reject::kFutureStamp;
  if (r != Reject::kNone) { counters_.Inc(r); return; }
  diag_.origin_latency.Push(origin_latency);       // 진단만 (§4.1)

  const BallTime t_ref{recv_steady - origin_latency};          // D-2 변환 1회
  if (!ParsePoints(m, fm_, t_ref, T_w_src_, need_transform_, snap_, cov_)) {  // NaN 거부
    counters_.Inc(Reject::kParse); return;
  }
  const auto chk = CheckTrajectory(snap_, cfg_.n_min, cfg_.dt_min);  // L2: 단조, dt_min 거부
  if (!chk.ok) { counters_.Inc(Reject::kMalformed); return; }
  snap_.horizon_short = chk.horizon_ns < cfg_.horizon_min_ns;        // D-15 진단
  snap_.recv_steady_ns = recv_steady;

  // provenance token (D-22) — 궤적 스냅샷과 계획기 공분산 버퍼가 같은 값을 싣는다
  snap_.activation_generation = ActivationGeneration();  // base RTControllerInterface (D-23)
  snap_.generation = hdr_.generation;                    // vision 트랙 epoch (§4.4)
  snap_.snapshot_sequence = hdr_.snapshot_sequence;
  snap_.traj_recv_ns = recv_steady;                      // == recv_steady_ns
  cov_.token = {snap_.activation_generation, snap_.generation, snap_.snapshot_sequence, snap_.traj_recv_ns};

  UpdateJumpAndNu(prev_, snap_, prev_cov_, cov_);  // §4.4, §4.5 (같은 generation 일 때만)
  traj_box_.Store(snap_);                          // rtc::SeqLock<TrajectorySnapshot> (공분산 없음)
  PublishToPlanner(snap_, cov_);                   // 계획기 버퍼 (A-3) — 같은 token, 전달 수단은 S5.2/S6
  planner_wakeup_.Notify();                        // eventfd (D-7c)
  last_ = hdr_;
}
```

- `TrajectorySnapshot` 은 공용 궤적 타입(L2 §5.1)이며 trivially copyable POD 다(L0 §5.2). `generation`, `snapshot_sequence`, `recv_steady_ns`, 점별 `BallTime` 과 provenance token `{activation_generation, generation, snapshot_sequence, traj_recv_ns}` (D-22, `traj_recv_ns` == `recv_steady_ns`) 을 싣는다.
- 계획기 쪽 공분산 버퍼 항목도 같은 token 을 싣는다(A-3, D-22). 계획기는 계산 시작과 게시 직전에 최신 token 을 다시 조회해, 대체됐거나 짝이 안 맞는 조합(예: 궤적 N ↔ 공분산 N−1)을 버린다.
- **lifecycle 은 publisher 만 게이트한다** — 비활성 중에도 이 구독은 살아 있다(D-23). RT 소비자는 `snap_.activation_generation` 이 `IsCurrentGeneration()` 이 아니면 그 스냅샷을 무효로 본다(§5.3).
- 공분산 대칭화 $\Sigma\leftarrow\tfrac12(\Sigma+\Sigma^\top)$ 는 NaN 이 없는 원소 쌍에만 적용한다.
- 문자열 비교(`frame_id`)와 해시 계산은 nrt 콜백이라 허용한다. 콜백 계산량은 lifecycle 서비스와 executor 를 공유하므로 작게 유지한다(D-7 이 계획 계산을 이 executor 에서 뺀 이유).

### 5.3 RT 측 읽기와 stale 판정

```cpp
// RTControllerInterface::Compute 안 — noexcept, 할당 없음
struct TrajView { bool stale; bool expired; bool is_new; };

[[nodiscard]] TrajView ReadTraj(const rtc::SeqLock<TrajectorySnapshot>& box, NowReal now,
                                NowLead now_lead, std::int64_t t_stale_ns,
                                std::uint32_t current_generation,  // 호출부: ActivationGeneration()
                                std::uint64_t& last_snapshot_seq, TrajectorySnapshot& buf) noexcept {
  buf = box.Load();                                // 매 tick 무조건 (D-21) — 재시도 상한 없음 (G1-8)
  const bool is_new = buf.snapshot_sequence != last_snapshot_seq;  // payload 안 token 으로 판정 (D-22)
  if (is_new) { last_snapshot_seq = buf.snapshot_sequence; }
  const bool current_gen = buf.activation_generation == current_generation;  // D-23
  const bool stale = !current_gen || !buf.valid || (now.ns - buf.recv_steady_ns) > t_stale_ns;  // steady 수신 나이
  const bool expired = buf.n > 0 && now_lead > buf.LastBallTime();              // 선행축 (§4.1)
  return {stale, expired, is_new};
}
```

- **D-21.** `Load()` 는 매 tick 무조건 한 번 하고, `SeqLock::sequence()` 와 짝지어 조건부로 복사하지 않는다 — 두 호출 사이에 writer 가 끼면 옛 payload 와 새 sequence 가 짝지어져 최신 스냅샷을 놓친다. 새 스냅샷 여부는 payload 안의 `snapshot_sequence` (D-22) 로만 판정한다.
- **D-23.** `buf.activation_generation` 이 현재 activation 과 다르면(`IsCurrentGeneration()` 이 아니면) 비활성 중 받은 궤적이 재활성 첫 tick 에 그대로 쓰이는 것을 막기 위해 무효(stale)로 본다.
- `now` 는 매 tick steady 실측이다(tick × `dt` 아님, plan §3).

### 5.4 RT 상태 POD (RT → 계획기)

계획기 입력용 로봇 상태는 `rtc::SeqLock<RtStatePod>` 로 넘긴다(plan §6). v0.4 의 Eigen 멤버 `RobotSnapshot` 은 SeqLock 에 실을 수 없어 대체한다.

- 필드: `NowReal` 시각, 팔 q·q̇ (측정), 직전 명령 q_c, 손 구동 좌표, 지문 wrench·스탬프 — 모두 `std::array<double, kMax…>` + 사용 차원 `n_arm`/`n_hand`/`n_tip`
- 채우는 곳: `Compute` 안에서 `ControllerState` 로부터. 지문 wrench 부호는 sim·실기 모두 finger-on-object (0fcc1d23), S7.3 에서 재확인 (G1-9)
- **지문 센서 freshness (D-24, S5 착수 전 결정 대기, S5.2e).** 옵션 (a) 가 채택되면 이 POD 에 지문 wrench 의 `recv_steady_ns`·`sequence`·`valid` 를 추가로 싣는다. 결정 전에는 필드를 확정하지 않는다

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `io.traj_topic` | string | – | ball_perception debug 예측 궤적 토픽 | – | G1-2, D-4 (stable ABI 아님) |
| `io.qos_reliability` | enum | – | `best_effort` | – | S3.4 실측으로 확정 (TBD-VIS-08 닫힘, G1-2). depth 는 `KEEP_LAST(1)` 고정(ARCH-6)이라 설정 키가 아니다 |
| `io.expected_frame` | string | – | `world` | – | 마스터 §3. sim 실측 `world` (TBD-VIS-06 닫힘). 다르면 §4.3 변환 |
| `io.n_min` | int | – | **11** (provisional, S3.6) | ≥2 | 형식 검사 하한. **단일 키** — L2 검사도 이 값을 쓴다 (plan S0.3). = ⌈`io.horizon_min` / `prediction.dt_expected`⌉ = ⌈0.51 / 0.05⌉ (plan §4.4 S3.6 결과) |
| `io.t_stale` | double | s | `TBD` — **[제안] 0.10** (S3.6) | 0.02–0.2 | steady 수신 나이 임계. 발행 주기 + 여유 (S3.4·S8 실측 후). 제안 근거: S3.4 실측 발행 30 Hz (33 ms), 드롭 30 % 주입에서 p95 15 Hz (67 ms) 이므로 3 주기 = 0.10 s 면 드롭 한 건은 stale 이 아니고 유령 트랙 침묵 (마지막 VALID 뒤 ≤ 34 ms 한 건, 이후 침묵) 은 0.10 s 안에 소실로 읽힌다. **확정은 S5.2** (S8 부하 실측 후) |
| `io.future_tol` | double | s | `TBD` — **[제안] 1e-3** (S3.6) | 1e-4–1e-2 | 원점 지연 음수 허용치 = 시계 동기 오차 예산 (§4.1, §4.2). 제안 근거: sim 은 같은 호스트 wall clock (S3.4: stamp = capture wall `now()`, stamp→수신 p50 32 ms 로 음수 없음) 이라 예산은 변환 반올림뿐. 실기는 카메라 PC 와의 동기 실측 (TBD-NET-01, S10) 후 — **확정은 S5.2** |
| `io.horizon_min` | double | s | **0.51** (provisional, S3.6) | >0 | D-15 지평 요구. S3.6 산출 — **R1 (commit 조건) 기준**, R2 (정지 출발) 로 잡지 않는다 (plan §4.4 S0 결과, 2026-09-19): $T_{freeze}+L$ = ($T_{close,tot}$ 0.2815 + $T_{arm}$ 0.05 + $T_{margin}$ 0.03) + L 0.14 = 0.5015 → 10 ms 로 **올림** 0.51 s (내림 0.50 은 R1 을 1.5 ms 미달하는 궤적을 통과시킨다; 0.05 s 간격에서 11 점 = 0.55 s). L 0.14 s 는 S0.7 가정값이라 provisional. sim profile 지평은 이 게이트가 아니라 목표 분포 요구 H_req 가 정한다 — 기구학 reachable 창 기준 0.93 s → 권장 0.95 s / 19 점 (plan D-27·§4.4 S3.6 결과) |
| `io.track.eval_offset` | double | s | 0.05 | 0–0.3 | §4.4 비교 시각 오프셋 |
| `io.track.j_warn` | double | m | `TBD` | >0 | §4.4 점프 경고 |
| `io.pred.nu_window` | int | – | 30 | 5–300 | §4.5 창 길이 |
| `io.pred.nu_alpha` | double | – | 0.05 | 0.001–0.2 | §4.5 |
| `io.pred.nu_reg` | double | m² | `TBD` | >0 | §4.5 정규화 λ (NUM-1), S5.2 |

v0.5 삭제: `io.max_age` (stamp 기반 나이 거부 — invariant 위반, §4.1), `io.track.t_gap`·`io.track.j_new` (트랙 판정은 `generation`, §4.4), `io.seqlock_max_retries` (`rtc::SeqLock` 에 재시도 상한 없음, G1-8), `io.tip_topic_prefix`·`io.tip_t_stale` (지문 센서는 `ControllerState` 경로, 규약은 S7.3).

## 7. 단위 기술 구현 순서

- **S1.2** (L2 와 공동) 공용 궤적 타입 + 점 개수 `[n_min, kCap]` (런타임 `n_max ≤ kCap` 은 S3.6) 검사(파서·check·RT 읽기 공통, 인덱싱 전), NaN 거부, provenance token 필드(D-22).
- **S1.3** D-2 변환 함수(**S0.6 승인 후**) + $T_{arm}\ne0$ fixture 에서 stale(실제축)·지평 소진(선행축) 판정 테스트.
- **S5.2a** `BuildFieldMap` (이름·datatype·count) + 형식 검사 + 레이아웃 해시 진단 + 거부 케이스 테스트 (필드 누락, datatype 변경, offset 만 변경 → 수락, width > `n_max` (and > `kCap`)).
- **S5.2b** `OnCloud`: `generation`/`snapshot_sequence`/`validity` 처리, uint64 low/high 조립, D-2 변환, provenance token(D-22, `ActivationGeneration()` 스탬프), 지평 요구 진단, SeqLock 게시, 계획기 버퍼, eventfd.
- **S5.2c** 물리 일관성 검사(nrt, 메시지당 $O(N)$): $a$ 가 상수 $g$ 인지(vision 규약, L2 G2-3), $v_j$ 대 Hermite 미분 잔차, `frame_id` 매 메시지 비교, cov 대각 양수(NaN 제외). 레이아웃 해시·이름 검사가 **의미 변경을 못 잡으므로**(§4.6) 이 검사가 그 역할을 한다.
- **S5.2d** $J$, $\nu$ (정규화 역행렬, 공분산 시각 보간 규칙 확정 후).
- **S5.2e** RT 상태 POD 채우기: sim 경로 → 실기 경로. D-24 채택 옵션의 지문 센서 freshness 필드 포함(§5.4).
- S3.4 는 QoS reliability·validity 히스토그램·`snapshot_sequence` 재시작 거동·유령 트랙을 **측정만** 한다 — 그 결과에 따른 정책(C-1 재검토 포함)은 S5.2 에서 정한다.
- 진단 발행: 거부 사유 카운트, 원점 지연·수신 나이 분포, 점 수·지평 분포, 점프 분포(nrt, 1 Hz, `PublishRole` 없이).

## 8. 디버깅 방법

- 파싱이 통째로 실패: `ros2 topic echo --once --field fields` 로 실제 `PointField` 배열을 덤프해 필수 필드 표(§5.1)와 비교한다. 레이아웃 해시 변경 진단이 먼저 떴는지 본다.
- 값이 그럴듯하지만 틀림: endianness, `horizon_ns` 해석(상대 ns), uint64 low/high 순서, cov 순서 $(p,v)$ 를 의심한다. 한 점을 손으로 바이트 단위로 읽어 대조한다.
- 모든 메시지가 stale: stale 은 steady 수신 나이이므로 발행이 멈췄거나 거부되고 있는 것이다 — 거부 사유 카운터를 먼저 본다.
- `FutureStamp` 발생: 실기에서 PTP offset, sim 에서 누군가 `use_sim_time=true` 로 떠서 stamp 가 sim time 인지 확인한다(G1-7).
- 순서 역전 거부가 계속됨: vision 노드 재시작으로 `snapshot_sequence` 가 되감긴 것인지 `generation` 과 함께 본다(§4.4).
- 트랙이 자주 새로 잡힘: vision 쪽 `generation` 변화 빈도를 직접 기록한다(제어 PC 가 만들지 않는다).
- 궤적이 계속 이상: `/sim/ball/ground_truth`(sim)와 수신 궤적을 같은 steady 축에 그린다.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G1-A | 거부 사유별 단위 테스트 (필수 필드 누락·datatype·count, shape, size, `width` > `n_max` (and > `kCap`) (ASan 무오류), 미래 스탬프, `NOT_EVALUATED`, `snapshot_sequence` 중복·역전, 비단조 시각, `dt_min` 미만, NaN) 전부 통과. offset 만 바뀐 레이아웃은 수락 | `[SIM-ANY]` |
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

~~TBD-VIS-06·07·08~~ (S3.4 에서 닫힘 — G1-2·G1-4·G1-5), `snapshot_sequence` 되감김 처리·`validity` 부분 수용 (S5.2), $\nu$ 의 공분산 시각 보간 규칙과 `io.pred.nu_reg` (S5.2), 계획기 공분산 버퍼 전달 수단 (S5.2/S6), `io.n_min`·`io.t_stale`·`io.future_tol`·`io.horizon_min` (S3.4·S3.6), `io.track.j_warn`, D-24 지문 센서 freshness 경로 (S5 착수 전).
