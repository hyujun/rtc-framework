# L1 — IO: `PointCloud2` 예측 궤적 수신·파싱·검증, 스냅샷 브리지, 로봇·센서 상태

- 브랜치: `feat/catching-L1-io`
- 패키지: `catching_io` (가칭, `TBD-WS-02`)
- 선행: 단계 W, L0
- 산출물: `cloud_parser.{hpp,cpp}`, `traj_receiver.{hpp,cpp}`, `snapshot_bridge.hpp`, `robot_snapshot.hpp`, `staleness.hpp`, 진단 카운터

---

## 1. 범위 / 비범위

범위: vision 노드의 `sensor_msgs/PointCloud2`(마스터 §5.1)를 검증해 `traj::PredictedTrajectory`로 바꾸고 RT에 전달한다. 계획 결과(`PlanSnapshot`)와 손·지문 센서 상태를 RT로 넘기는 브리지를 제공한다. 스냅샷 나이(stale), 시계 이상, 트랙 불연속을 판정한다.

비범위:
- 좌표 변환 조회를 RT에서 수행하는 것(금지). `frame_id`→`world` 변환은 configure 단계에서 한 번 읽어 캐시한다.
- 궤적 예측·전파(vision 노드, 마스터 §5.2).
- RT 원시형(SeqLock/SPSC) 구현 — workspace의 것을 쓴다(W2-2).

**v0.3 변경.** 입력이 자체 정의 `catching_msgs/BallState`에서 vision의 `PointCloud2`로 바뀌었다. 파싱 계층(§5.1)이 새로 생겼고, 메시지에 없는 정보(트랙 ID·상태, 추정기 건강도)를 대체하는 판정이 추가됐다(§4.4). NIS 창 감시는 입력이 사라져 삭제했다.

## 2. 코드 확인 게이트

단계 W에서 처리한다(`WORKSPACE_ANALYSIS.md` W2, W5). 본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 기록 |
|---|---|---|
| G1-1 | **`PointField` 배열 실제 덤프**: name·offset·datatype·count, `point_step`, `row_step`, `is_bigendian`, `is_dense` | TBD-VIS-02 (W5-2) |
| G1-2 | 토픽 이름과 QoS | TBD-VIS-01, 08 (W5-1) |
| G1-3 | `t` 필드 타입·기준 | TBD-VIS-03 (W5-3) |
| G1-4 | `header.frame_id`와 `world`의 관계, static TF 제공 방식 | TBD-VIS-06 (W5-5) |
| G1-5 | 트랙 식별·소실 판정 수단이 있는지 (메시지에는 자리가 없다) | TBD-VIS-07 (W5-7) |
| G1-6 | subscription callback이 도는 executor / callback group 규약 | TBD-RTC-04 (W2-5) |
| G1-7 | RT `update()`의 `time` clock type과 스탬프 clock의 일치 여부 (sim: `use_sim_time`) | TBD-RTC-05 (W2-3) |
| G1-8 | SeqLock/SPSC 원시형 API와 재시도 정책 | TBD-RTC-01 (W2-2) |
| G1-9 | 지문 센서·손 상태가 RT 루프에 들어오는 경로와 메시지 규약 | TBD-HAND-03 (W4-6) |

## 3. 참고자료

[R8] 공분산 좌표 변환, [R12] PTP. `sensor_msgs/PointCloud2` 필드 규약은 ROS 2 인터페이스 정의(1차 출처: `sensor_msgs` 패키지).

## 4. 수학적 이론

### 4.1 스냅샷 나이

수신 시점의 나이 $\Delta t=t-t_{ref}$ ($t_{ref}$ = `header.stamp`). vision이 예측 궤적을 주므로 나이가 커져도 **예측 자체는 유효**하다 — 다만 그 예측이 반영하지 못한 최근 관측이 쌓인다.

- stale 임계 $T_{stale}$: 발행 주기 + 전송 지연 + 여유. YAML로 둔다(G1-2 실측 후).
- 미래 스탬프 허용치 $T_{future}$: 시계 동기 오차 상한. $\Delta t<-T_{future}$이면 시계 이상으로 거부한다.
- 지평 여유: $t_{ref}+t_{N-1}$ 이 현재 시각보다 충분히 뒤여야 한다. 아니면 궤적이 이미 지나간 것이다(L2 §4.6).

v0.2의 $\Sigma_p(\Delta t)$ 증가 근사는 필요 없다. 각 샘플의 공분산을 vision이 직접 준다.

### 4.2 시계 오차의 영향

두 PC 시계 오차 $\delta$는 보상 불가능한 위치 오차 $\approx\Vert v\Vert\,\delta$를 만든다(1차 근사). 6 m/s에서 1 ms는 6 mm다. $T_{future}$와 PTP offset 임계는 포구 허용오차 예산에서 역산해 정한다(L3 §4.6).

### 4.3 좌표 변환과 공분산

계획과 기준 생성은 `W`에서 수행한다. `header.frame_id`가 `W`가 아닌 경우만 변환한다(G1-4). **네 가지를 모두 변환해야 한다.**

$$p'=Rp+t,\qquad v'=Rv,\qquad a'=Ra,\qquad \Sigma'=T\Sigma T^\top,\quad T=\mathrm{diag}(R,R)$$

속도·가속도에는 평행이동이 들어가지 않는다. 공분산은 6×6이므로 $T$가 $\mathrm{blkdiag}(R,R)$ 이다. 변환은 configure에서 캐시한 $R,t$ 로 non-RT 콜백에서 수행한다.

frame이 움직이는 경우(예: 카메라가 로봇에 달린 경우)는 본 설계 범위 밖이다. G1-4에서 static인지 확인한다.

### 4.4 트랙 연속성 판정 `[논문 외 설계]` `[TBD-VIS-07]`

`PointCloud2`에는 `track_id`도 트랙 상태(초기화/추적/소실)도 없다(마스터 §5.1). vision이 별도 수단을 주지 않으면 제어 PC가 다음 두 지표로 대체한다.

**스탬프 간격.** 연속 두 메시지의 `header.stamp` 차가 $T_{gap}$ 을 넘으면 새 트랙으로 본다(발행이 끊겼다 재개된 것).

**궤적 점프.** 직전 메시지와 새 메시지를 **같은 절대 시각** $t^\ast$ 에서 샘플링해 비교한다.

$$J=\big\Vert \hat p_{new}(t^\ast)-\hat p_{old}(t^\ast)\big\Vert,\qquad t^\ast=\max(t_{ref,new},\,t_{ref,old})+\Delta_{eval}$$

$J>J_{new}$ 이면 새 트랙(다른 공이거나 예측이 리셋된 것), $J_{jump,warn}<J\le J_{new}$ 이면 경고만 기록한다. 이 값은 L3의 교체 히스테리시스(L3 §4.7)와 L4의 재예측 점프(L4 §4.3)가 감당해야 할 크기와 같은 양이므로, 임계를 그쪽과 함께 정한다.

**한계 명시.** 이것은 대체 수단이지 트랙 ID가 아니다. 공이 두 개 동시에 날아오거나 vision이 트랙을 바꿔치기하면 판별하지 못한다. 단일 투척 시나리오를 전제한다(`TBD-BALL-02`). vision 쪽에 트랙 ID를 실을 여지가 있으면 그 편이 낫다 — W5-7에서 사용자와 결정한다.

### 4.5 예측 일관성 감시 `[논문 외 설계]`

v0.2는 vision이 준 NIS 로 추정기 건강도를 감시했다. `PointCloud2` 에는 그 자리가 없지만(마스터 §5.1), **연속 두 메시지를 같은 절대 시각에서 비교하면 혁신(innovation) 대용 통계를 만들 수 있다.** §4.4가 이미 계산하는 $J$ 를 공분산으로 정규화하면 된다.

$$\nu=J(t^\ast)^\top\big(\Sigma_{pp,new}(t^\ast)+\Sigma_{pp,old}(t^\ast)\big)^{-1}J(t^\ast),\qquad J=\hat p_{new}(t^\ast)-\hat p_{old}(t^\ast)$$

두 예측이 독립이고 공분산이 정직하면 $\nu\sim\chi^2_3$ 이므로 $\mathbb E[\nu]=3$ 이다. 창 길이 $N$ 의 평균이

$$\bar\nu\notin\Big[\tfrac1N\chi^2_{3N}(\alpha/2),\ \tfrac1N\chi^2_{3N}(1-\alpha/2)\Big]$$

이면 경고한다([R8], 구간 경계는 configure에서 미리 계산). 위쪽을 넘으면 vision 공분산이 과소, 아래쪽이면 과대다.

**한계를 명시한다.** 두 예측은 겹치는 관측을 쓰므로 독립이 아니다. 따라서 $\bar\nu$ 는 절대 임계로 쓰기보다 **추세**로 본다 — 평소 값을 기록해 두고 그보다 크게 벗어나면 경고한다. 그래도 아무 감시도 없는 것보다는 낫다.

$\bar\nu$ 는 세 곳에서 쓴다: L3의 $\kappa_\sigma$ 보정 근거(§L3 4.4), L7의 `PRED_INCONSISTENT` 사유(§L7 4.2), L8 지표.

**여전히 못 잡는 것: 유령 트랙.** vision이 공을 놓치고 관성 예측만 계속 발행하면 스탬프는 신선하고 $J$ 는 오히려 **작아진다** — 두 지표가 정확히 반대 방향으로 움직인다. $\bar\nu$ 도 마찬가지다. 이 경우를 판별하려면 vision 쪽에 트랙 상태가 필요하다(`TBD-VIS-07`, W5-7). 그래서 W5-7을 "사용자 결정"이 아니라 **vision 쪽 요청 사항**으로 올렸다.

### 4.6 레이아웃 검증

v0.2는 `model_version` 정수 하나로 계약 위반을 잡았다. `PointCloud2`에는 그런 필드가 없으므로 **필드 레이아웃 자체를 지문으로 쓴다**.

configure 단계에서 첫 메시지의 `(name, offset, datatype, count)` 목록과 `point_step`, `is_bigendian`을 읽어 파서를 구성하고, 그 조합의 해시를 저장한다. 이후 메시지는 해시만 비교한다(O(1)). 불일치하면 거부하고 진단에 올린다 — vision 쪽이 필드를 바꿨는데 제어 PC가 조용히 잘못 읽는 상황을 막는다.

## 5. C++ 구현

### 5.1 파서 (non-RT, configure에서 1회 구성)

**offset을 상수로 가정하지 않는다.** 마스터 §5.1에 적었듯 진술된 `point_step`(372 B)이 필드 합(360 + 4~8 B)과 맞지 않는다. 실제 배열에서 읽어 구성한다.

```cpp
// catching_io/cloud_parser.hpp
struct FieldMap {                       // configure 에서 1회 구성
  int off_x{-1}, off_v{-1}, off_a{-1};  // 각 3성분의 시작 offset (연속 가정은 검증한다)
  int off_t{-1}, off_cov{-1};
  bool t_is_float64{true};              // false = uint32 ns (TBD-VIS-03)
  std::uint32_t point_step{0};
  std::uint64_t layout_hash{0};
  bool ok{false};
};

// PointField 배열에서 FieldMap 을 만든다. 실패 사유를 남긴다.
[[nodiscard]] FieldMap buildFieldMap(const sensor_msgs::msg::PointCloud2& m,
                                     ParseReject& why);

// 메시지 → PredictedTrajectory. 좌표 변환(§4.3) 포함. non-RT.
[[nodiscard]] bool parseCloud(const sensor_msgs::msg::PointCloud2& m, const FieldMap& fm,
                              const Eigen::Isometry3d& T_w_src, bool need_transform,
                              traj::PredictedTrajectory& out, CovBuffer& cov_out);
```

구성 시 검사 항목:

- 필수 필드 존재: `x,y,z,vx,vy,vz,ax,ay,az,t,cov`
- `x,y,z`가 float64이고 offset이 8 B 간격으로 연속인가 (아니면 성분별 offset을 따로 보관)
- `cov`의 `count`가 36인가
- `point_step`이 마지막 필드 끝보다 크거나 같은가
- `is_bigendian == false` (아니면 거부. 바이트 스왑은 지원하지 않는다)
- `height == 1`, `width == N`, `data.size() == point_step * width`

### 5.2 수신 콜백 (non-RT)

```cpp
void TrajReceiver::onCloud(const sensor_msgs::msg::PointCloud2& m) {
  const Nanoseconds t_ref = rclcpp::Time(m.header.stamp, clock_type_).nanoseconds();
  const double age = 1e-9 * static_cast<double>(clock_->now().nanoseconds() - t_ref);

  Reject r = Reject::None;
  if (!fm_.ok)                                        r = Reject::NoLayout;
  else if (layoutHash(m) != fm_.layout_hash)          r = Reject::LayoutChanged;   // §4.6
  else if (m.height != 1 || m.width < cfg_.n_min)     r = Reject::Shape;
  else if (m.data.size() != std::size_t(m.point_step) * m.width) r = Reject::Size;
  else if (age < -cfg_.future_tol)                    r = Reject::FutureStamp;
  else if (age > cfg_.max_age)                        r = Reject::TooOld;
  if (r != Reject::None) { counters_.inc(r); return; }

  if (!parseCloud(m, fm_, T_w_src_, need_transform_, traj_, cov_)) {
    counters_.inc(Reject::Parse); return;
  }
  const auto chk = traj::check(traj_, cfg_.n_min);              // 단조 t, 유한값
  if (!chk.ok) { counters_.inc(Reject::Malformed); return; }
  if (!withinExpectedSpacing(chk, cfg_))  counters_.inc(Reject::SpacingWarn);  // 경고만

  jump_ = trackJump(prev_traj_, traj_, cfg_.eval_offset);       // §4.4
  nu_    = predConsistency(prev_traj_, prev_cov_, traj_, cov_, cfg_.eval_offset);  // §4.5
  if (jump_ > cfg_.j_new || (t_ref - prev_t_ref_) > cfg_.t_gap_ns) ++track_epoch_;
  nu_window_.push(nu_);                                          // 창 평균, non-RT
  traj_.seq = ++seq_;
  traj_.track_epoch = track_epoch_;      // **스냅샷에 실어야 RT·L3 가 읽는다**

  traj_box_.write(traj_);        // RT 용: (t, p, v, a) 만        (단일 writer)
  plan_buf_.write(traj_, cov_);  // non-RT 계획용: 공분산 포함    (§L2 4.5)
  planner_signal_.notify();
  prev_traj_ = traj_; prev_t_ref_ = t_ref;
}
```

- 문자열 비교(`frame_id`)와 해시 계산은 non-RT 콜백이라 허용한다.
- `track_epoch_`는 `PointCloud2`에 없는 `track_id`를 대신한다(§4.4). L3·L7이 이 값의 변화를 트랙 교체로 해석한다.
- 공분산 대칭화: 수신 직후 각 점에 대해 $\Sigma\leftarrow\tfrac12(\Sigma+\Sigma^\top)$.

### 5.3 RT 측 읽기와 stale 판정

```cpp
struct TrajView { const traj::PredictedTrajectory* tr; double age; bool stale; };

[[nodiscard]] TrajView readTraj(const SnapshotBox<traj::PredictedTrajectory>& box,
                                Nanoseconds now, double t_stale,
                                traj::PredictedTrajectory& buf) noexcept {
  const bool ok = box.tryRead(buf);                   // SeqLock, 재시도 상한 (G1-8)
  const double age = 1e-9 * static_cast<double>(now - buf.t_ref);
  const bool expired = (buf.n > 0) && (age > buf.s[buf.n - 1].t);   // 지평 소진
  const bool stale = !ok || !buf.valid || age > t_stale || expired;
  return {&buf, age, stale};
}
```

재시도 상한에 걸려 읽기에 실패하면 직전 버퍼를 유지하고 stale로 처리한다.

### 5.4 RobotSnapshot

```cpp
struct RobotSnapshot {                               // RT 틱마다 갱신
  Nanoseconds t{0};
  Eigen::Matrix<double, kMaxArmDof, 1> q, dq;        // 측정 (kMaxArmDof = 7, 사용 차원은 n_arm)
  Eigen::Matrix<double, kMaxArmDof, 1> q_cmd;        // 직전 명령
  std::array<Eigen::Matrix<double, 6, 1>, kMaxFingertips> tip_wrench;  // S_i 기준
  std::array<Nanoseconds, kMaxFingertips> tip_stamp; // 실기 비동기 경로의 나이 판정용
  Eigen::Matrix<double, kMaxHandDof, 1> q_hand;      // 구동 좌표
  int n_arm{0}, n_tip{0}, n_hand{0};
};
```

채우는 경로는 workspace의 것을 쓴다(W4-6). 시뮬레이션과 실기가 다른 경로면 어댑터를 둔다.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `io.traj_topic` | string | – | `TBD` | – | TBD-VIS-01 |
| `io.qos` | enum | – | `TBD` | – | TBD-VIS-08, vision 설정을 따름 |
| `io.expected_frame` | string | – | `world` | – | 마스터 §3. 다르면 §4.3 변환 |
| `io.n_min` | int | – | `TBD` | ≥2 | 형식 검사 하한. **단일 원천** — L2는 이 값을 인자로 받는다(L2 §6 `prediction.n_min`은 같은 키) (G1-1) |
| `io.max_age` | double | s | 0.15 | 0.02–0.5 | 수신 거부 상한 (튜닝) |
| `io.t_stale` | double | s | `TBD` | 0.02–0.2 | 발행 주기 + 전송 지연 + 여유 (실측 후) |
| `io.future_tol` | double | s | `TBD` | 1e-4–1e-2 | 시계 동기 오차 예산 (§4.2) |
| `io.track.t_gap` | double | s | `TBD` | >0 | §4.4 스탬프 간격 임계 |
| `io.track.eval_offset` | double | s | 0.05 | 0–0.3 | §4.4 비교 시각 오프셋 |
| `io.track.j_new` | double | m | `TBD` | >0 | §4.4 새 트랙 판정 |
| `io.track.j_warn` | double | m | `TBD` | >0 | §4.4 경고 |
| `io.pred.nu_window` | int | – | 30 | 5–300 | §4.5 창 길이 |
| `io.pred.nu_alpha` | double | – | 0.05 | 0.001–0.2 | §4.5 |
| `io.seqlock_max_retries` | int | – | 4 | 1–64 | G1-8 결과 반영 |
| `io.tip_topic_prefix` | string | – | `TBD` | – | TBD-HAND-03 (실기) |
| `io.tip_t_stale` | double | s | `TBD` | – | 센서 주기에 따름 |

v0.2의 `io.nis_window`, `io.nis_alpha`는 삭제했다(입력 없음).

## 7. 단위 기술 구현 순서

- **L1.1** 단계 W의 W5-2 결과로 `buildFieldMap` + 레이아웃 해시 + 구성 실패 케이스 테스트.
- **L1.2** `parseCloud` + 좌표 변환(§4.3) + 저장된 bag 1건으로 왕복 테스트.
- **L1.3** `SnapshotBox<T>` 래퍼: RTC SeqLock 위에 `write()/tryRead()` 제공 (G1-8).
- **L1.4** `TrajReceiver` + 거부 사유 카운터 + 트랙 연속성 판정(§4.4) + 예측 일관성 지표(§4.5). `track_epoch` 를 스냅샷에 싣는지 확인.
- **L1.4b** 물리 일관성 검사(non-RT, 메시지당 $O(N)$): $\Vert a_j-(v_{j+1}-v_j)/\Delta t\Vert$ 잔차, $v_j$ 대 Hermite 미분 잔차, $\Vert a\Vert$ 범위, `frame_id` 매 메시지 비교, cov 대각 양정. 레이아웃 해시는 **의미 변경을 못 잡으므로**(§4.6) 이 검사가 그 역할을 한다.
- **L1.5** stale·지평 소진 판정.
- **L1.6** `RobotSnapshot` 채우기: 시뮬레이션 경로 → 실기 경로.
- **L1.7** 진단 발행: 거부 사유 카운트, 나이 분포, 샘플 수 분포, 점프 분포(non-RT, 1 Hz).

## 8. 디버깅 방법

- 파싱이 통째로 실패: `ros2 topic echo --once --field fields` 로 실제 `PointField` 배열을 덤프해 `buildFieldMap`의 기대와 비교한다. `point_step`과 필드 끝의 차이(마스터 §5.1)를 먼저 본다.
- 값이 그럴듯하지만 틀림: endianness, `t` 필드 타입(초/ns), cov 순서 $(p,v)$ 를 의심한다. 한 점을 손으로 바이트 단위로 읽어 대조한다.
- 모든 메시지가 `TooOld`: 시뮬레이션에서 `use_sim_time`이 양쪽 모두 켜졌는지, 스탬프가 예측 기준 시각인지 확인한다(G1-7).
- `FutureStamp` 발생: 실기에서 PTP offset, 시뮬레이션에서 clock type 혼용을 확인한다.
- `LayoutChanged` 발생: vision 노드가 필드를 바꿨다. 파서를 재구성하기 전에 무엇이 바뀌었는지 기록한다.
- 트랙이 자주 새로 잡힘: `io.track.j_new`가 예측 갱신량보다 작은지 확인한다. L2 점프 분포와 함께 본다.
- 궤적이 계속 이상: `/ball/truth`(시뮬레이션)와 수신 궤적을 같은 그래프에 그린다.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G1-A | 거부 사유별 단위 테스트 (레이아웃 불일치, shape, size, 미래·과거 스탬프, 비단조 $t$, NaN, 비양정 cov 대각) 전부 통과 | `[SIM-ANY]` |
| G1-B | 실제 vision bag 1건을 파싱해 점 개수·필드 값이 `ros2 topic echo` 출력과 일치 | `[HW-P1B]` |
| G1-C | writer 부하 + RT reader 500 Hz에서 찢어진 스냅샷 0 (필드 체크섬 비교), TSAN 경고 0 | `[SIM-ANY]` |
| G1-D | RT 읽기 경로 할당 0, 최악 실행시간 기록 | `[SIM-ANY]` |
| G1-E | 인위적 지연·누락 주입 시 stale 전이가 기대 시각 ±1 틱 이내 | `[SIM-ANY]` |
| G1-F | 좌표 변환 왕복 테스트: $p,v,a,\Sigma$ 를 변환 후 역변환해 원값 복원 (< 1e-12) | `[SIM-ANY]` |
| G1-G | 실기 연결에서 나이 분포(평균, 99%), 점프 분포 기록 → `t_stale`, `j_new` 확정 | `[HW-P1B]` |

## 10. 미확정 항목

TBD-VIS-01~08, TBD-RTC-01, 04, 05, TBD-HAND-03, `io.n_min`, `io.t_stale`, `io.future_tol`, `io.track.*`, `io.pred.*` 임계.
