# p1b fingertip fixture — 파지 절차의 근거와 실측

[SKILL.md](../SKILL.md) §Gotchas 의 p1b · compliance 규칙이 가리키는 전문이다. 규칙 (무엇을 하고 하지 않는가) 은 SKILL.md 가 갖고, 여기는 그 규칙이 나온 순서 · 실측 · 기각된 가설이다. 새 fixture 를 세우거나 "왜 이 제약인가" 를 물을 때 읽는다 — 레시피를 따르기만 할 때는 필요 없다.

## 고정 디스크 (`fingertip_grasp`) 와 자유 디스크 (`fingertip_grasp_free`)

**p1b 는 다르다** — 작업대 씬 `scene_with_table.xml` + `object_pool` 로 작업대 위에 물체가 하나 올라와 있으므로 접촉 검증이 **가능하다**. **출하 씬은 이제 받침대 씬이라 (작업대 없음, 팔 base 가 0.7 m 위 — 2026-10-09) 이 레시피는 `model_path:=package://hand_description/robots/ur5e_p1b/mjcf/scene_with_table.xml` 을 줘야 한다** (`fingertip_grasp*` overlay 는 스스로 그 씬을 고른다). 다만 손을 그냥 내리는 것으로는 안 된다: palm-down 자세에서 손가락은 옆을 향하고 **`wrist_2_link` · `l_palm_link` 가 상판에 먼저 닿아** 팔이 `shoulder_lift ≈ -0.72` 에서 멈추며, 물체까지 손 아래를 받쳐 손끝을 띄운다. 실제로 손끝에 하중을 거는 가장 짧은 경로는 **손을 닫아 둔 채 `/sim/set_external_wrench` 로 물체를 위로 밀어올리는 것** (`body_name: pool_<obj>_object`, +z 60 N) — 실측으로 `l_ring_tip_contact` 에 39 N 이 걸렸다. **진짜 파지(2 접촉 + 유효 pull estimate)가 필요하면 그 방법으로는 안 되고**, 물체를 손끝 높이에 맞춰 놓고 **팔을 전혀 안 움직인 채** 출하 palm-down 자세에서 엄지+검지만 닫는다. 2026-09-12 에 그렇게 처음 성공했고, 다섯 번 실패하면서 나온 제약이 이것들이다 — 다음에 다시 세울 때 이 순서로 확인한다:
1. **손끝 링크의 접촉 반경은 약 10 mm 이지 브래킷 간격이 시사하는 22 mm 가 아니다.** 발행되는 `l_*_tip_bracket_actual` 프레임은 접촉면이 아니다. 눈대중 배치는 전부 빗나가거나 파묻힌다 — `mujoco.mj_geomDistance` 로 **모든** 로봇 충돌 geom 에 대해 탐색할 것.
2. **손 높이를 관통하는 기둥형 물체는 배치 가능한 (x,y) 가 없다.** 열린 손이 이미 그 부피를 차지해 palm·dip·mcp 링크에 먼저 박힌다. 손끝만 사는 z 밴드 (기본 자세에서 약 0.335~0.365 m) 만 차지하는 **디스크**여야 한다.
3. **tip 이 아닌 링크의 접촉은 contact 센서가 못 본다** — 따라서 grasp lane 에도 pull estimator 에도 안 잡힌다. 실측: 100 kg 물체가 수 mm 밀려나는 동안 손끝 4개 토픽이 전부 `0.000 N`. 성공한 파지에서도 접촉 링크 3개 중 1개(`l_index_pip_linkage_b`, 셋 중 가장 깊은 −2.9 mm)는 무센서였다.
4. **자유 물체는 밀려난다.** 0.5 kg 기둥은 엄지가 그대로 밀어내고 (2.63 N 한 스텝 뒤 0 N), 5 kg 도 한 run 에 2 cm 걸어간다. 100 kg 받침이면 상판 마찰(약 1 kN)이 손이 낼 수 있는 무엇보다 커서 **손을 재는 실험**이 된다.
5. **`force_saturation: 15.0` N 을 넘기면 그 손끝은 contact 집합에서 빠진다** — 더 세게 쥐면 접촉이 *줄어든다*. 정지 조건은 peak 이 아니라 밴드여야 한다.
6. **엄지와 손가락은 폐쇄율 스케줄이 다르다.** 엄지 손끝은 제 구간에서 y 로 약 10 cm 쓸고, 검지 손끝은 25 % 까지 거의 안 움직이다가 x 로 6 cm 를 간다. 하나의 스칼라로 둘을 몰면 엄지가 포화(14 N)할 때 검지는 아직 1 mm 모자란다 — **손가락마다 램프를 따로** 준다.

**이 배치는 이제 출하된다 — 다시 탐색하지 말 것.** `sim_overlay:=fingertip_grasp`
(`integrated_bringup/config/ur5e_p1b/sim_overlays/fingertip_grasp.yaml` + 물체
`robot_descriptions/objects/fingertip_disk/object.xml`) 가 위 제약을 전부 만족하는
(x,y,z) 와 형상을 들고 있다. 기하 근거는 물체 파일 주석, 결정은 `#504` AC 6.

```bash
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=fingertip_grasp enable_viewer:=false use_cpu_affinity:=false
```

**파지 절차의 순서는 자유가 아니다 (`#504`)** — compliance 를 쓸 것이면
**먼저 compliance 로 전환하고 그 다음에 손을 닫는다.** 활성화의
`ResetTargetInitialization()` 이 hand target 을 측정값에서 재시드하므로, 파지한 채로
전환하면 `cmd − actual` 간극(= 이 손에서는 곧 파지력)이 0 이 된다. 2026-09-12 실측
(`260912_1848`): joint 에서 엄지 2.7 N / 검지 4.1 N 로 5 초 안정 파지 → 전환 →
**네 손끝 전부 0.00 N, 6 초 내내 회복 없음**. 손은 위치 지령이라 겉보기엔 닫힌 채다.
반대로 먼저 전환하고 닫으면 (`260912_1845`) 같은 물체에서 pull estimate 가
`valid`, `contact_mask=0b011`(thumb+index), `|F̂|` 최대 12.4 N 까지 간다.

폐쇄 자체는 실측으로 **세 가지**를 더 요구한다:

7. **닫는 양은 힘으로 정한다** — 폐쇄각을 열린 값에서 목표까지 밀면
   `force_saturation: 15.0 N` 을 넘겨 그 손끝이 contact 집합에서 빠진다.
   2026-09-12 실측: 코멘트 `5644085864` 의 개루프 스케줄을 그대로 주면 검지가
   24~52 N 까지 가고 `contact_mask` 가 **런의 99.8 % 동안 0** 이었다. 손끝 힘을
   읽으며 작은 스텝(검지 폐쇄율 0.01 ≈ 0.004 rad)으로 접근해 **2~7 N 밴드에 들면 멈춘다**.
8. **같은 target 을 재전송하지 말 것** — `hand_new_target_pending_` 이 서면 궤적이
   **측정 위치에서 다시 시작**하므로 간극이 0 으로 떨어졌다가 램프로 복구된다. 즉
   유지용 재전송이 매번 파지를 잠깐 푼다. 컨트롤러가 자기 target slot 을 들고 있으므로
   재전송은 애초에 불필요하다 — 1 초마다 재전송하며 직후에 샘플하면 **CSV 는 멀쩡한데
   토픽은 0 N** 으로 읽힌다 (2026-09-12 에 한 번 이렇게 오진했다).
9. **compliance 가 렌치를 소비하면 팔이 물체에서 걸어 나간다** — `K_p^a = 0`
   hand-guiding 이라 정상 힘이 compliant frame 을 밀고, 디스크는 100 kg 받침에
   고정돼 따라오지 못한다. 실측 `260912_1845`: `alpha` 1.0, `|x̃|` **4.22 cm**
   (`disp_limited` 0), 팔 관절 0.093 rad 이동, 그리고 **0.37 s 만에 접촉 소실**.
   ⇒ **이 fixture 로는 "pull 이 valid 를 지속하는가" 를 판정할 수 없다.** 지속 관측에는
   손을 따라올 수 있는 물체(매달린 씬)가 필요하고, 그것이 `#177` crit#6 ① 의 선행조건과
   같은 산출물이다.

**⇒ 지속 관측용 두 번째 fixture 가 있다: `sim_overlay:=fingertip_grasp_free`**
(`robot_descriptions/objects/fingertip_disk_free/object.xml`). 같은 스탠드·같은 스폰·같은
손끝 밴드지만 디스크가 **자식 body 로 분리돼 6-DOF + `gravcomp="1"`** 이라 손을 따라온다.
쓰는 쪽 규칙이 셋 더 붙는다 — 전부 2026-09-12 실측이다:

10. **무중력 자유물체는 첫 접촉에 밀려나고 돌아오지 않는다.** 중력이 원래 물체를 앉혀 두는
    역할을 하므로, 그걸 끄면 착좌를 잃는다. 그래서 슬라이드에 **약한 복원 스프링**
    (150 N/m, near-critical damping) 을 넣었다. 스프링 없이 돌린 첫 런은 `fi=1.0`·엄지
    0.40 rad 까지 닫아도 네 손끝 **20 초 내내 0.00 N** 이었다.
11. **자유물체는 순차 폐쇄로 못 잡는다 — 손가락을 번갈아 진행시킨다.** 검지를 먼저 밴드까지
    밀면 엄지가 닿기 한참 전에 디스크를 밀어낸다 (고정 디스크에서는 안 보이는 실패다).
    매 스텝 **힘이 작은 쪽**을 한 칸 진행시키면 두 접촉이 같이 도착한다 — 실측 엄지 1.53 N /
    검지 1.50 N 으로 대칭 파지가 섰고 디스크는 제자리였다.
12. **자유물체는 파지력이 스스로 빠진다 — 유지하려면 grip 서보가 필요하다.** 디스크가
    스프링 평형으로 물러나면 손가락이 지령 자세에 도달해 접촉력이 사라진다 (1.5 N → 0.05 N,
    20 초). p1b 는 `grasp_controller_type: "none"` 이라 컨트롤러가 이 루프를 안 닫으므로
    **fixture 드라이버가 닫아야 한다** (1 초 주기로 밴드 밖이면 한 칸). 이때의 재전송은 11번의
    금지 대상이 아니다 — *새* target 이라 정당하고, 다시 시작되는 램프가 짧다.

이 fixture 로 **`#504` AC 2 가 닫혔다**: 지원 순서 + 위 세 규칙으로 pull 이 세션의 **39.6 %
(23960 tick, 최장 연속 12610 tick)** 동안 `valid`·`contact_mask=0b011` 을 유지했고
(`|F̂|` 평균 0.068 N, `Σf_n` 평균 2.63 N, `|x̃|` 최대 1.95 cm), 같은 fixture 의 반대 순서는
전환 직후 0.00 N 으로 떨어져 6 초간 회복하지 않았다.

**이 fixture 가 못 하는 것 — `#177` crit#6 ① 의 grip sweep.** 자유 디스크에서는 **정상상태 파지력이
복원 스프링에 묶인다**: 손가락을 12 단계 더 밀어도 정착 힘이 0.9 → 1.6 N 밖에 안 움직였다
(팔은 0.0004 rad 로 정지). `d\|F̂\|/dΣf_n` 회귀는 `Σf_n` 스프레드 ≥2 N 을 요구하므로
**이 상태로는 전제 게이트에서 거부된다.**

13. **원인은 대향 기하가 **아니다** — 2026-09-12 에 계측으로 갈랐다** (종전 기록이 이 자리에
    *"두 손끝이 충분히 마주보지 않는 것"* 이라고 적었고, 그건 **틀렸다**). `publish_debug`
    lane 으로 두 접촉점을 world 에서 직접 읽어 판정한다 — 접촉점 사이 현(chord)은 원판 지름
    38 mm 에 가까울수록 대향이다 (`2r sin(Δφ/2)`). 실측:
    - 출하 스폰에서 chord 36.9 mm ⇒ **Δφ 152°**, 접촉 중점이 원판 중심에서 **3.9 mm** 벗어나
      있었다. 스폰을 그 중점으로 3.9 mm 옮기면 chord **37.8~38.0 mm ⇒ Δφ 169~176°** 로 거의
      완전 대향이 된다 (open 자세 여유는 13.6 → 13.8 mm 로 무해).
    - **그런데 파지력은 그대로다** — 정착 Σ\|f\| 는 여전히 1.4~2.5 N 이고 원판은 오히려 더
      멀리(7.3 mm) 밀려났다. ⇒ **대향은 원인이 아니었다.**
    - 함께 기각된 것 둘: **엄지 대향 관절**(`thumb_cmc_aa`, 이전 스케줄이 전부 0 에 두던
      관절)은 range 가 `0..2.36` 인데 **+방향이 엄지를 디스크에서 멀어지게** 한다 —
      0.4 에서 이미 나빠지고 0.8 이상은 접촉 자체가 없다. **키 큰 원판**(top 0.365→0.375,
      접촉이 상단 rim 이라는 관찰에 대한 처방)도 무효였고 오히려 rung 9 에서 index 접촉을
      잃었다.
    - 남은 설명은 **자유물체 + 위치지령 2점 pinch 에 안정화가 없다**는 것이다. 중력이 보통
      그 역할을 하는데 `gravcomp="1"` 이 그것을 껐고, 대신 넣은 복원 스프링은 **그 자체가
      참하중**이 된다 — 같은 사다리에서 스프링 힘이 0 → **1.09 N** 로 자랐다 (접촉점 중점의
      이동 × 150 N/m). 즉 *"참하중 고정"* 전제는 팔이 멈춰 있어도 깨진다. **이건 이제
      추측이 아니라 측정값**이고, grip sweep 의 전제 게이트 (`#177` crit#6) 가 읽어야 할 양이다.
    - **capture 기하(홈/플랜지)로 가두는 길은 막혀 있다**: 손끝 접촉 높이(world z 0.3645)가
      열린 손이 비워 두는 유일한 void(0.335~0.365)의 **맨 위**라, 위쪽 플랜지를 놓을 자리가
      `mj_geomDistance` 기준 **1.2 mm** (flange r 22 mm) 밖에 없고 24 mm 부터는 이미 겹친다.
    - **"스프링을 빼고 damping 만 남긴다" 도 기각됐다** (같은 날 실측). damper 의 힘은
      속도에 비례하므로 정착하면 참하중이 **정확히 0** 이 되어 전제로는 이상적인데,
      그렇게 돌리면 **파지 자체가 성립하지 않는다** — 끝까지 닫아도 네 손끝 `0.00 N`
      (스프링 도입 전의 첫 런과 같은 실패가 그대로 재현된다). 복원력이 없으면 첫 접촉이
      디스크를 밀어내고 되돌리는 것이 없다.
    ⇒ **전제와 파지 가능성이 이 설계공간 안에서 상충한다**: 스프링이 있으면 *그것이
    참하중*이고, 없으면 *잡을 수가 없다*. **다음 결정은 튜닝이 아니라 fixture 설계**이고,
    *"물체를 어떻게 매다는가"* 는 사용자 소유 항목이다. 스프링을 키우는 것은
    여전히 답이 아니다 — 참하중이 파지력을 더 빨리 따라간다.
    **결정 (2026-10-17, `#861` CLOSE): sweep 용 fixture 는 만들지 않는다.** crit#6 ① 은
    unit test `integrated_bringup/test/test_pull_estimator_leakage_bound.cpp` 가 닫았고
    sim sweep 에 의존하는 작업이 없다. 재개 조건 (`|F̂|` 절대값이 제어 입력에 걸릴 때) 과
    재개 시 출발점 (볼록-볼록 핀치의 측방 불안정 가설 · 평판 물체의 정적 간섭 실측) 은
    `#861` 코멘트가 갖는다 — 여기 복제하지 않는다.

그리고 접촉을 **어떻게 판정하지 않을지**도 위와 같은 무게로 정해져 있다. **joint_states 를 kinematic FK 로 재현해 접촉을 추정하지 말 것**: 이 손은 폐쇄 체인이라 수동 linkage 관절이 qpos 에 따로 있고, 그것을 0 으로 둔 재구성은 손끝이 상판을 1 cm 파고든 것처럼 보이는 **허상**을 만든다. 그리고 **팔이 어디까지 내려가는지는 아직 안 갈렸다**: `scene_with_table.xml` 의 배너 주석은 palm 을 `z = 0.049` 까지 내릴 수 있다고 적는데(자유공간 유도) 위 `shoulder_lift ≈ -0.72` 는 상판·물체에 막힌 실측이고, 둘을 맞춰 본 적이 없다. 컨트롤러나 정책이 물체를 못 잡을 때 **원인이 그쪽인지 씬인지 가르려면 이 실측이 선행**한다.

## compliance 외부 렌치의 sim 도달 가능성

**compliance 외부 렌치의 sim 도달 가능성은 프로필마다 다르고, 2026-09-11 에 바뀌었다.** `FromPullEstimate` 는 `vtcp.valid` 없이 publish 하지 않는데, 출하 `virtual_tcp_mode` 가 세 프로필 모두 `"constant"` 가 되면서 (`1b29c1d5`; iiwa7_leap 은 종전 `"disabled"`, p1a·p1b 는 `"centroid"`) 그 게이트는 **이제 어디서나 열린다** — `kConstant` 는 손끝 참여와 무관하게 offset 이 유한하면 `valid` 다 (`integrated_bringup/include/integrated_bringup/support/virtual_tcp.hpp`). 남는 축은 **지문력 lane 이 있느냐**다: `ur5e_p1b`·`iiwa7_leap` 은 sim 에서 손끝 contact wrench 를 받고 (`sim.yaml` 의 `devices.<hand>.backend.fingertip_wrench_topics`, 부호는 실기와 같은 finger-on-object), **`ur5e_p1a` 는 그 lane 도 잡을 물체도 없다**. ⇒ 종전의 "iiwa7_leap sim 에서 한 샘플도 안 나온다" 는 **더 이상 근거가 아니다**. **2026-09-12 실측으로 닿는 것을 확인했다** (`ur5e_p1b`, 위 파지 절차): pull estimate `valid=1` · `contact_mask=0b011`(thumb+index) · `|F̂|` 6.8 N · `leakage_bound` 0.57 N · `basis_source=REFERENCE`, 그리고 compliance 쪽 `compliance_diag.csv` 가 `wrench_valid=1`, `|f|` 최대 **7.96 N**, `bias_calibrated=1`(D-A5 100 샘플 완주 — 2026-09-04 실기에서는 22 tick 뿐이라 끝내 false 였다), **compliance §10.7 램프 α 0→1 이 0.688 s**, `|x̃|` 최대 **1.46 cm** (15 cm envelope 의 10 %, `disp_limited`·`vel_limited` 0). ⇒ **compliance §7.4 envelope·bias 표류·램프는 이제 sim 에서 관측 가능하다.** 단 **파지가 전환을 못 넘긴다** (`#504`): joint→compliance 스위치 직후 sum|f| 18.5 N → 0 N, 두 손끝이 `contact_on_threshold` 0.5 N 아래로 떨어지기까지 **238 ms**, compliance 가 본 유효 pull tick 은 **213개 / 0.494 s** 뿐이다. **`#504` 의 답은 코드가 아니라 순서다 (방향 (a))** — 위 p1b 항목의 파지 절차가 그 순서를 갖는다. 그리고 **정상상태 envelope 소진(α>0 지속)은 이 fixture 로는 못 본다**: 순서를 지켜 파지해도 렌치가 유효해지는 순간 팔이 고정된 물체에서 걸어 나가 0.37 s 만에 접촉이 끊긴다 (2026-09-12 실측, 위 9번). 이 sim 은 기전 재현과 회귀 red 까지이고 정상상태 hand-guiding 이 아니다. 한편 프레임 전이(vTCP ⇄ tool0) 경로는 여전히 출하 설정으로 도달 불가인데, 이유가 바뀌었다: `"constant"` 는 매 tick `is_vtcp=true` 로 고정이라 kind 가 flip 하지 않는다. 그걸 겨냥한 테스트는 그대로 `set_gains()` 로 모드를 명시적으로 바꾸고 그 전제를 단언해야 한다 (`test_compliance_admittance_coupling` 의 kind-change 테스트가 그 형태).
