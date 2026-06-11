# 로봇 암 조인트 커맨드 (position / torque / pd_feedforward)
#
# joint_names와 values는 1:1 대응.
# joint_names가 비어있으면 기본 조인트 순서로 fallback.
#
# command_type == "pd_feedforward": PD position-servo backbone(values) + per-joint
# feedforward torque(feedforward[]) 오버레이. rtc::CommandType::kPdFeedforward 로
# 컨트롤러 스택이 발행 가능(C-3) — 단 mujoco_sim 만 feedforward 를 qfrc_applied 로
# 주입하고, 실 하드웨어(udp_hand_driver 등 position-only)는 feedforward 를 무시한다.

std_msgs/Header header

string[]  joint_names     # YAML에서 정의 (e.g. ["shoulder_pan_joint", "elbow_joint"])
float64[] values          # position(rad) | torque(Nm) | pd_feedforward: PD position target(rad)

string command_type       # "position" | "torque" | "pd_feedforward"

# ── pd_feedforward 전용 (다른 모드에선 무시) ──────────────────────────────────
# feedforward: joint generalized torque (Nm), gravity 포함 — sim 은 body_gravcomp=0
#   으로 두므로 컨트롤러가 중력보상까지 전부 제공해야 한다. 비면 0.
# kp/kd: PD gain (둘 다 >=0). sticky — 비면 현재 게인 유지, 채우면 갱신 후 지속.
#   all-or-nothing: kp/kd 중 하나만 채우거나 그룹 조인트 전체를 덮지 못하면 무시된다.
float64[] feedforward
float64[] kp
float64[] kd
