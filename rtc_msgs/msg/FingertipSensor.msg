# 단일 fingertip 센서 데이터
# barometer/ToF 개수는 하드웨어 고정

string     name           # YAML에서 정의 (e.g. "index", "thumb")
float32[8] barometer      # 압력 센서 8개 (filtered, post-LPF)
float32[3] tof            # ToF 거리 센서 3개 (filtered, post-LPF)
float32[8] barometer_raw  # 압력 센서 8개 (raw, pre-LPF)
float32[3] tof_raw        # ToF 거리 센서 3개 (raw, pre-LPF)

# Inference output (force/displacement estimation)
#
# Force contract (#167) — 모든 fingertip-bearing backend 공통:
#   frame : fingertip *link* frame (URDF tip link). sensor→link extrinsic 은
#           발행자(ONNX 모델/sim 변환)에 baked-in — 소비자는 FK 회전 R_link(q) 만 적용한다.
#   units : N
#   sign  : fingertip 이 물체에 *가하는* 힘 (finger-on-object). 이 토픽을 발행하는
#           fingertip 센서 경로 (proto-1b firmware, 1a ONNX 추론) 는 전부 이 부호다.
#           "환경이 손가락에 가하는 힘" 이 필요한 소비자는 -1 을 곱한다.
#           주의 — rtc_mujoco_sim 의 contact-wrench lane 은 이 메시지가 아니라
#           geometry_msgs/WrenchStamped 로 나가며 부호가 **반대** (env-on-link,
#           ROS wrench 관례). mujoco_native backend 를 통해 같은 inference slot
#           1..3 으로 들어오므로, 그 경로의 소비자는 부호를 따로 맞춰야 한다.
float32[3] f              # 추정 힘 벡터 [Fx, Fy, Fz] — 위 contract 참조
float32[3] u              # 추정 변위 벡터 [ux, uy, uz]
float32    contact_flag   # 접촉 판별 플래그 (0.0 or 1.0)
# 이 fingertip 의 f/u/contact_flag 가 소비 가능한가 — 발행자가 힘 값을 *어떻게*
# 얻었는지(ML 추론 vs firmware 실측)가 아니라 데이터 유효성만 뜻한다. false 면
# 소비자는 f/u 를 통째로 버린다 (CM sensor lane, controller 의 ft.valid).
bool       inference_enable
