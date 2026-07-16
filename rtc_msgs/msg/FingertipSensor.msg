# 단일 fingertip 센서 데이터
# barometer/ToF 개수는 하드웨어 고정

string     name           # YAML에서 정의 (e.g. "index", "thumb")
float32[8] barometer      # 압력 센서 8개 (filtered, post-LPF)
float32[3] tof            # ToF 거리 센서 3개 (filtered, post-LPF)
float32[8] barometer_raw  # 압력 센서 8개 (raw, pre-LPF)
float32[3] tof_raw        # ToF 거리 센서 3개 (raw, pre-LPF)

# Inference output (force/displacement estimation)
float32[3] f              # 추정 힘 벡터 [Fx, Fy, Fz]
float32[3] u              # 추정 변위 벡터 [ux, uy, uz]
float32    contact_flag   # 접촉 판별 플래그 (0.0 or 1.0)
# 이 fingertip 의 f/u/contact_flag 가 소비 가능한가 — 발행자가 힘 값을 *어떻게*
# 얻었는지(ML 추론 vs firmware 실측)가 아니라 데이터 유효성만 뜻한다. false 면
# 소비자는 f/u 를 통째로 버린다 (CM sensor lane, controller 의 ft.valid).
bool       inference_enable
