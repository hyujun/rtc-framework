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
bool       inference_enable  # true when calibration done for this fingertip
