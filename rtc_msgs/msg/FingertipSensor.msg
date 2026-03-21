# 단일 fingertip 센서 데이터
# barometer/ToF 개수는 하드웨어 고정

string     name           # YAML에서 정의 (e.g. "index", "thumb")
float32[8] barometer      # 압력 센서 8개 (고정)
float32[3] tof            # ToF 거리 센서 3개 (고정)
