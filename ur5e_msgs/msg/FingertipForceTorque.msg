# 단일 fingertip의 F/T 추정값 (ONNX 모델 출력)
# 대역폭 최적화를 위해 Header 제외

string     name           # YAML에서 정의 (e.g. "index", "thumb")
float32[3] force          # [Fx, Fy, Fz] in N
float32[3] torque         # [Tx, Ty, Tz] in Nm
