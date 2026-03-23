# 단일 fingertip의 접촉/힘 추정값 (ONNX 3-head 모델 출력)
# Output layout: [contact_prob(1), F(3), u(3)]
# 대역폭 최적화를 위해 Header 제외

string     name                  # YAML에서 정의 (e.g. "index", "thumb")
bool       contact               # 접촉 여부 (contact_probability > 0.5)
float32    contact_probability   # sigmoid(contact_logit) → [0, 1]
float32[3] force                 # F: force vector [Fx, Fy, Fz] in N
float32[3] direction             # u: unit direction vector [ux, uy, uz] (비접촉 시 0)
