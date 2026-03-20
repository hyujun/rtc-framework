# 단일 fingertip의 접촉/힘 추정값 (ONNX 모델 출력)
# Output layout: [contact(1), F(3), u(3), Fn(3), Fx(1), Fy(1), Fz(1)]
# 대역폭 최적화를 위해 Header 제외

string     name           # YAML에서 정의 (e.g. "index", "thumb")
bool       contact        # 접촉 여부 (0/1)
float32[3] force          # F: force vector [Fx, Fy, Fz] in N
float32[3] direction      # u: unit direction vector [ux, uy, uz]
float32[3] normal_force   # Fn: normal force vector [Fnx, Fny, Fnz] in N
float32    force_x        # Fx scalar
float32    force_y        # Fy scalar
float32    force_z        # Fz scalar
