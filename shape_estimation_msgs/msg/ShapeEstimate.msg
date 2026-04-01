# 추정된 물체 형상 primitive 정보

builtin_interfaces/Time stamp

# 형상 타입
uint8 shape_type

uint8 SHAPE_UNKNOWN  = 0
uint8 SHAPE_PLANE    = 1
uint8 SHAPE_SPHERE   = 2
uint8 SHAPE_CYLINDER = 3
uint8 SHAPE_BOX      = 4

# 추정 신뢰도 [0, 1]
float64 confidence

# Primitive 중심 위치 (월드 프레임)
geometry_msgs/Point center

# 주축 방향 (cylinder 축 / plane 법선 / box 주축)
geometry_msgs/Vector3 axis

# 반지름 [m] (sphere, cylinder에 사용)
float64 radius

# Box 치수 [m] (width, height, depth)
geometry_msgs/Vector3 dimensions

# 피팅에 사용된 포인트 수
uint32 num_points_used

# 3개 손가락의 로컬 곡률 (실시간 값)
float64[3] local_curvatures
bool[3] curvature_valid
