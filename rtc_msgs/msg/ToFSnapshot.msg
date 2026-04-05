# ToF 측정값 + fingertip 자세를 하나의 메시지로 통합
# RT 컨트롤러에서 동일 제어 사이클의 데이터를 묶어 publish

builtin_interfaces/Time stamp

# ToF 센서 거리 [m], 순서: thumb_A, thumb_B, index_A, index_B, middle_A, middle_B
float64[6] distances

# ToF 센서 유효성
bool[6] valid

# Fingertip SE3 자세 (thumb, index, middle)
geometry_msgs/Pose[3] tip_poses
