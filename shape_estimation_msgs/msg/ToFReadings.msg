# RT 컨트롤러에서 500Hz로 publish하는 ToF 센서 원시 데이터
# 예비 메시지: 분리 토픽 사용 시 활용

builtin_interfaces/Time stamp

# 6개 센서 거리 [m], 순서: thumb_A, thumb_B, index_A, index_B, middle_A, middle_B
float64[6] distances

# 6개 센서 유효성 플래그
bool[6] valid
