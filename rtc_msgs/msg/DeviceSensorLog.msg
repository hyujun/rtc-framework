# Device별 센서 로그 (raw + filtered sensor data + inference output)
#
# sensor_names: fingertip 이름 (e.g. ["thumb", "index", "middle", "ring"])
# sensor_data: per-fingertip barometer + ToF 값

std_msgs/Header header

string[]  sensor_names       # fingertip 이름
int32[]   sensor_data_raw    # [M] raw sensor values
int32[]   sensor_data        # [M] filtered sensor values

bool      inference_valid
float32[] inference_output   # [K] inference 결과 (F/u/contact per fingertip)
