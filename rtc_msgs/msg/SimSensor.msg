# Single MuJoCo sensor output (robot-agnostic).
# Populated from data_->sensordata[] after mj_step().

string name              # XML sensor name (e.g. "ft_sensor", "thumb_touch")
int32 sensor_type        # mjtSensor enum value (e.g. mjSENS_TOUCH=0, mjSENS_FORCE=3)
float64[] values         # sensor output — length equals sensor dim
