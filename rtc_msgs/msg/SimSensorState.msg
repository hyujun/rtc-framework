# Aggregated MuJoCo sensor data for a device group (robot-agnostic).
# Published per physics step from rtc_mujoco_sim.

std_msgs/Header header
SimSensor[] sensors      # array of sensors configured for this group
