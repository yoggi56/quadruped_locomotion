Header header

uint8 mode
bool start
# modes:
# --- ikine check
float32[4] ef_vx
float32[4] ef_vy
float32[4] ef_vz
# --- body pose control
float32 body_vx
float32 body_vy
float32 body_vz
float32 body_wx
float32 body_wy
float32 body_wz
# --- robot walking mode
float32 robot_dx
float32 robot_dy
float32 robot_dz
float32 robot_step_freq
float32 robot_step_lenght
float32 robot_step_height
uint8 robot_gait_type
# --- servo check mode
float32[12] ref_angle_vel
# --- cute action mode
uint8 cute_action