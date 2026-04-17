# 导远 INS5711DAA 全量数据话题
# header.stamp = ros::Time::now()（系统时间，非卫星时间）
std_msgs/Header header

# ---- 卫星时间 (GPS Time) ----
uint32 gps_week              # GPS 周（offset 58-61）
float64 gps_tow              # GPS 周内秒（offset 52-55，单位 s）
bool gps_time_valid          # week>2400 且 tow>0 时为 true

# ---- 姿态：欧拉角（FLU 坐标系，弧度）----
float64 roll
float64 pitch
float64 yaw

# ---- 角速度（FLU，rad/s）----
float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z

# ---- 含重力线加速度（FLU，m/s²）----
float64 linear_acceleration_x
float64 linear_acceleration_y
float64 linear_acceleration_z

# ---- 去重力线加速度（FLU，m/s²，仅 88 字节帧有效）----
float64 no_grav_acceleration_x
float64 no_grav_acceleration_y
float64 no_grav_acceleration_z

# ---- 标准精度 GNSS 位置（offset 21-32）----
float64 latitude             # 度，精度 1e-7
float64 longitude
float64 altitude             # 米

# ---- 高精度 GNSS 位置（offset 63-78，1e-8 度，毫米级，仅 88 字节帧有效）----
float64 hp_latitude
float64 hp_longitude
bool high_precision_valid    # true = 当前帧为 88 字节帧

# ---- NED 速度（m/s，offset 33-38）----
float32 north_velocity
float32 east_velocity
float32 ground_velocity      # 正值 = 向下（地速）

# ---- INS 初始化状态（offset 39）----
uint8 ins_status_raw
bool pos_initialized
bool vel_initialized
bool att_initialized
bool heading_initialized

# ---- 卫星与差分状态（Type=32，轮询缓存）----
uint8 satellite_count
int32 diff_pos_status        # 差分定位状态，枚举值见协议第 4 节
int32 diff_heading_status    # 差分定向状态

# ---- 定位精度标准差（Type=0，轮询缓存，单位 m）----
float32 lat_std
float32 lon_std
float32 alt_std

# ---- 测速精度标准差（Type=1，轮询缓存，m/s）----
float32 north_vel_std
float32 east_vel_std
float32 ground_vel_std

# ---- 姿态精度标准差（Type=2，轮询缓存，度）----
float32 roll_std
float32 pitch_std
float32 yaw_std

# ---- 温度（Type=22，轮询缓存，℃）----
float32 temperature
bool temperature_valid

# ---- 轮速状态（Type=33，轮询缓存）----
bool wheel_speed_available
