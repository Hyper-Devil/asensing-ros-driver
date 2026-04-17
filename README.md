# asensing-ros-driver

组合惯导 ROS 驱动（当前版本仅适配 INS5711DAA）。

## 支持范围

- 已适配设备：INS5711DAA
- 协议：BD DB 0B 帧头
- 帧长：优先解析 88 字节，兼容 63 字节帧
- 校验：仅使用整帧 XOR 校验

说明：`ins570d.launch` 仍在仓库中，但当前解析实现已按 5711 协议演进，不保证 570 可直接使用。

## 运行方式

```bash
apt-get install -y ros-noetic-serial
```

```bash
source /root/catkin_ws/devel/setup.bash
roslaunch asensing-ros-driver ins5711DAA.launch
```

关闭调试打印：

```bash
roslaunch asensing-ros-driver ins5711DAA.launch debug_display:=false
```

## 重要提醒

- 目前安装方式为正装，未通过SDK进行安装参数的写入。
- 如果吊装，仅pitch和yaw需要取反，其他数据都不需要。这个结论测试通过。

## 发布话题

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 姿态四元数、角速度、线加速度（FLU 坐标系） |
| `/imu/gps` | `sensor_msgs/NavSatFix` | GNSS 位置（差分时自动切换高精度） |
| `/imu/temperature` | `std_msgs/Float32` | 设备内部温度（℃，轮询 Type=22） |
| `/imu/satellites` | `std_msgs/UInt8` | 收星数（轮询 Type=32） |
| `/imu/ins_data` | `asensing_ros_driver/InsData` | 全量数据话题（见下节） |

`/imu/gps` 经纬度发布规则：

- 默认发布标准经纬度（协议基础字段，精度 1e-7 度）
- 当轮询 `Type=32` 的位置差分状态（Data1）为 `48/49/50` 时，发布高精度经纬度（精度 1e-8 度）

## 全量数据话题 `/imu/ins_data`

类型：`asensing_ros_driver/InsData`（自定义消息，见 `msg/InsData.msg`）

该话题在有订阅者时每帧发布一次，包含协议的全部解析字段。

**时间戳规则：`header.stamp` 固定为 `ros::Time::now()`（系统时间），不受 `use_gps_time` 参数影响。**

卫星时间通过独立字段提供，与 header 解耦：

| 字段 | 含义 |
|---|---|
| `gps_week` | GPS 周（整数） |
| `gps_tow` | GPS 周内时（秒，由协议偏移 52~55 换算） |
| `gps_time_valid` | `gps_week > 2400` 且 `gps_tow > 0` 时为 true |

**注意：姿态、角速度、线加速度、去重力加速度均为协议原始物理量，不经过 FLU 坐标变换。** 需要 FLU 坐标系数据请使用 `/imu/data`。

主要字段列表：

| 字段 | 单位 | 说明 |
|---|---|---|
| `roll / pitch / yaw` | rad | 协议原始欧拉角（无坐标变换） |
| `angular_velocity_x/y/z` | rad/s | 协议原始角速度 |
| `linear_acceleration_x/y/z` | m/s² | 含重力线加速度（协议原始） |
| `no_grav_acceleration_x/y/z` | m/s² | 去重力线加速度（仅 88 字节帧有效） |
| `latitude / longitude / altitude` | deg / m | 标准精度 GNSS 位置 |
| `hp_latitude / hp_longitude` | deg | 高精度 GNSS 位置（仅 88 字节帧有效） |
| `high_precision_valid` | bool | 当前帧是否为 88 字节帧 |
| `north/east/ground_velocity` | m/s | NED 速度 |
| `ins_status_raw` / `pos/vel/att/heading_initialized` | — | INS 初始化状态 |
| `satellite_count` | 个 | 收星数（Type=32 缓存） |
| `diff_pos_status / diff_heading_status` | enum | 差分定位/定向状态（Type=32 缓存） |
| `lat/lon/alt_std` | m | 定位精度标准差（Type=0 缓存） |
| `north/east/ground_vel_std` | m/s | 测速精度标准差（Type=1 缓存） |
| `roll/pitch/yaw_std` | deg | 姿态精度标准差（Type=2 缓存） |
| `temperature` / `temperature_valid` | ℃ | 温度（Type=22 缓存） |
| `wheel_speed_available` | bool | 轮速有无（Type=33 缓存） |

## 时间戳逻辑

驱动在解析每一帧串口数据后，先生成统一时间戳 `measurement_time`，用于 `/imu/data` 和 `/imu/gps` 的 `header.stamp`。

计算规则：

1. 默认使用系统时间：`ros::Time::now()`
2. 当 `use_gps_time=true` 且 `gpsWeek > 2400` 且 `gpsTimeSeconds > 0` 时，使用 GPS 时间

`/imu/ins_data` 的 `header.stamp` **始终**为 `ros::Time::now()`，卫星时间由 `gps_week`/`gps_tow` 字段单独提供。

## 参数说明（ins5711DAA.launch）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | 串口设备 |
| `buadrate` | int | `460800` | 串口波特率 |
| `device_model` | string | `ins5711daa` | 设备型号标识 |
| `frame_id` | string | `base_link` | 消息坐标系 |
| `gravity_acceleration` | double | `9.7883105` | 重力加速度（m/s²） |
| `use_gps_time` | bool | `true` | 是否优先使用 GPS 时间戳（仅影响 imu/data 和 imu/gps） |
| `debug_display` | bool | `true` | 是否输出逐帧调试信息 |
| `time_error_threshold` | double | `0.01` | GPS 时间跳变告警阈值（比例） |

## 调试输出内容（debug_display=true）

终端会输出逐帧调试信息，包括：

- Euler(deg)、Angular Rate(deg/s)、Acceleration(m/s²)
- GNSS Time(UTC)、LLA、High Prec LL
- NED Velocity(m/s)
- Satellites、Diff Status(Type=32)

差分状态仅在当前帧轮询类型为 `Type=32` 时更新，其余帧输出缓存值。

## 解析与鲁棒性说明

- 严格帧头匹配：`0xBD 0xDB 0x0B`
- 防越界策略：仅在缓冲区长度满足帧长后读取字段
- 88/63 字节双长度校验回退：避免设备输出模式差异导致无发布
- `ay`/`az` 取反改为 float 运算，消除 INT16_MIN 有符号溢出 UB
- 已清理未使用全局变量（`digit`、`oldtime`）和无效头文件

## 已知限制

- 当前代码路径以 INS5711DAA 为主，570D 未做测试
- 局部校验位（偏移 57/62/79）未启用，仅使用全局 XOR
- `std::string` 作为串口缓冲区，头部截断为 O(n)；高波特率场景可考虑换用环形缓冲区
