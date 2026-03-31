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

- `/imu/data` (`sensor_msgs/Imu`)
- `/imu/gps` (`sensor_msgs/NavSatFix`)
- `/imu/temperature` (`std_msgs/Float32`)
- `/imu/satellites` (`std_msgs/UInt8`)

`/imu/gps` 经纬度发布规则：

- 默认发布标准经纬度（协议基础字段）
- 当轮询 `Type=32` 的位置差分状态（Data1）为 `48/49/50` 时，发布高精度经纬度（`high_prec_lat/high_prec_lon`）

## 时间戳逻辑

驱动在解析每一帧串口数据后，先生成统一时间戳 `measurement_time`，并用于本帧发布消息的 `header.stamp`。

计算规则：

1. 默认使用系统时间：`ros::Time::now()`
2. 当 `use_gps_time=true` 且 `gpsWeek > 0` 且 `gpsTimeSeconds > 0` 时，使用 GPS 时间：
   `convertGPSTimeToROSTime(gpsWeek, gpsTimeSeconds)`

作用：

- 保证同一帧 IMU/GPS 时间一致
- 便于在系统时间和 GPS 时间之间切换
- 提高下游时间同步与融合（如 EKF）一致性

## 参数说明（ins5711DAA.launch）

- `port` (string, 默认 `/dev/ttyUSB0`)：串口设备
- `buadrate` (int, 默认 `460800`)：串口波特率
- `device_model` (string, 默认 `ins5711daa`)：设备型号标识
- `frame_id` (string, 默认 `base_link`)：IMU 消息坐标系
- `gravity_acceleration` (double, 默认 `9.7883105`)：重力加速度换算系数
- `use_gps_time` (bool, 默认 `true`)：是否优先使用 GPS 时间戳
- `debug_display` (bool, 默认 `true`)：是否输出逐帧调试信息

## 调试输出内容（debug_display=true）

终端会输出英文调试信息（浮点统一 4 位小数）：

- Euler(deg)
- Angular Rate(deg/s)
- Acceleration(m/s^2)
- GNSS Time(UTC)
- LLA(lat/lon/alt)
- High Prec LL(lat/lon)
- Satellites
- Diff Status(Type=32)

注意：差分状态释义只在当前帧轮询类型 `Type=32` 时更新并打印。

## 解析与鲁棒性说明

- 严格帧头匹配：`0xBD 0xDB 0x0B`
- 防越界策略：仅在缓冲区长度满足帧长后读取字段
- 88/63 双长度校验回退：避免因设备输出模式差异导致“有串口数据但无发布”

## 已知限制

- 当前代码路径以 INS5711DAA 为主，570D未做测试
- 局部校验位（57/62/79）未启用，仅使用全局 XOR
- 部分扩展字段仅解析用于调试，未额外发布独立话题

