# asensing-ros-driver

组合惯导ros驱动

## 时间戳逻辑

驱动在解析每一帧串口数据后，会先生成统一时间戳 `measurement_time`，再将其写入本帧所有已发布消息：

- `sensor_msgs/Imu` (`/imu/data`)
- `sensor_msgs/Temperature` (`/imu/temperature`)
- `sensor_msgs/NavSatFix` (`/imu/gps`)

`measurement_time` 计算规则：

1. 默认使用系统时间：`ros::Time::now() + time_offset_in_seconds`
2. 当 `use_gps_time=true` 且 `gpsWeek > 0` 时，使用 GPS 时间：
	`convertGPSTimeToROSTime(gpsWeek, gpsTimeSeconds) + time_offset_in_seconds`

其中 `gpsTimeSeconds` 来自设备包内的 GPS 时间字段换算。

## measurement_time 的作用

- 保证同一帧 IMU/温度/GPS 的 `header.stamp` 一致
- 方便在“系统时间”和“GPS时间”之间切换，而不需要在每个发布点重复判断逻辑
- 提高下游时间同步与融合（如 EKF）的一致性

## 相关参数

- `device_model` (string, 例如 `ins570d` / `ins5711daa`)：设备型号选择，用于后续按型号区分位定义与解析逻辑
- `use_gps_time` (bool, 默认 `false`)：是否优先使用 GPS 时间戳
- `time_offset_in_seconds` (double, 默认 `0.0`)：对最终时间戳统一加偏移（秒）
