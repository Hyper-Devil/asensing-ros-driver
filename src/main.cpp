#include <fstream>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <serial/serial.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <string>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>

bool zero_orientation_set = false;

ros::Time oldtime;

int digit = 1;

std::string type32DiffStatusToString(int status)
{
  switch (status)
  {
  case 0:
    return "NONE";
  case 1:
    return "FIXEDPOS";
  case 2:
    return "FIXEDHEIGHT";
  case 8:
    return "DOPPLER_VELOCITY";
  case 16:
    return "SINGLE";
  case 17:
    return "PSRDIFF";
  case 18:
    return "SBAS";
  case 32:
    return "L1_FLOAT";
  case 33:
    return "IONOFREE_FLOAT";
  case 34:
    return "NARROW_FLOAT";
  case 48:
    return "L1_INT";
  case 49:
    return "WIDE_INT";
  case 50:
    return "NARROW_INT";
  default:
    return "UNKNOWN(" + std::to_string(status) + ")";
  }
}

std::string formatRosTimeUTC(const ros::Time &t)
{
  const time_t sec = static_cast<time_t>(t.sec);
  std::tm tm_utc;
  gmtime_r(&sec, &tm_utc);

  std::ostringstream oss;
  oss << std::put_time(&tm_utc, "%Y-%m-%d %H:%M:%S") << "."
      << std::setw(3) << std::setfill('0') << (t.nsec / 1000000) << " UTC";
  return oss.str();
}

bool set_zero_orientation(std_srvs::Empty::Request &,
                          std_srvs::Empty::Response &)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

ros::Time convertGPSTimeToROSTime(int gpsWeek, double gpsSec)
{
  // GPS纪元开始的时间（1980年1月6日）到Unix纪元（1970年1月1日）之间的秒数
  const int64_t GPS_TO_UNIX_SECONDS = 315964800; // 从1970到1980年的秒数（不考虑闰秒）
  const int LEAP_SECONDS = 18;                   // 从1980年到2024年的闰秒总数
  const int WEEK_SECONDS = 604800;               // 一周的秒数

  // 计算从Unix纪元开始到当前GPS时间的总秒数
  double totalSeconds = GPS_TO_UNIX_SECONDS - LEAP_SECONDS + gpsWeek * WEEK_SECONDS + gpsSec;

  // 创建ROS时间
  ros::Time rosTime;
  rosTime.fromSec(totalSeconds);

  return rosTime;
}

int main(int argc, char **argv)
{
  serial::Serial ser;
  std::string port;
  int buadrate;
  std::string frame_id;
  std::string device_model;
  double gravity_acceleration;
  bool use_gps_time;
  bool debug_display;
  double time_error_threshold;
  int data_packet_start;

  ros::init(argc, argv, "asensing");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<int>("buadrate", buadrate, 460800);
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<std::string>("device_model", device_model,
                                         "ins5711daa");
  private_node_handle.param<double>("gravity_acceleration",
                                    gravity_acceleration, 9.7883105);
  private_node_handle.param<bool>("use_gps_time", use_gps_time, true);
  private_node_handle.param<bool>("debug_display", debug_display, false);
  private_node_handle.param<double>("time_error_threshold", time_error_threshold, 0.01);

  ROS_INFO_STREAM("Device model: " << device_model);

  ros::NodeHandle nh("imu");
  // Use small publish queues to prefer fresh data and reduce end-to-end latency.
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 10);
  ros::Publisher imu_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
  ros::Publisher temp_pub = nh.advertise<std_msgs::Float32>("temperature", 10);
  ros::Publisher sat_pub = nh.advertise<std_msgs::UInt8>("satellites", 10);

  ros::ServiceServer service =
      nh.advertiseService("set_zero_orientation", set_zero_orientation);

  sensor_msgs::Imu imu;
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity_covariance[0] = -1.0;
  imu.linear_acceleration_covariance[0] = -1.0;

  sensor_msgs::NavSatFix gps_msg;
  gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  std::string input;
  std::string read;
  zero_orientation_set = false;
  uint8_t xorcheck = 0;

  uint8_t last_sat_count = 0;
  const uint8_t kMinSatForGpsTime = 10;
  int type32_diff_pos_status = 0;
  int type32_diff_heading_status = 0;
  bool type32_updated_this_frame = false;
  bool type32_ever_received = false;
  
  const int kLength88 = 88;
  const int kLength63 = 63;
  
  // 用于监测GPS时间跳变的变量
  int time_monitor_frame_count = 0;
  ros::Time last_sys_time;
  ros::Time last_gps_time_for_monitor;
  bool time_monitor_initialized = false;
  
  while (ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        size_t bytes_to_read = ser.available() > 0 ? ser.available() : 1;
        read = ser.read(bytes_to_read);
        if (!read.empty())
        {
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.",
                    (int)read.size(), (int)input.size());
          input += read;
          
          while (input.length() >= 3)
          {
            data_packet_start = input.find(static_cast<char>(0xBD));

            if (data_packet_start != std::string::npos)
            {
              // Drop bytes before candidate header to keep parser aligned.
              if (data_packet_start > 0)
              {
                input.erase(0, data_packet_start);
              }

              // 防越界: at least 63 bytes are required to start frame validation.
              if (input.length() < static_cast<size_t>(kLength63))
              {
                break;
              }

              // Strict contiguous header check: 0xBD 0xDB 0x0B at offsets 0/1/2.
              if ((static_cast<uint8_t>(input[0]) == 0xBD) &&
                  (static_cast<uint8_t>(input[1]) == 0xDB) &&
                  (static_cast<uint8_t>(input[2]) == 0x0B))
              {
                data_packet_start = 0;
                auto frameXorValid = [&](int frame_length) -> bool {
                  if (input.length() < static_cast<size_t>(frame_length))
                  {
                    return false;
                  }
                  xorcheck = 0;
                  for (int i = 0; i < frame_length - 1; i++)
                  {
                    xorcheck = xorcheck ^ static_cast<uint8_t>(input[data_packet_start + i]);
                  }
                  return static_cast<uint8_t>(input[data_packet_start + frame_length - 1]) == xorcheck;
                };

                int frame_length = 0;
                if (input.length() >= static_cast<size_t>(kLength88))
                {
                  if (frameXorValid(kLength88))
                  {
                    frame_length = kLength88;
                  }
                  else if (frameXorValid(kLength63))
                  {
                    frame_length = kLength63;
                  }
                  else
                  {
                    // Header matched but checksum failed for both known frame lengths.
                    input.erase(0, 1);
                    continue;
                  }
                }
                else
                {
                  // In [63, 87] bytes, only 63-byte frame can be fully validated.
                  if (frameXorValid(kLength63))
                  {
                    frame_length = kLength63;
                  }
                  else
                  {
                    // Could be an incomplete 88-byte frame; wait for more bytes.
                    break;
                  }
                }

                if (frame_length > 0)
                {
                  // 在确认帧有效后立刻获取上位机时间，减小解析过程带来的时间延迟
                  ros::Time measurement_time = ros::Time::now();

                  type32_updated_this_frame = false;

                  // get RPY
                  short int roll =
                      ((0xff & (char)input[data_packet_start + 4]) << 8) |
                      (0xff & (char)input[data_packet_start + 3]);
                  short int pitch =
                      ((0xff & (char)input[data_packet_start + 6]) << 8) |
                      (0xff & (char)input[data_packet_start + 5]);
                  short int yaw =
                      ((0xff & (char)input[data_packet_start + 8]) << 8) |
                      (0xff & (char)input[data_packet_start + 7]);
                  
                  // 如果吊装只有pitch yaw需要调整坐标轴
                  short int *temp = (short int *)&roll;
                  float rollf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int *)&pitch;
                  float pitchf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int *)&yaw;
                  float yawf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);

                  const double roll_deg = rollf * 180.0 / M_PI;
                  const double pitch_deg = pitchf * 180.0 / M_PI;
                  const double yaw_deg = yawf * 180.0 / M_PI;

                  // get gyro values
                  short int gx =
                      ((0xff & (char)input[data_packet_start + 10]) << 8) |
                      (0xff & (char)input[data_packet_start + 9]);
                  short int gy =
                      ((0xff & (char)input[data_packet_start + 12]) << 8) |
                      (0xff & (char)input[data_packet_start + 11]);
                  short int gz =
                      ((0xff & (char)input[data_packet_start + 14]) << 8) |
                      (0xff & (char)input[data_packet_start + 13]);

                  temp = (short int *)&gx;
                  float gxf = (*temp) * 300.0 / 32768 * (M_PI / 180.0);
                  temp = (short int *)&gy;
                  float gyf = (*temp) * 300.0 / 32768 * (M_PI / 180.0); 
                  temp = (short int *)&gz;
                  float gzf = (*temp) * 300.0 / 32768 * (M_PI / 180.0); 

                  const double gx_deg = gxf * 180.0 / M_PI;
                  const double gy_deg = gyf * 180.0 / M_PI;
                  const double gz_deg = gzf * 180.0 / M_PI;

                  // get acelerometer values
                  short int ax =
                      ((0xff & (char)input[data_packet_start + 16]) << 8) |
                      (0xff & (char)input[data_packet_start + 15]);
                  short int ay =
                      ((0xff & (char)input[data_packet_start + 18]) << 8) |
                      (0xff & (char)input[data_packet_start + 17]);
                  short int az =
                      ((0xff & (char)input[data_packet_start + 20]) << 8) |
                      (0xff & (char)input[data_packet_start + 19]);

                  temp = (short int *)&ax;
                  float axf = *temp * 12.0 / 32768 * gravity_acceleration;
                  temp = (short int *)&ay;
                  float ayf = *temp * 12.0 / 32768 * gravity_acceleration; 
                  temp = (short int *)&az;
                  float azf = *temp * 12.0 / 32768 * gravity_acceleration; 

                  // get gps values (Standard Precision)
                  int latitude =
                      (((0xff & (char)input[data_packet_start + 24]) << 24) |
                       ((0xff & (char)input[data_packet_start + 23]) << 16) |
                       ((0xff & (char)input[data_packet_start + 22]) << 8) |
                       (0xff & (char)input[data_packet_start + 21]));
                  int longitude =
                      (((0xff & (char)input[data_packet_start + 28]) << 24) |
                       ((0xff & (char)input[data_packet_start + 27]) << 16) |
                       ((0xff & (char)input[data_packet_start + 26]) << 8) |
                       (0xff & (char)input[data_packet_start + 25]));
                  int altitude =
                      (((0xff & (char)input[data_packet_start + 32]) << 24) |
                       ((0xff & (char)input[data_packet_start + 31]) << 16) |
                       ((0xff & (char)input[data_packet_start + 30]) << 8) |
                       (0xff & (char)input[data_packet_start + 29]));

                  int *tempA = (int *)&latitude;
                  double latitudef = *tempA * 1e-7L;
                  tempA = (int *)&longitude;
                  double longitudef = *tempA * 1e-7L;
                  tempA = (int *)&altitude;
                  double altitudef = *tempA * 1e-3L;
                  double high_prec_lat = latitudef;
                  double high_prec_lon = longitudef;

                  // 【新增】解析偏移 39: INS 状态
                  uint8_t ins_status = (0xff & (char)input[data_packet_start + 39]);
                  bool pos_initialized   = ins_status & 0x01;
                  bool vel_initialized   = ins_status & 0x02;
                  bool att_initialized   = ins_status & 0x04;
                  bool head_initialized  = ins_status & 0x08;
                  ROS_DEBUG("INS Status - Pos: %d, Vel: %d, Att: %d, Head: %d", pos_initialized, vel_initialized, att_initialized, head_initialized);

                  // 速度解析
                  short int northSpeed =
                      ((0xff & (char)input[data_packet_start + 34]) << 8) |
                      (0xff & (char)input[data_packet_start + 33]);
                  short int eastSpeed =
                      ((0xff & (char)input[data_packet_start + 36]) << 8) |
                      (0xff & (char)input[data_packet_start + 35]);
                  short int groundSpeed =
                      ((0xff & (char)input[data_packet_start + 38]) << 8) |
                      (0xff & (char)input[data_packet_start + 37]);

                  temp = (short int *)&northSpeed;
                  float northSpeedf = (*temp) * 1e2 / 32768;
                  temp = (short int *)&eastSpeed;
                  float eastSpeedf = (*temp) * 1e2 / 32768;
                  temp = (short int *)&groundSpeed;
                  float groundSpeedf = (*temp) * 1e2 / 32768;

                  // 时间解析
                  uint32_t gpsTime =
                      ((0xff & (char)input[data_packet_start + 55]) << 24) |
                      ((0xff & (char)input[data_packet_start + 54]) << 16) |
                      ((0xff & (char)input[data_packet_start + 53]) << 8) |
                      (0xff & (char)input[data_packet_start + 52]);
                  double gpsTimeMilliseconds = gpsTime * 0.00025;

                  // 轮询数据解析
                  uint8_t type = (0xff & (char)input[data_packet_start + 56]);
                  int16_t Data1 =
                      ((0xff & (char)input[data_packet_start + 47]) << 8) |
                      (0xff & (char)input[data_packet_start + 46]);
                  int16_t Data2 =
                      ((0xff & (char)input[data_packet_start + 49]) << 8) |
                      (0xff & (char)input[data_packet_start + 48]);
                  int16_t Data3 =
                      ((0xff & (char)input[data_packet_start + 51]) << 8) |
                      (0xff & (char)input[data_packet_start + 50]);

                  switch (type)
                  {
                  case 0:
                  {
                    const double lat_std = std::exp(Data1 / 100.0);
                    const double lon_std = std::exp(Data2 / 100.0);
                    const double alt_std = std::exp(Data3 / 100.0);
                    gps_msg.position_covariance[0] = lon_std * lon_std;
                    gps_msg.position_covariance[4] = lat_std * lat_std;
                    gps_msg.position_covariance[8] = alt_std * alt_std;
                    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    break;
                  }
                  case 1: 
                  {
                    // 【新增】定速信息精度 (解析保留不发布)
                    double vn_std = std::exp(Data1 / 100.0);
                    double ve_std = std::exp(Data2 / 100.0);
                    double vd_std = std::exp(Data3 / 100.0);
                    ROS_DEBUG("Velocity STD - N: %f, E: %f, D: %f", vn_std, ve_std, vd_std);
                    break;
                  }
                  case 2:
                  {
                    // 【新增】姿态信息精度 (解析保留不发布)
                    double roll_std = std::exp(Data1 / 100.0);
                    double pitch_std = std::exp(Data2 / 100.0);
                    double yaw_std = std::exp(Data3 / 100.0);

                    // Protocol provides attitude std; fill IMU orientation covariance.
                    const double roll_std_rad = roll_std * (M_PI / 180.0);
                    const double pitch_std_rad = pitch_std * (M_PI / 180.0);
                    const double yaw_std_rad = yaw_std * (M_PI / 180.0);
                    imu.orientation_covariance[0] = roll_std_rad * roll_std_rad;
                    imu.orientation_covariance[1] = 0.0;
                    imu.orientation_covariance[2] = 0.0;
                    imu.orientation_covariance[3] = 0.0;
                    imu.orientation_covariance[4] = pitch_std_rad * pitch_std_rad;
                    imu.orientation_covariance[5] = 0.0;
                    imu.orientation_covariance[6] = 0.0;
                    imu.orientation_covariance[7] = 0.0;
                    imu.orientation_covariance[8] = yaw_std_rad * yaw_std_rad;

                    ROS_DEBUG("Attitude STD - R: %f, P: %f, Y: %f", roll_std, pitch_std, yaw_std);
                    break;
                  }
                  case 22:
                  {
                    std_msgs::Float32 temp_msg;
                    temp_msg.data = Data1 * 200.0f / 32768.0f;
                    temp_pub.publish(temp_msg);
                    break;
                  }
                  case 32:
                  {
                    switch (Data1)
                    {
                    case 0:  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
                    case 16: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
                    case 18: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
                    case 17:
                    case 32:
                    case 33:
                    case 34:
                    case 48: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
                    case 49: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
                    case 50: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
                    default: break;
                    }
                    std_msgs::UInt8 sat_msg;
                    sat_msg.data = static_cast<uint8_t>(Data2);
                    last_sat_count = sat_msg.data;
                    type32_diff_pos_status = static_cast<int>(Data1);
                    type32_diff_heading_status = static_cast<int>(Data3);
                    type32_updated_this_frame = true;
                    type32_ever_received = true;
                    sat_pub.publish(sat_msg);
                    break;
                  }
                  default:
                    break;
                  }

                    if (frame_length == kLength88)
                    {
                    // 88-byte extension fields: high-precision GNSS + no-gravity acceleration.
                    int64_t hp_lat_raw =
                      ((int64_t)(0xff & input[data_packet_start + 70]) << 56) |
                      ((int64_t)(0xff & input[data_packet_start + 69]) << 48) |
                      ((int64_t)(0xff & input[data_packet_start + 68]) << 40) |
                      ((int64_t)(0xff & input[data_packet_start + 67]) << 32) |
                      ((int64_t)(0xff & input[data_packet_start + 66]) << 24) |
                      ((int64_t)(0xff & input[data_packet_start + 65]) << 16) |
                      ((int64_t)(0xff & input[data_packet_start + 64]) << 8) |
                      ((int64_t)(0xff & input[data_packet_start + 63]));

                    int64_t hp_lon_raw =
                      ((int64_t)(0xff & input[data_packet_start + 78]) << 56) |
                      ((int64_t)(0xff & input[data_packet_start + 77]) << 48) |
                      ((int64_t)(0xff & input[data_packet_start + 76]) << 40) |
                      ((int64_t)(0xff & input[data_packet_start + 75]) << 32) |
                      ((int64_t)(0xff & input[data_packet_start + 74]) << 24) |
                      ((int64_t)(0xff & input[data_packet_start + 73]) << 16) |
                      ((int64_t)(0xff & input[data_packet_start + 72]) << 8) |
                      ((int64_t)(0xff & input[data_packet_start + 71]));

                    high_prec_lat = hp_lat_raw * 1e-8;
                    high_prec_lon = hp_lon_raw * 1e-8;
                    ROS_DEBUG("High Precision Lat: %.8f, Lon: %.8f", high_prec_lat, high_prec_lon);

                    short int ax_no_grav = ((0xff & (char)input[data_packet_start + 81]) << 8) | (0xff & (char)input[data_packet_start + 80]);
                    short int ay_no_grav = ((0xff & (char)input[data_packet_start + 83]) << 8) | (0xff & (char)input[data_packet_start + 82]);
                    short int az_no_grav = ((0xff & (char)input[data_packet_start + 85]) << 8) | (0xff & (char)input[data_packet_start + 84]);

                    float ax_nograv_f = ax_no_grav * 12.0 / 32768 * gravity_acceleration;
                    float ay_nograv_f = ay_no_grav * 12.0 / 32768 * gravity_acceleration;
                    float az_nograv_f = az_no_grav * 12.0 / 32768 * gravity_acceleration;
                    ROS_DEBUG("No-Grav Accel X: %f, Y: %f, Z: %f", ax_nograv_f, ay_nograv_f, az_nograv_f);
                    }

                  uint32_t gpsWeek =
                      ((0xff & (char)input[data_packet_start + 61]) << 24) |
                      ((0xff & (char)input[data_packet_start + 60]) << 16) |
                      ((0xff & (char)input[data_packet_start + 59]) << 8) |
                      (0xff & (char)input[data_packet_start + 58]);

                  const ros::Time gps_utc_time = convertGPSTimeToROSTime((int)gpsWeek, gpsTimeMilliseconds);
                  
                  if (use_gps_time && gpsWeek > 2400 && gpsTimeMilliseconds > 0)
                  {
                    if (last_sat_count < kMinSatForGpsTime)
                    {
                      ROS_WARN_THROTTLE(5.0,
                                        "GPS时间已启用，但当前卫星数过低: %u (< %u)。时间戳稳定性可能受影响。",
                                        static_cast<unsigned int>(last_sat_count),
                                        static_cast<unsigned int>(kMinSatForGpsTime));
                    }

                    // 监测GPS时间跳变
                    if (!time_monitor_initialized)
                    {
                      last_sys_time = measurement_time;
                      last_gps_time_for_monitor = gps_utc_time;
                      time_monitor_initialized = true;
                    }
                    else
                    {
                      time_monitor_frame_count++;
                      if (time_monitor_frame_count >= 100)
                      {
                        double delta_sys = (measurement_time - last_sys_time).toSec();
                        double delta_gps = (gps_utc_time - last_gps_time_for_monitor).toSec();
                        
                        // 防止除以0
                        if (std::abs(delta_sys) > 1e-5)
                        {
                          double error_ratio = std::abs(delta_gps - delta_sys) / std::abs(delta_sys);
                          // 检查误差是否大于阈值，或者GPS时间是否发生倒退或不合理跳跃
                          if (error_ratio > time_error_threshold || delta_gps < 0.0)
                          {
                            ROS_WARN("GPS时间异常! 100帧内GPS时间流逝: %.4fs, 系统时间流逝: %.4fs, 误差比例: %.2f%% (阈值: %.2f%%)", 
                                     delta_gps, delta_sys, error_ratio * 100.0, time_error_threshold * 100.0);
                          }
                        }
                        
                        last_sys_time = measurement_time;
                        last_gps_time_for_monitor = gps_utc_time;
                        time_monitor_frame_count = 0;
                      }
                    }

                    measurement_time = gps_utc_time;
                  }

                  const double cr = std::cos(rollf * 0.5);
                  const double sr = std::sin(rollf * 0.5);
                  const double cp = std::cos(pitchf * 0.5);
                  const double sp = std::sin(pitchf * 0.5);
                  const double cy = std::cos(yawf * 0.5);
                  const double sy = std::sin(yawf * 0.5);

                  geometry_msgs::Quaternion orientation;
                  orientation.w = cr * cp * cy + sr * sp * sy;
                  orientation.x = sr * cp * cy - cr * sp * sy;
                  orientation.y = cr * sp * cy + sr * cp * sy;
                  orientation.z = cr * cp * sy - sr * sp * cy;
                  
                  // ROS_INFO_STREAM("In degree::rollf: " << rollf * 180 / M_PI << " pitchf: " << pitchf * 180 / M_PI << " yawf: " << yawf * 180 / M_PI);
                  
                  imu.orientation.x = orientation.x;
                  imu.orientation.y = orientation.y;
                  imu.orientation.z = orientation.z;
                  imu.orientation.w = orientation.w;

                  imu.header.stamp = measurement_time;
                  imu.header.frame_id = frame_id;

                  imu.angular_velocity.x = gxf;
                  imu.angular_velocity.y = gyf;
                  imu.angular_velocity.z = gzf;

                  imu.linear_acceleration.x = axf;
                  imu.linear_acceleration.y = ayf;
                  imu.linear_acceleration.z = azf;

                  imu_pub.publish(imu);

                  gps_msg.header.stamp = measurement_time;
                  gps_msg.header.frame_id = frame_id;
                    const bool use_high_prec_ll_for_pub =
                      (type32_diff_pos_status == 48) ||
                      (type32_diff_pos_status == 49) ||
                      (type32_diff_pos_status == 50);
                    gps_msg.latitude = use_high_prec_ll_for_pub ? high_prec_lat : latitudef;
                    gps_msg.longitude = use_high_prec_ll_for_pub ? high_prec_lon : longitudef;
                  gps_msg.altitude = altitudef;
                  imu_gps_pub.publish(gps_msg);

                  if (debug_display)
                  {
                    std::ostringstream dbg;
                    dbg << std::fixed << std::setprecision(4)
                        << "INS Init Status: raw=" << static_cast<int>(ins_status)
                        << ", Pos=" << (pos_initialized ? "Init" : "NotInit")
                        << ", Vel=" << (vel_initialized ? "Init" : "NotInit")
                        << ", Att=" << (att_initialized ? "Init" : "NotInit")
                        << ", Head=" << (head_initialized ? "Init" : "NotInit") << "\n"
                        << "Euler(deg): roll=" << roll_deg << ", pitch=" << pitch_deg << ", yaw=" << yaw_deg << "\n"
                        << "Angular Rate(deg/s): gx=" << gx_deg << ", gy=" << gy_deg << ", gz=" << gz_deg << "\n"
                        << "Acceleration(m/s^2): ax=" << axf << ", ay=" << ayf << ", az=" << azf << "\n"
                        << "GNSS Time(UTC): week=" << gpsWeek << ", tow_s=" << gpsTimeMilliseconds
                        << ", time=" << formatRosTimeUTC(gps_utc_time) << "\n"
                        << "LLA: lat=" << latitudef << ", lon=" << longitudef << ", alt=" << altitudef << "\n"
                        << "High Prec LL: lat=" << high_prec_lat << ", lon=" << high_prec_lon << "\n"
                        << "Satellites: " << static_cast<int>(last_sat_count) << "\n";
                    if (type32_updated_this_frame)
                    {
                      dbg << "Diff Status(Type=32): pos=" << type32_diff_pos_status << " " << type32DiffStatusToString(type32_diff_pos_status)
                          << ", heading=" << type32_diff_heading_status << " " << type32DiffStatusToString(type32_diff_heading_status)
                          << " [updated_this_frame]";
                    }
                    else
                    {
                      if (type32_ever_received)
                      {
                        dbg << "Diff Status(Type=32): pos=" << type32_diff_pos_status << " " << type32DiffStatusToString(type32_diff_pos_status)
                            << ", heading=" << type32_diff_heading_status << " " << type32DiffStatusToString(type32_diff_heading_status)
                            << " [cached_last_value]";
                      }
                      else
                      {
                        dbg << "Diff Status(Type=32): N/A [waiting_first_update]";
                      }
                    }
                    ROS_INFO_STREAM(dbg.str());
                  }
                }
                else
                {
                  ROS_DEBUG("xorcheck error");
                }
                input.erase(0, data_packet_start + frame_length);
              }
              else
              {
                // 清理假帧头，继续向后搜索下一字节。
                input.erase(0, 1);
              }
            }
            else
            {
              input.clear();
            }
          }
        }
      }
      else
      {
        try
        {
          ser.setPort(port);
          ser.setBaudrate(buadrate);
          serial::Timeout to = serial::Timeout::simpleTimeout(15);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException &e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if (ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
  }
}