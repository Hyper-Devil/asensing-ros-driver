#include <fstream>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
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
#include <asensing_ros_driver/InsData.h>

bool zero_orientation_set = false;

std::string type32DiffStatusToString(int status)
{
  switch (status)
  {
  case 0:   return "NONE";
  case 1:   return "FIXEDPOS";
  case 2:   return "FIXEDHEIGHT";
  case 8:   return "DOPPLER_VELOCITY";
  case 16:  return "SINGLE";
  case 17:  return "PSRDIFF";
  case 18:  return "SBAS";
  case 32:  return "L1_FLOAT";
  case 33:  return "IONOFREE_FLOAT";
  case 34:  return "NARROW_FLOAT";
  case 48:  return "L1_INT";
  case 49:  return "WIDE_INT";
  case 50:  return "NARROW_INT";
  default:  return "UNKNOWN(" + std::to_string(status) + ")";
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
  // GPS纪元开始（1980-01-06）到Unix纪元（1970-01-01）的秒数
  const int64_t GPS_TO_UNIX_SECONDS = 315964800;
  const int LEAP_SECONDS = 18;   // 1980~2024 累积闰秒
  const int WEEK_SECONDS = 604800;

  double totalSeconds = static_cast<double>(GPS_TO_UNIX_SECONDS - LEAP_SECONDS)
                        + static_cast<double>(gpsWeek) * WEEK_SECONDS
                        + gpsSec;

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

  ros::init(argc, argv, "asensing");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<int>("buadrate", buadrate, 460800);
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<std::string>("device_model", device_model, "ins5711daa");
  private_node_handle.param<double>("gravity_acceleration", gravity_acceleration, 9.7883105);
  private_node_handle.param<bool>("use_gps_time", use_gps_time, true);
  private_node_handle.param<bool>("debug_display", debug_display, false);
  private_node_handle.param<double>("time_error_threshold", time_error_threshold, 0.01);

  ROS_INFO_STREAM("Device model: " << device_model);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub      = nh.advertise<sensor_msgs::Imu>("data", 10);
  ros::Publisher imu_gps_pub  = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
  ros::Publisher temp_pub     = nh.advertise<std_msgs::Float32>("temperature", 10);
  ros::Publisher sat_pub      = nh.advertise<std_msgs::UInt8>("satellites", 10);
  ros::Publisher ins_data_pub = nh.advertise<asensing_ros_driver::InsData>("ins_data", 10);

  ros::ServiceServer service =
      nh.advertiseService("set_zero_orientation", set_zero_orientation);

  sensor_msgs::Imu imu;
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity_covariance[0] = -1.0;
  imu.linear_acceleration_covariance[0] = -1.0;

  sensor_msgs::NavSatFix gps_msg;
  gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_msg.status.status  = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  std::string input;
  std::string read;
  zero_orientation_set = false;
  uint8_t xorcheck = 0;

  uint8_t last_sat_count = 0;
  const uint8_t kMinSatForGpsTime = 10;
  int type32_diff_pos_status     = 0;
  int type32_diff_heading_status = 0;
  bool type32_updated_this_frame = false;
  bool type32_ever_received      = false;

  const int kLength88 = 88;
  const int kLength63 = 63;

  // 轮询字段缓存（Type=0/1/2/22/33，每帧可能只更新其中一个）
  float cached_lat_std        = 0.0f;
  float cached_lon_std        = 0.0f;
  float cached_alt_std        = 0.0f;
  float cached_north_vel_std  = 0.0f;
  float cached_east_vel_std   = 0.0f;
  float cached_ground_vel_std = 0.0f;
  float cached_roll_std       = 0.0f;
  float cached_pitch_std      = 0.0f;
  float cached_yaw_std        = 0.0f;
  float cached_temperature    = 0.0f;
  bool  cached_temperature_valid    = false;
  bool  cached_wheel_speed_available = false;

  // GPS时间跳变监测
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
            size_t data_packet_start = input.find(static_cast<char>(0xBD));

            if (data_packet_start == std::string::npos)
            {
              input.clear();
              break;
            }

            // 丢弃帧头之前的无效字节
            if (data_packet_start > 0)
            {
              input.erase(0, data_packet_start);
            }

            // 至少需要 kLength63 字节才能开始校验
            if (input.length() < static_cast<size_t>(kLength63))
            {
              break;
            }

            // 严格连续帧头校验：0xBD 0xDB 0x0B
            if ((static_cast<uint8_t>(input[0]) != 0xBD) ||
                (static_cast<uint8_t>(input[1]) != 0xDB) ||
                (static_cast<uint8_t>(input[2]) != 0x0B))
            {
              input.erase(0, 1);
              continue;
            }

            // lambda：校验前 frame_length-1 字节的 XOR 是否等于最后一字节
            auto frameXorValid = [&](int frame_length) -> bool {
              if (input.length() < static_cast<size_t>(frame_length))
              {
                return false;
              }
              xorcheck = 0;
              for (int i = 0; i < frame_length - 1; i++)
              {
                xorcheck ^= static_cast<uint8_t>(input[i]);
              }
              return static_cast<uint8_t>(input[frame_length - 1]) == xorcheck;
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
                // 帧头匹配但两种长度均校验失败，跳过一字节继续搜索
                input.erase(0, 1);
                continue;
              }
            }
            else
            {
              // [63, 87] 字节：只能验证 63 字节帧
              if (frameXorValid(kLength63))
              {
                frame_length = kLength63;
              }
              else
              {
                // 可能是不完整的 88 字节帧，等待更多数据
                break;
              }
            }

            // 帧有效后立刻采集系统时间，减小解析延迟
            ros::Time measurement_time = ros::Time::now();

            type32_updated_this_frame = false;

            // ---- 姿态解析 (offset 3~8) ----
            // 直接转 float 后再做符号变换，避免 INT16_MIN 取反溢出（UB）
            const int16_t roll_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[4]) << 8) |
                                      static_cast<uint8_t>(input[3]));
            const int16_t pitch_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[6]) << 8) |
                                      static_cast<uint8_t>(input[5]));
            const int16_t yaw_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[8]) << 8) |
                                      static_cast<uint8_t>(input[7]));

            // 正装，FLU 坐标系（REP-103）
            float rollf  =  static_cast<float>(roll_raw)  * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
            float pitchf = -static_cast<float>(pitch_raw) * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
            float yawf   =  static_cast<float>(M_PI) / 2.0f
                           - static_cast<float>(yaw_raw) * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
            while (yawf >  static_cast<float>(M_PI)) yawf -= 2.0f * static_cast<float>(M_PI);
            while (yawf < -static_cast<float>(M_PI)) yawf += 2.0f * static_cast<float>(M_PI);

            const double roll_deg  = rollf  * 180.0 / M_PI;
            const double pitch_deg = pitchf * 180.0 / M_PI;
            const double yaw_deg   = yawf   * 180.0 / M_PI;

            // ---- 角速度解析 (offset 9~14) ----
            const int16_t gx_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[10]) << 8) |
                                      static_cast<uint8_t>(input[9]));
            const int16_t gy_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[12]) << 8) |
                                      static_cast<uint8_t>(input[11]));
            const int16_t gz_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[14]) << 8) |
                                      static_cast<uint8_t>(input[13]));

            // 正装，FLU
            const float gxf =  static_cast<float>(gx_raw) * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
            const float gyf = -static_cast<float>(gy_raw) * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
            const float gzf = -static_cast<float>(gz_raw) * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);

            const double gx_deg = gxf * 180.0 / M_PI;
            const double gy_deg = gyf * 180.0 / M_PI;
            const double gz_deg = gzf * 180.0 / M_PI;

            // ---- 含重力加速度解析 (offset 15~20) ----
            const int16_t ax_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[16]) << 8) |
                                      static_cast<uint8_t>(input[15]));
            const int16_t ay_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[18]) << 8) |
                                      static_cast<uint8_t>(input[17]));
            const int16_t az_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[20]) << 8) |
                                      static_cast<uint8_t>(input[19]));

            // 正装，FLU（ay、az 取反；直接在 float 乘中处理，避免 INT16_MIN UB）
            const float axf =  static_cast<float>(ax_raw) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
            const float ayf = -static_cast<float>(ay_raw) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
            const float azf = -static_cast<float>(az_raw) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);

            // ---- 标准精度 GNSS 位置 (offset 21~32) ----
            const int32_t latitude_raw =
                static_cast<int32_t>((static_cast<uint8_t>(input[24]) << 24) |
                                     (static_cast<uint8_t>(input[23]) << 16) |
                                     (static_cast<uint8_t>(input[22]) <<  8) |
                                      static_cast<uint8_t>(input[21]));
            const int32_t longitude_raw =
                static_cast<int32_t>((static_cast<uint8_t>(input[28]) << 24) |
                                     (static_cast<uint8_t>(input[27]) << 16) |
                                     (static_cast<uint8_t>(input[26]) <<  8) |
                                      static_cast<uint8_t>(input[25]));
            const int32_t altitude_raw =
                static_cast<int32_t>((static_cast<uint8_t>(input[32]) << 24) |
                                     (static_cast<uint8_t>(input[31]) << 16) |
                                     (static_cast<uint8_t>(input[30]) <<  8) |
                                      static_cast<uint8_t>(input[29]));

            const double latitudef  = latitude_raw  * 1e-7;
            const double longitudef = longitude_raw * 1e-7;
            const double altitudef  = altitude_raw  * 1e-3;

            // 高精度经纬度初值退化为标准精度，88 字节帧会覆盖
            double high_prec_lat = latitudef;
            double high_prec_lon = longitudef;

            // ---- INS 状态 (offset 39) ----
            const uint8_t ins_status = static_cast<uint8_t>(input[39]);
            const bool pos_initialized  = (ins_status & 0x01) != 0;
            const bool vel_initialized  = (ins_status & 0x02) != 0;
            const bool att_initialized  = (ins_status & 0x04) != 0;
            const bool head_initialized = (ins_status & 0x08) != 0;
            ROS_DEBUG("INS Status - Pos: %d, Vel: %d, Att: %d, Head: %d",
                      pos_initialized, vel_initialized, att_initialized, head_initialized);

            // ---- NED 速度 (offset 33~38) ----
            const int16_t northSpeed_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[34]) << 8) |
                                      static_cast<uint8_t>(input[33]));
            const int16_t eastSpeed_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[36]) << 8) |
                                      static_cast<uint8_t>(input[35]));
            const int16_t groundSpeed_raw =
                static_cast<int16_t>((static_cast<uint8_t>(input[38]) << 8) |
                                      static_cast<uint8_t>(input[37]));

            const float northSpeedf  = static_cast<float>(northSpeed_raw)  * (100.0f / 32768.0f);
            const float eastSpeedf   = static_cast<float>(eastSpeed_raw)   * (100.0f / 32768.0f);
            const float groundSpeedf = static_cast<float>(groundSpeed_raw) * (100.0f / 32768.0f);

            // ---- GPS 时间 (offset 52~55 ms, 58~61 week) ----
            const uint32_t gpsTime_raw =
                (static_cast<uint32_t>(static_cast<uint8_t>(input[55])) << 24) |
                (static_cast<uint32_t>(static_cast<uint8_t>(input[54])) << 16) |
                (static_cast<uint32_t>(static_cast<uint8_t>(input[53])) <<  8) |
                 static_cast<uint32_t>(static_cast<uint8_t>(input[52]));
            const double gpsTimeSeconds = gpsTime_raw * 0.00025;

            const uint32_t gpsWeek =
                (static_cast<uint32_t>(static_cast<uint8_t>(input[61])) << 24) |
                (static_cast<uint32_t>(static_cast<uint8_t>(input[60])) << 16) |
                (static_cast<uint32_t>(static_cast<uint8_t>(input[59])) <<  8) |
                 static_cast<uint32_t>(static_cast<uint8_t>(input[58]));

            const bool gps_time_valid = (gpsWeek > 2400) && (gpsTimeSeconds > 0.0);

            // ---- 轮询字段解析 (offset 46~51, 56) ----
            const uint8_t type = static_cast<uint8_t>(input[56]);
            const int16_t Data1 =
                static_cast<int16_t>((static_cast<uint8_t>(input[47]) << 8) |
                                      static_cast<uint8_t>(input[46]));
            const int16_t Data2 =
                static_cast<int16_t>((static_cast<uint8_t>(input[49]) << 8) |
                                      static_cast<uint8_t>(input[48]));
            const int16_t Data3 =
                static_cast<int16_t>((static_cast<uint8_t>(input[51]) << 8) |
                                      static_cast<uint8_t>(input[50]));

            switch (type)
            {
            case 0:
            {
              cached_lat_std = static_cast<float>(std::exp(Data1 / 100.0));
              cached_lon_std = static_cast<float>(std::exp(Data2 / 100.0));
              cached_alt_std = static_cast<float>(std::exp(Data3 / 100.0));
              gps_msg.position_covariance[0] = static_cast<double>(cached_lon_std) * cached_lon_std;
              gps_msg.position_covariance[4] = static_cast<double>(cached_lat_std) * cached_lat_std;
              gps_msg.position_covariance[8] = static_cast<double>(cached_alt_std) * cached_alt_std;
              gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
              break;
            }
            case 1:
            {
              cached_north_vel_std  = static_cast<float>(std::exp(Data1 / 100.0));
              cached_east_vel_std   = static_cast<float>(std::exp(Data2 / 100.0));
              cached_ground_vel_std = static_cast<float>(std::exp(Data3 / 100.0));
              ROS_DEBUG("Velocity STD - N: %f, E: %f, D: %f",
                        cached_north_vel_std, cached_east_vel_std, cached_ground_vel_std);
              break;
            }
            case 2:
            {
              cached_roll_std  = static_cast<float>(std::exp(Data1 / 100.0));
              cached_pitch_std = static_cast<float>(std::exp(Data2 / 100.0));
              cached_yaw_std   = static_cast<float>(std::exp(Data3 / 100.0));

              const double roll_std_rad  = cached_roll_std  * (M_PI / 180.0);
              const double pitch_std_rad = cached_pitch_std * (M_PI / 180.0);
              const double yaw_std_rad   = cached_yaw_std   * (M_PI / 180.0);
              imu.orientation_covariance[0] = roll_std_rad  * roll_std_rad;
              imu.orientation_covariance[1] = 0.0;
              imu.orientation_covariance[2] = 0.0;
              imu.orientation_covariance[3] = 0.0;
              imu.orientation_covariance[4] = pitch_std_rad * pitch_std_rad;
              imu.orientation_covariance[5] = 0.0;
              imu.orientation_covariance[6] = 0.0;
              imu.orientation_covariance[7] = 0.0;
              imu.orientation_covariance[8] = yaw_std_rad   * yaw_std_rad;
              ROS_DEBUG("Attitude STD - R: %f, P: %f, Y: %f",
                        cached_roll_std, cached_pitch_std, cached_yaw_std);
              break;
            }
            case 22:
            {
              cached_temperature       = Data1 * 200.0f / 32768.0f;
              cached_temperature_valid = true;
              std_msgs::Float32 temp_msg;
              temp_msg.data = cached_temperature;
              temp_pub.publish(temp_msg);
              break;
            }
            case 32:
            {
              switch (Data1)
              {
              case 0:  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;   break;
              case 16: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;       break;
              case 18: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;  break;
              case 17:
              case 32:
              case 33:
              case 34:
              case 48:
              case 49:
              case 50: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;  break;
              default: break;
              }
              last_sat_count              = static_cast<uint8_t>(Data2);
              type32_diff_pos_status      = static_cast<int>(Data1);
              type32_diff_heading_status  = static_cast<int>(Data3);
              type32_updated_this_frame   = true;
              type32_ever_received        = true;
              {
                std_msgs::UInt8 sat_msg;
                sat_msg.data = last_sat_count;
                sat_pub.publish(sat_msg);
              }
              break;
            }
            case 33:
            {
              cached_wheel_speed_available = (Data2 != 0);
              break;
            }
            default:
              break;
            }

            // ---- 88 字节扩展帧：高精度 GNSS + 去重力加速度 (offset 63~85) ----
            bool high_precision_valid = false;
            float ax_nograv_f = 0.0f;
            float ay_nograv_f = 0.0f;
            float az_nograv_f = 0.0f;

            if (frame_length == kLength88)
            {
              high_precision_valid = true;

              const int64_t hp_lat_raw =
                (static_cast<int64_t>(static_cast<uint8_t>(input[70])) << 56) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[69])) << 48) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[68])) << 40) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[67])) << 32) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[66])) << 24) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[65])) << 16) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[64])) <<  8) |
                 static_cast<int64_t>(static_cast<uint8_t>(input[63]));

              const int64_t hp_lon_raw =
                (static_cast<int64_t>(static_cast<uint8_t>(input[78])) << 56) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[77])) << 48) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[76])) << 40) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[75])) << 32) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[74])) << 24) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[73])) << 16) |
                (static_cast<int64_t>(static_cast<uint8_t>(input[72])) <<  8) |
                 static_cast<int64_t>(static_cast<uint8_t>(input[71]));

              high_prec_lat = hp_lat_raw * 1e-8;
              high_prec_lon = hp_lon_raw * 1e-8;
              ROS_DEBUG("High Precision Lat: %.8f, Lon: %.8f", high_prec_lat, high_prec_lon);

              // 去重力加速度，同样应用 FLU 变换
              const int16_t ax_ng =
                  static_cast<int16_t>((static_cast<uint8_t>(input[81]) << 8) |
                                        static_cast<uint8_t>(input[80]));
              const int16_t ay_ng =
                  static_cast<int16_t>((static_cast<uint8_t>(input[83]) << 8) |
                                        static_cast<uint8_t>(input[82]));
              const int16_t az_ng =
                  static_cast<int16_t>((static_cast<uint8_t>(input[85]) << 8) |
                                        static_cast<uint8_t>(input[84]));

              // InsData 发原始值，不做 FLU 变换
              ax_nograv_f = static_cast<float>(ax_ng) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
              ay_nograv_f = static_cast<float>(ay_ng) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
              az_nograv_f = static_cast<float>(az_ng) * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
              ROS_DEBUG("No-Grav Accel (raw) X: %f, Y: %f, Z: %f", ax_nograv_f, ay_nograv_f, az_nograv_f);
            }

            // ---- 时间戳选择 ----
            const ros::Time gps_utc_time = convertGPSTimeToROSTime(static_cast<int>(gpsWeek), gpsTimeSeconds);

            if (use_gps_time && gps_time_valid)
            {
              if (last_sat_count < kMinSatForGpsTime)
              {
                ROS_WARN_THROTTLE(5.0, "LOWER STA. COUNT: %u (< %u)",
                                  static_cast<unsigned int>(last_sat_count),
                                  static_cast<unsigned int>(kMinSatForGpsTime));
              }

              if (!time_monitor_initialized)
              {
                last_sys_time               = measurement_time;
                last_gps_time_for_monitor   = gps_utc_time;
                time_monitor_initialized    = true;
              }
              else
              {
                time_monitor_frame_count++;
                if (time_monitor_frame_count >= 100)
                {
                  const double delta_sys = (measurement_time - last_sys_time).toSec();
                  const double delta_gps = (gps_utc_time - last_gps_time_for_monitor).toSec();

                  if (std::abs(delta_sys) > 1e-5)
                  {
                    const double error_ratio = std::abs(delta_gps - delta_sys) / std::abs(delta_sys);
                    if (error_ratio > time_error_threshold || delta_gps < 0.0)
                    {
                      ROS_WARN("GPS时间异常! 100帧内GPS时间流逝: %.4fs, 系统时间流逝: %.4fs, "
                               "误差比例: %.2f%% (阈值: %.2f%%)",
                               delta_gps, delta_sys,
                               error_ratio * 100.0, time_error_threshold * 100.0);
                    }
                  }

                  last_sys_time              = measurement_time;
                  last_gps_time_for_monitor  = gps_utc_time;
                  time_monitor_frame_count   = 0;
                }
              }

              measurement_time = gps_utc_time;
            }

            // ---- 四元数计算 ----
            const double cr = std::cos(rollf  * 0.5);
            const double sr = std::sin(rollf  * 0.5);
            const double cp = std::cos(pitchf * 0.5);
            const double sp = std::sin(pitchf * 0.5);
            const double cy = std::cos(yawf   * 0.5);
            const double sy = std::sin(yawf   * 0.5);

            geometry_msgs::Quaternion orientation;
            orientation.w = cr * cp * cy + sr * sp * sy;
            orientation.x = sr * cp * cy - cr * sp * sy;
            orientation.y = cr * sp * cy + sr * cp * sy;
            orientation.z = cr * cp * sy - sr * sp * cy;

            // ---- 发布 imu/data ----
            imu.header.stamp    = measurement_time;
            imu.header.frame_id = frame_id;
            imu.orientation     = orientation;
            imu.angular_velocity.x    = gxf;
            imu.angular_velocity.y    = gyf;
            imu.angular_velocity.z    = gzf;
            imu.linear_acceleration.x = axf;
            imu.linear_acceleration.y = ayf;
            imu.linear_acceleration.z = azf;
            imu_pub.publish(imu);

            // ---- 发布 imu/gps ----
            const bool use_high_prec =
                (type32_diff_pos_status == 48) ||
                (type32_diff_pos_status == 49) ||
                (type32_diff_pos_status == 50);

            gps_msg.header.stamp    = measurement_time;
            gps_msg.header.frame_id = frame_id;
            gps_msg.latitude  = use_high_prec ? high_prec_lat : latitudef;
            gps_msg.longitude = use_high_prec ? high_prec_lon : longitudef;
            gps_msg.altitude  = altitudef;
            imu_gps_pub.publish(gps_msg);

            // ---- 发布 imu/ins_data（全量数据，header.stamp = ros::Time::now()）----
            if (ins_data_pub.getNumSubscribers() > 0)
            {
              // 协议原始物理量，不经过 FLU 坐标变换
              const float roll_s  = static_cast<float>(roll_raw)  * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float pitch_s = static_cast<float>(pitch_raw) * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float yaw_s   = static_cast<float>(yaw_raw)   * (360.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float gx_s    = static_cast<float>(gx_raw)    * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float gy_s    = static_cast<float>(gy_raw)    * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float gz_s    = static_cast<float>(gz_raw)    * (300.0f / 32768.0f) * (static_cast<float>(M_PI) / 180.0f);
              const float ax_s    = static_cast<float>(ax_raw)    * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
              const float ay_s    = static_cast<float>(ay_raw)    * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);
              const float az_s    = static_cast<float>(az_raw)    * (12.0f / 32768.0f) * static_cast<float>(gravity_acceleration);

              asensing_ros_driver::InsData ins;
              ins.header.stamp    = ros::Time::now();
              ins.header.frame_id = frame_id;

              ins.gps_week       = gpsWeek;
              ins.gps_tow        = gpsTimeSeconds;
              ins.gps_time_valid = gps_time_valid;

              ins.roll  = roll_s;
              ins.pitch = pitch_s;
              ins.yaw   = yaw_s;

              ins.angular_velocity_x = gx_s;
              ins.angular_velocity_y = gy_s;
              ins.angular_velocity_z = gz_s;

              ins.linear_acceleration_x = ax_s;
              ins.linear_acceleration_y = ay_s;
              ins.linear_acceleration_z = az_s;

              ins.no_grav_acceleration_x = ax_nograv_f;
              ins.no_grav_acceleration_y = ay_nograv_f;
              ins.no_grav_acceleration_z = az_nograv_f;

              ins.latitude  = latitudef;
              ins.longitude = longitudef;
              ins.altitude  = altitudef;

              ins.hp_latitude         = high_prec_lat;
              ins.hp_longitude        = high_prec_lon;
              ins.high_precision_valid = high_precision_valid;

              ins.north_velocity  = northSpeedf;
              ins.east_velocity   = eastSpeedf;
              ins.ground_velocity = groundSpeedf;

              ins.ins_status_raw    = ins_status;
              ins.pos_initialized   = pos_initialized;
              ins.vel_initialized   = vel_initialized;
              ins.att_initialized   = att_initialized;
              ins.heading_initialized = head_initialized;

              ins.satellite_count       = last_sat_count;
              ins.diff_pos_status       = type32_diff_pos_status;
              ins.diff_heading_status   = type32_diff_heading_status;

              ins.lat_std         = cached_lat_std;
              ins.lon_std         = cached_lon_std;
              ins.alt_std         = cached_alt_std;
              ins.north_vel_std   = cached_north_vel_std;
              ins.east_vel_std    = cached_east_vel_std;
              ins.ground_vel_std  = cached_ground_vel_std;
              ins.roll_std        = cached_roll_std;
              ins.pitch_std       = cached_pitch_std;
              ins.yaw_std         = cached_yaw_std;

              ins.temperature       = cached_temperature;
              ins.temperature_valid = cached_temperature_valid;

              ins.wheel_speed_available = cached_wheel_speed_available;

              ins_data_pub.publish(ins);
            }

            // ---- 调试输出 ----
            if (debug_display)
            {
              std::ostringstream dbg;
              dbg << std::fixed
                  << "INS Init Status: Pos=" << (pos_initialized  ? "Init" : "NotInit")
                  << ", Vel=" << (vel_initialized  ? "Init" : "NotInit")
                  << ", Att=" << (att_initialized  ? "Init" : "NotInit")
                  << ", Head=" << (head_initialized ? "Init" : "NotInit") << "\n"
                  << "Euler(deg): roll=" << roll_deg << ", pitch=" << pitch_deg << ", yaw=" << yaw_deg << "\n"
                  << "Angular Rate(deg/s): gx=" << gx_deg << ", gy=" << gy_deg << ", gz=" << gz_deg << "\n"
                  << "Acceleration(m/s^2): ax=" << axf << ", ay=" << ayf << ", az=" << azf << "\n"
                  << "GNSS Time(UTC): week=" << gpsWeek << ", tow_s=" << gpsTimeSeconds
                  << ", time=" << formatRosTimeUTC(gps_utc_time) << "\n"
                  << "LLA: lat=" << latitudef << ", lon=" << longitudef << ", alt=" << altitudef << "\n"
                  << "High Prec LL: lat=" << high_prec_lat << ", lon=" << high_prec_lon << "\n"
                  << "Velocity(m/s): N=" << northSpeedf << ", E=" << eastSpeedf << ", D=" << groundSpeedf << "\n"
                  << "Satellites: " << static_cast<int>(last_sat_count) << "\n";
              if (type32_updated_this_frame)
              {
                dbg << "Diff Status(Type=32): pos=" << type32_diff_pos_status
                    << " " << type32DiffStatusToString(type32_diff_pos_status)
                    << ", heading=" << type32_diff_heading_status
                    << " " << type32DiffStatusToString(type32_diff_heading_status)
                    << " [updated_this_frame]";
              }
              else if (type32_ever_received)
              {
                dbg << "Diff Status(Type=32): pos=" << type32_diff_pos_status
                    << " " << type32DiffStatusToString(type32_diff_pos_status)
                    << ", heading=" << type32_diff_heading_status
                    << " " << type32DiffStatusToString(type32_diff_heading_status)
                    << " [cached_last_value]";
              }
              else
              {
                dbg << "Diff Status(Type=32): N/A [waiting_first_update]";
              }
              ROS_INFO_STREAM(dbg.str());
            }

            input.erase(0, frame_length);
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
