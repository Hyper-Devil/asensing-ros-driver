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

bool zero_orientation_set = false;

ros::Time oldtime;

int digit = 1;

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
  uint8_t last_received_message_number;
  bool received_message = false;
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
  private_node_handle.param<bool>("use_gps_time", use_gps_time, false);

  ROS_INFO_STREAM("Device model: " << device_model);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 200);
  ros::Publisher imu_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 200);
  ros::Publisher temp_pub = nh.advertise<std_msgs::Float32>("temperature", 200);
  ros::Publisher sat_pub = nh.advertise<std_msgs::UInt8>("satellites", 200);

  ros::ServiceServer service =
      nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(300); // 300 hz

  sensor_msgs::Imu imu;
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity_covariance[0] = -1.0;
  imu.linear_acceleration_covariance[0] = -1.0;

  sensor_msgs::TimeReference trigger_time_msg; 
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  bool UseTime = false;
  ros::Duration time_offset;
  ros::Time gpsRosTime;

  std::string input;
  std::string read;
  zero_orientation_set = false;
  double long i = 0;
  char xorcheck = 0;
  
  // 【修复】数据包总长度修正为 88 字节
  int Length = 88; 
  
  while (ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        if (ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.",
                    (int)read.size(), (int)input.size());
          input += read;
          
          while (input.length() >= Length)
          {
            data_packet_start = input.find(0xBD);

            if (data_packet_start != std::string::npos)
            {
              if (input.find(0x0B) - data_packet_start == 2 && input.find(0xDB) - data_packet_start == 1)
              {
                xorcheck = 0;
                // 全帧异或校验 (0 ~ 86 字节)
                for (int i = 0; i < Length - 1; i++)
                {
                  xorcheck = xorcheck ^ input[data_packet_start + i];
                }

                if (input[data_packet_start + Length - 1] == xorcheck)
                {
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
                  
                  // 只有这里需要调整坐标轴
                  short int *temp = (short int *)&roll;
                  float rollf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int *)&pitch;
                  float pitchf = (*temp) * (360.0 / 32768) * (M_PI / 180.0) * -1.0;
                  temp = (short int *)&yaw;
                  float yawf = (*temp) * (360.0 / 32768) * (M_PI / 180.0) * -1.0;

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
                  char type = (0xff & (char)input[data_packet_start + 56]);
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
                    case 48:
                    case 49:
                    case 50: gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
                    default: break;
                    }
                    std_msgs::UInt8 sat_msg;
                    sat_msg.data = static_cast<uint8_t>(Data2);
                    sat_pub.publish(sat_msg);
                    break;
                  }
                  case 33:
                  {
                    // 【新增】轮速状态 (解析保留不发布)
                    bool has_wheel_speed = (Data1 != 0);
                    ROS_DEBUG("Wheel Speed Status: %d", has_wheel_speed);
                    break;
                  }
                  default:
                    break;
                  }

                  // 【新增】高精度经纬度 (毫米级精度, 解析保留不发布)
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

                  double high_prec_lat = hp_lat_raw * 1e-8;
                  double high_prec_lon = hp_lon_raw * 1e-8;
                  ROS_DEBUG("High Precision Lat: %.8f, Lon: %.8f", high_prec_lat, high_prec_lon);

                  // 【新增】去重力加速度 (解析保留不发布)
                  short int ax_no_grav = ((0xff & (char)input[data_packet_start + 81]) << 8) | (0xff & (char)input[data_packet_start + 80]);
                  short int ay_no_grav = ((0xff & (char)input[data_packet_start + 83]) << 8) | (0xff & (char)input[data_packet_start + 82]);
                  short int az_no_grav = ((0xff & (char)input[data_packet_start + 85]) << 8) | (0xff & (char)input[data_packet_start + 84]);
                  
                  float ax_nograv_f = ax_no_grav * 12.0 / 32768 * gravity_acceleration;
                  float ay_nograv_f = ay_no_grav * 12.0 / 32768 * gravity_acceleration;
                  float az_nograv_f = az_no_grav * 12.0 / 32768 * gravity_acceleration;
                  ROS_DEBUG("No-Grav Accel X: %f, Y: %f, Z: %f", ax_nograv_f, ay_nograv_f, az_nograv_f);

                  received_message = true;

                  uint32_t gpsWeek =
                      ((0xff & (char)input[data_packet_start + 61]) << 24) |
                      ((0xff & (char)input[data_packet_start + 60]) << 16) |
                      ((0xff & (char)input[data_packet_start + 59]) << 8) |
                      (0xff & (char)input[data_packet_start + 58]);

                  ros::Time measurement_time = ros::Time::now();
                  if (use_gps_time && gpsWeek > 0 && gpsTimeMilliseconds > 0)
                  {
                    measurement_time = convertGPSTimeToROSTime((int)gpsWeek, gpsTimeMilliseconds);
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
                  gps_msg.header.frame_id = "world";
                  bool gps_jumped = (gps_msg.longitude != 0.0 && gps_msg.latitude != 0.0) &&
                                    (std::abs(gps_msg.altitude - altitudef) > 20.0 ||
                                     std::abs(gps_msg.longitude - longitudef) > 20.0 ||
                                     std::abs(gps_msg.latitude - latitudef) > 20.0);
                  if (!gps_jumped)
                  {
                    gps_msg.latitude = latitudef;
                    gps_msg.longitude = longitudef;
                    gps_msg.altitude = altitudef;
                    imu_gps_pub.publish(gps_msg);
                  }
                  else
                  {
                    ROS_WARN_THROTTLE(1.0, "GPS data jumped. Skipping publication for this frame.");
                  }
                }
                else
                {
                  ROS_INFO("xorcheck error");
                }
                // 清理已处理完的整个包 (88 字节)
                input.erase(0, data_packet_start + Length); 
              }
              else
              {
                // 清理假帧头
                input.erase(0, data_packet_start + 1); 
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
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
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
    r.sleep();
  }
}