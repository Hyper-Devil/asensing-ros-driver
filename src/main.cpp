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
  double time_offset_in_seconds;
  double gravity_acceleration;
  bool use_gps_time;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;
  int Offset_time;     // 周内秒加每个月的偏移量，需要根据每个月的不同天数进行调整
  int UTC_Offset_time; // gps周内秒转UTC或在北京时间的偏移量

  ros::init(argc, argv, "asensing");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<int>("buadrate", buadrate, 230400);
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<std::string>("device_model", device_model,
                                         "ins570d");
  private_node_handle.param<double>("time_offset_in_seconds",
                                    time_offset_in_seconds, 0.0);
  private_node_handle.param<double>("gravity_acceleration",
                                    gravity_acceleration, 9.7883105);
  private_node_handle.param<bool>("use_gps_time", use_gps_time, false);
  private_node_handle.param<int>("Offset_time", Offset_time, 0);         // 周内秒加每个月的偏移量，需要根据每个月的不同天数进行调整
  private_node_handle.param<int>("UTC_Offset_time", UTC_Offset_time, 0); // gps周内秒转UTC或在北京时间的偏移量

  ROS_INFO_STREAM("Device model: " << device_model);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 200);
  ros::Publisher imu_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 200);
  ros::Publisher temp_pub = nh.advertise<std_msgs::Float32>("temperature", 200);
  ros::Publisher sat_pub = nh.advertise<std_msgs::UInt8>("satellites", 200);
  // ros::Publisher trigger_time_pub =
  //     nh.advertise<sensor_msgs::TimeReference>("trigger_time", 100);

  ros::ServiceServer service =
      nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(300); // 200 hz

  sensor_msgs::Imu imu;
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity_covariance[0] = -1.0;
  imu.linear_acceleration_covariance[0] = -1.0;

  sensor_msgs::TimeReference trigger_time_msg; ////
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
  int Length = 63; // data length 63
  while (ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if (ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i "
                    "characters of old input.",
                    (int)read.size(), (int)input.size());
          input += read;
          // while there might be a complete package in input
          while (input.length() >= Length)
          {
            // print input in hexadecimal format
            // for (int i = 0; i < Length; i++) {
            //   printf("%02X ", (unsigned char)input[i]);
            // }
            // printf("\n");
            // parse for data packets
            data_packet_start = input.find(0xBD);

            if (data_packet_start != std::string::npos)
            {
              if (input.find(0x0B) - data_packet_start == 2 && input.find(0xDB) - data_packet_start == 1)
              {
                // ROS_INFO("Found possible start of data packet at position %d", data_packet_start);
                xorcheck = 0;
                // std::string pack;
                for (int i = 0; i < Length - 1; i++)
                {
                  xorcheck = xorcheck ^ input[data_packet_start + i];
                  // pack+=input[data_packet_start+i];
                }

                // input[data_packet_start +Length-1]==xorcheck
                if (input[data_packet_start + Length - 1] == xorcheck)
                {
                  // ROS_DEBUG("seems to be a real data package: long enough and
                  // printf("initialDATA:%x \n",input[data_packet_start+2]);

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

                  // calculate RPY in rad
                  // 只有这里需要。将y右z下转y左z上 * -1.0
                  short int *temp = (short int *)&roll;
                  float rollf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int *)&pitch;
                  float pitchf =
                      (*temp) * (360.0 / 32768) * (M_PI / 180.0) * -1.0;
                  temp = (short int *)&yaw;
                  float yawf =
                      (*temp) * (360.0 / 32768) * (M_PI / 180.0) * -1.0;

                  // ROS_INFO("yawf:%f",yawf);
                  // ROS_INFO("rollf:%f",rollf);
                  // ROS_INFO("pitchf:%f",pitchf);

                  // get gyro values
                  // unit deg/s
                  short int gx =
                      ((0xff & (char)input[data_packet_start + 10]) << 8) |
                      (0xff & (char)input[data_packet_start + 9]);
                  short int gy =
                      ((0xff & (char)input[data_packet_start + 12]) << 8) |
                      (0xff & (char)input[data_packet_start + 11]);
                  short int gz =
                      ((0xff & (char)input[data_packet_start + 14]) << 8) |
                      (0xff & (char)input[data_packet_start + 13]);
                  // cal unit from deg/s to rad/s
                  // 取消。将y右z下转y左z上 * -1.0
                  temp = (short int *)&gx;
                  float gxf = (*temp) * 300.0 / 32768 * (M_PI / 180.0);
                  temp = (short int *)&gy;
                  float gyf = (*temp) * 300.0 / 32768 * (M_PI / 180.0) ;//* -1.0;
                  temp = (short int *)&gz;
                  float gzf = (*temp) * 300.0 / 32768 * (M_PI / 180.0) ;//* -1.0;

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

                  // calculate accelerations unit from g to m/s²
                  // 将y右z下转y左z上 * -1.0
                  temp = (short int *)&ax;
                  float axf = *temp * 12.0 / 32768 * gravity_acceleration;
                  temp = (short int *)&ay;
                  float ayf = *temp * 12.0 / 32768 * gravity_acceleration ; //* -1.0;
                  temp = (short int *)&az;
                  float azf = *temp * 12.0 / 32768 * gravity_acceleration ; //* -1.0;

                  // get gps values
                  int latitude =
                      (((0xff & (char)input[data_packet_start + 24]) << 24) |
                       ((0xff & (char)input[data_packet_start + 23]) << 16) |
                       ((0xff & (char)input[data_packet_start + 22]) << 8) |
                       0xff & (char)input[data_packet_start + 21]);
                  int longitude =
                      (((0xff & (char)input[data_packet_start + 28]) << 24) |
                       ((0xff & (char)input[data_packet_start + 27]) << 16) |
                       ((0xff & (char)input[data_packet_start + 26]) << 8) |
                       0xff & (char)input[data_packet_start + 25]);
                  int altitude =
                      (((0xff & (char)input[data_packet_start + 32]) << 24) |
                       ((0xff & (char)input[data_packet_start + 31]) << 16) |
                       ((0xff & (char)input[data_packet_start + 30]) << 8) |
                       0xff & (char)input[data_packet_start + 29]);

                  int *tempA = (int *)&latitude;
                  double latitudef = *tempA * 1e-7L;
                  tempA = (int *)&longitude;
                  double longitudef = *tempA * 1e-7L;
                  tempA = (int *)&altitude;
                  double altitudef = *tempA * 1e-3L;

                  ROS_INFO("latitude: %f, longitude: %f, altitude: %f", latitudef, longitudef, altitudef);

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

                  // ROS_INFO("northSpeed: %f, eastSpeef: %f, groundSpeed: %f",northSpeedf, eastSpeedf, groundSpeedf);

                  // 解析 32 位时间数据
                  uint32_t gpsTime =
                      ((0xff & (char)input[data_packet_start + 55]) << 24) |
                      ((0xff & (char)input[data_packet_start + 54]) << 16) |
                      ((0xff & (char)input[data_packet_start + 53]) << 8) |
                      (0xff & (char)input[data_packet_start + 52]);

                  // 计算 GPS 时间（单位：秒）
                  double gpsTimeMilliseconds = gpsTime * 0.00025;


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
                    gps_msg.position_covariance[1] = 0.0;
                    gps_msg.position_covariance[2] = 0.0;
                    gps_msg.position_covariance[3] = 0.0;
                    gps_msg.position_covariance[4] = lat_std * lat_std;
                    gps_msg.position_covariance[5] = 0.0;
                    gps_msg.position_covariance[6] = 0.0;
                    gps_msg.position_covariance[7] = 0.0;
                    gps_msg.position_covariance[8] = alt_std * alt_std;
                    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
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
                    case 0:
                      gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                      break;
                    case 16:
                      gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                      break;
                    case 18:
                      gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                      break;
                    case 17:
                    case 32:
                    case 33:
                    case 34:
                    case 48:
                    case 49:
                    case 50:
                      gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                      break;
                    default:
                      break;
                    }

                    std_msgs::UInt8 sat_msg;
                    sat_msg.data = static_cast<uint8_t>(Data2);
                    sat_pub.publish(sat_msg);
                    break;
                  }
                  default:
                    break;
                  }

                  received_message = true;

                  uint32_t gpsWeek =
                      ((0xff & (char)input[data_packet_start + 61]) << 24) |
                      ((0xff & (char)input[data_packet_start + 60]) << 16) |
                      ((0xff & (char)input[data_packet_start + 59]) << 8) |
                      (0xff & (char)input[data_packet_start + 58]);

                    ros::Time measurement_time =
                      ros::Time::now() + ros::Duration(time_offset_in_seconds);
                    if (use_gps_time && gpsWeek > 0)
                    {
                    measurement_time =
                      convertGPSTimeToROSTime((int)gpsWeek, gpsTimeMilliseconds) +
                      ros::Duration(time_offset_in_seconds);
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
                  ROS_INFO_STREAM("in degree::rollf: " << rollf * 180 / M_PI << " pitchf: " << pitchf * 180 / M_PI << " yawf: " << yawf * 180 / M_PI);
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
                input.erase(0, data_packet_start +
                                   Length); // delete everything up to and
                                            // including the processed packet
              }
              else
              {
                input.erase(0, data_packet_start +
                                   1); // delete up to false data_packet_start
                                       // character so it is not found again
              }
            }
            else
            {
              // no start character found in input, so delete everything
              input.clear();
              // ROS_INFO("clear all");
            }
          }
        }
      }
      else
      {
        // try and open the serial port
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
          ROS_ERROR_STREAM("Unable to open serial port "
                           << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if (ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort()
                                          << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port "
                       << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
