#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <iostream>
#include <string>

#include "ins/ASENSING.h"
using namespace std;
int main(int argc, char** argv) {
  serial::Serial ser;
  std::string port;
  std::string frame_id;
  bool received_message = false;
  ros::init(argc, argv, "INS");
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<std::string>("frame_id", frame_id, "base_link");
  ros::NodeHandle nh("INS");
  ros::Publisher INS_pub = nh.advertise<ins::ASENSING>("ASENSING_INS", 500);  //导远自定义的所有信息发布者
  // TODO1:这两个发布者发布的信息都一样，只是参考坐标系名字不一样？
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_correct", 500); //坐标系为"base_link" 坐标系和激光雷达一致
  ros::Publisher imu_pub1 = nh.advertise<sensor_msgs::Imu>("/imu_raw", 500);  //坐标系为"imu_link" 坐标系和激光雷达一致
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_NavSatFix", 500); 
  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("/odometry_gpsins", 200);

  ofstream debug_file1;
  ofstream debug_file2;
  debug_file1.open(
      "/home/zh/catkin_ws/src/ASENSING_INS_Driver_V1.02/data_test/imu.txt",
      ios::out | ios::app);
  debug_file1.clear();
  debug_file2.open(
      "/home/zh/catkin_ws/src/ASENSING_INS_Driver_V1.02/data_test/gps.txt",
      ios::out | ios::app);
  debug_file2.clear();
  ros::Rate r(100);

  std::string input;
  std::string read;
  double long i = 0;
  double gpswf;
  float latstd, lonstd, hstd, vnstd, vestd, vdstd, pitchstd, yawstd, rollstd,
      temperature, wheel_speed_status, numsv, position_type, heading_type;
  double trans_geo = 0.017453293;  // 1度=0.017453293弧度
  double trans_aug = 9.8;
  int Length = 58;

  while (ros::ok()) {
  try {
  if (ser.isOpen()) {
  if (ser.available()) {
    read = ser.read(ser.available());
    ROS_DEBUG(
      "read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
      input += read;
      while (input.size() >= 3) {
        if (((input[0] & 0xff) == 0xbd) && ((input[1] & 0xff) == 0xdb) &&
            ((input[2] & 0xff) == 0x0b)) {
          if (input.size() < 63) {
            break;
          }
          char xorcheck58 = 0;
          for (int i = 0; i < Length - 1; i++) {
            xorcheck58 = xorcheck58 ^ input[i];
          }
          if (input[Length - 1] == xorcheck58) {
            // get RPY
            short int roll =
                ((0xff & (char)input[4]) << 8) | (0xff & (char)input[3]);
            short int pitch =
                ((0xff & (char)input[6]) << 8) | (0xff & (char)input[5]);
            short int yaw =
                ((0xff & (char)input[8]) << 8) | (0xff & (char)input[7]);
              // calculate RPY in deg
                short int* temp = (short int*)&roll;
                float rollf = (*temp) * (360.0 / 32768);
                temp = (short int*)&pitch;
                float pitchf = (*temp) * (360.0 / 32768);
                temp = (short int*)&yaw;
                float yawf = (*temp) * (360.0 / 32768);

                // get gyro values
                short int gx =
                    ((0xff & (char)input[10]) << 8) | (0xff & (char)input[9]);
                short int gy =
                    ((0xff & (char)input[12]) << 8) | (0xff & (char)input[11]);
                short int gz =
                    ((0xff & (char)input[14]) << 8) | (0xff & (char)input[13]);
                // calculate gyro in deg/s
                temp = (short int*)&gx;
                float gxf = (*temp) * 300.0 / 32768;
                temp = (short int*)&gy;
                float gyf = (*temp) * 300.0 / 32768;
                temp = (short int*)&gz;
                float gzf = (*temp) * 300.0 / 32768;

                // get acelerometer values
                short int ax =
                    ((0xff & (char)input[16]) << 8) | (0xff & (char)input[15]);
                short int ay =
                    ((0xff & (char)input[18]) << 8) | (0xff & (char)input[17]);
                short int az =
                    ((0xff & (char)input[20]) << 8) | (0xff & (char)input[19]);
                // calculate acelerometer in g
                temp = (short int*)&ax;
                float axf = *temp * 12.0 / 32768;
                temp = (short int*)&ay;
                float ayf = *temp * 12.0 / 32768;
                temp = (short int*)&az;
                float azf = *temp * 12.0 / 32768;

                // get gps values
                int latitude =
                    (((0xff & (char)input[24]) << 24) |
                     ((0xff & (char)input[23]) << 16) |
                     ((0xff & (char)input[22]) << 8) | 0xff & (char)input[21]);
                int longitude =
                    (((0xff & (char)input[28]) << 24) |
                     ((0xff & (char)input[27]) << 16) |
                     ((0xff & (char)input[26]) << 8) | 0xff & (char)input[25]);
                int altitude =
                    (((0xff & (char)input[32]) << 24) |
                     ((0xff & (char)input[31]) << 16) |
                     ((0xff & (char)input[30]) << 8) | 0xff & (char)input[29]);
                // calculate lat、lon in deg(WGS84)
                int* tempA = (int*)&latitude;
                double latitudef = *tempA * 1e-7L;
                tempA = (int*)&longitude;
                double longitudef = *tempA * 1e-7L;
                // calculate lat、lon in m
                tempA = (int*)&altitude;
                double altitudef = *tempA * 1e-3L;

                // get  NED vel values
                short int Nvel =
                    ((0xff & (char)input[34]) << 8) | (0xff & (char)input[33]);
                short int Evel =
                    ((0xff & (char)input[36]) << 8) | (0xff & (char)input[35]);
                short int Dvel =
                    ((0xff & (char)input[38]) << 8) | (0xff & (char)input[37]);
                // calculate NED vel in m/s
                temp = (short int*)&Nvel;
                float Nvelf = (*temp) * (100.0 / 32768);
                temp = (short int*)&Evel;
                float Evelf = (*temp) * (100.0 / 32768);
                temp = (short int*)&Dvel;
                float Dvelf = (*temp) * (100.0 / 32768);

                // ins_status values
                short int ins_status = (0x0f & (char)input[39]);
                temp = (short int*)&ins_status;
                double ins_statusf = *temp * 1;

                // pdata_type
                short int pdata_type = (0xff & (char)input[56]);

                // data 1-3
                short int data1 =
                    ((0xff & (char)input[47]) << 8) | (0xff & (char)input[46]);

                temp = (short int*)&data1;
                float data1f = (*temp) * 1;
                short int data2 =
                    ((0xff & (char)input[49]) << 8) | (0xff & (char)input[48]);
                temp = (short int*)&data2;
                float data2f = (*temp) * 1;
                short int data3 =
                    ((0xff & (char)input[51]) << 8) | (0xff & (char)input[50]);
                temp = (short int*)&data3;
                float data3f = (*temp) * 1;

                switch (pdata_type) {
                  case 0:
                    latstd = exp(data1f / 100);
                    lonstd = exp(data2f / 100);
                    hstd = exp(data3f / 100);
                    break;

                  case 1:
                    vnstd = exp(data1f / 100);
                    vestd = exp(data2f / 100);
                    vdstd = exp(data3f / 100);
                    break;

                  case 2:
                    rollstd = exp(data1f / 100);
                    pitchstd = exp(data2f / 100);
                    yawstd = exp(data3f / 100);
                    break;

                  case 22:
                    temperature = (data1f)*200.0 / 32768;
                    break;

                  case 32:
                    position_type = data1f;
                    numsv = data2f;
                    heading_type = data3f;
                    break;

                  case 33:
                    wheel_speed_status = data2f;
                    break;
                }

                // get gps time values
                uint32_t gpst =
                    (((0xff & (char)input[55]) << 24) |
                     ((0xff & (char)input[54]) << 16) |
                     ((0xff & (char)input[53]) << 8) | 0xff & (char)input[52]);
                // calculate gps time in ms
                double gpstf = gpst * 0.25;

                char xorcheck63 = 0;
                for (int i = 0; i < 62; i++) {
                  xorcheck63 = xorcheck63 ^ input[i];
                }
                if (input[62] == xorcheck63) {
                  // get gps week values
                  short int gpsw = (((0xff & (char)input[61]) << 24) |
                                    ((0xff & (char)input[60]) << 16) |
                                    ((0xff & (char)input[59]) << 8) |
                                    0xff & (char)input[58]);
                  // calculate gps week
                  temp = (short int*)&gpsw;
                  gpswf = *temp * 1;
                }
                received_message = true;
                ins::ASENSING Asensing_msg;
                Asensing_msg.latitude = latitudef;
                Asensing_msg.longitude = longitudef;
                Asensing_msg.altitude = altitudef;

                Asensing_msg.north_velocity = Nvelf;
                Asensing_msg.east_velocity = Evelf;
                Asensing_msg.ground_velocity = Dvelf;

                Asensing_msg.roll = rollf;
                Asensing_msg.pitch = pitchf;
                Asensing_msg.azimuth = yawf;

                Asensing_msg.x_angular_velocity = gxf;
                Asensing_msg.y_angular_velocity = gyf;
                Asensing_msg.z_angular_velocity = gzf;

                Asensing_msg.x_acc = axf;
                Asensing_msg.y_acc = ayf;
                Asensing_msg.z_acc = azf;

                Asensing_msg.latitude_std = latstd;
                Asensing_msg.longitude_std = lonstd;
                Asensing_msg.altitude_std = hstd;

                Asensing_msg.north_velocity_std = vnstd;
                Asensing_msg.east_velocity_std = vestd;
                Asensing_msg.ground_velocity_std = vdstd;

                Asensing_msg.roll_std = rollstd;
                Asensing_msg.pitch_std = pitchstd;
                Asensing_msg.azimuth_std = yawstd;

                Asensing_msg.sec_of_week = gpstf;
                Asensing_msg.gps_week_number = gpswf;

                Asensing_msg.temperature = temperature;
                Asensing_msg.wheel_speed_status = wheel_speed_status;
                Asensing_msg.ins_status = ins_statusf;
                Asensing_msg.position_type = position_type;
                Asensing_msg.heading_type = heading_type;
                Asensing_msg.numsv = numsv;
                INS_pub.publish(Asensing_msg);

                debug_file1 << setiosflags(ios::fixed) << setprecision(6)
                            << ros::Time::now() << " " << position_type;
                debug_file1 << "\n";
                debug_file2 << setiosflags(ios::fixed) << setprecision(6)
                            << ros::Time::now() << " " << numsv;
                debug_file2 << "\n";

                // publish imudata
                /* imu坐标系变换
                 1.把imu原始的前右下转成前左上
                 2.单位换算
                  deg/s ----------> rad/s 角速度
                  g---------------> m/s^2 加速度
                  deg-------------> rad  角度*/
                sensor_msgs::Imu imu_data;
                imu_data.header.stamp = ros::Time::now();
                imu_data.header.frame_id = "base_link";
                sensor_msgs::Imu imu_data1;
                imu_data1.header.stamp = ros::Time::now();
                tf2::Quaternion myQuaternion;
                myQuaternion.setRPY(rollf * trans_geo,
                                    -pitchf * trans_geo,
                                    -yawf * trans_geo);
                imu_data.orientation = tf2::toMsg(myQuaternion);
                imu_data.angular_velocity.x = gxf * trans_geo;
                imu_data.angular_velocity.y = -gyf * trans_geo;
                imu_data.angular_velocity.z = -gzf * trans_geo;
                imu_data.linear_acceleration.x = axf * trans_aug;
                imu_data.linear_acceleration.y = -ayf * trans_aug;
                imu_data.linear_acceleration.z = -azf * trans_aug;
                imu_data.orientation_covariance[0] = pow(0.1, 2);
                imu_data.orientation_covariance[4] = pow(0.1, 2);
                imu_data.orientation_covariance[8] = pow(0.1, 2);
                imu_data.angular_velocity_covariance[0] = pow(0.1, 2);// 待标定
                imu_data.angular_velocity_covariance[4] = pow(0.1, 2);// 待标定
                imu_data.angular_velocity_covariance[8] = pow(0.1, 2);// 待标定
                imu_data.linear_acceleration_covariance[0] = pow(0.1, 2);// 待标定
                imu_data.linear_acceleration_covariance[4] = pow(0.1, 2);
                imu_data.linear_acceleration_covariance[8] = pow(0.1, 2);

                imu_pub.publish(imu_data);

                imu_data1.header.frame_id = "imu_link";
                tf2::Quaternion myQuaternion1;
                myQuaternion1.setRPY(rollf * trans_geo,
                                     -pitchf * trans_geo,
                                     -yawf * trans_geo);
                imu_data1.orientation = tf2::toMsg(myQuaternion1);
                imu_data1.angular_velocity.x = gxf * trans_geo;
                imu_data1.angular_velocity.y = -gyf * trans_geo;
                imu_data1.angular_velocity.z = -gzf * trans_geo;
                imu_data1.linear_acceleration.x = axf * trans_aug;
                imu_data1.linear_acceleration.y = -ayf * trans_aug;
                imu_data1.linear_acceleration.z = -azf * trans_aug;
                imu_data1.orientation_covariance[0] = pow(0.1, 2);
                imu_data1.orientation_covariance[4] = pow(0.1, 2);
                imu_data1.orientation_covariance[8] = pow(0.1, 2);
                imu_data1.angular_velocity_covariance[0] = pow(0.1, 2);// 待标定
                imu_data1.angular_velocity_covariance[4] = pow(0.1, 2);// 待标定
                imu_data1.angular_velocity_covariance[8] = pow(0.1, 2);// 待标定
                imu_data1.linear_acceleration_covariance[0] = pow(0.1, 2);// 待标定
                imu_data1.linear_acceleration_covariance[4] = pow(0.1, 2);
                imu_data1.linear_acceleration_covariance[8] = pow(0.1, 2);

                imu_pub1.publish(imu_data1);

                sensor_msgs::NavSatFix gps_data;
                gps_data.header.stamp = ros::Time::now();
                gps_data.header.frame_id = "navsat_link";
                gps_data.latitude = latitudef;
                gps_data.longitude = longitudef;
                gps_data.altitude = altitudef;
                gps_data.status.status = ins_statusf;
                gps_data.position_covariance_type = position_type;
                gps_data.position_covariance[0] = pow(latstd, 2);
                gps_data.position_covariance[4] = pow(lonstd, 2);
                gps_data.position_covariance[8] = pow(hstd, 2);
                gps_pub.publish(gps_data);

                /* //since all odometry is 6DOF we'll need a quaternion created
                 from yaw geometry_msgs::Quaternion odom_quat=
                 tf::createQuaternionMsgFromYaw(yawf);
                 geometry_msgs::TransformStamped odom_trans;
                 odom_trans.header.stamp = current_time;
                 odom_trans.header.frame_id = "/map";
                 odom_trans.child_frame_id = "/gps";
                 odom_trans.transform.translation.x = latitudef;
                 odom_trans.transform.translation.y = longitudef;
                 odom_trans.transform.translation.z = altitudef;
                 odom_trans.transform.rotation = odom_quat;
                 //send the transform
                 odom_broadcaster.sendTransform(odom_trans);

                 //next, we'll publish the odometry message over ROS
                 nav_msgs::Odometry odom;
                 odom.header.stamp = ros::Time::now();
                 odom.header.frame_id = "/map";

                 //set the position
                 odom.pose.pose.position.x = x;
                 odom.pose.pose.position.y = y;
                 odom.pose.pose.position.z = 0.0;
                 odom.pose.pose.orientation = odom_quat;

                 //set the velocity
                 odom.child_frame_id = "/gps";
                 odom.twist.twist.linear.x = vx;
                 odom.twist.twist.linear.y = vy;
                 odom.twist.twist.angular.z = vth;

                 //publish the message
                 odom_pub.publish(odom);*/

                // publish odometry
                nav_msgs::Odometry odom_ins_;
                odom_ins_.header.stamp = ros::Time::now();  // TODO: 用pps
                odom_ins_.header.frame_id = "map";         // 参考坐标系，表示起始坐标系
                odom_ins_.child_frame_id = "gps";          // 目标坐标系，表示当前所在的位置和方向

                odom_ins_.pose.pose.position.x = latitudef;
                odom_ins_.pose.pose.position.y = longitudef;
                odom_ins_.pose.pose.position.z = altitudef;

                // printf("%f %f %f \n",ins_.roll,ins_.pitch,ins_.yaw);
                myQuaternion.setRPY(rollf, pitchf, yawf);
                odom_ins_.pose.pose.orientation = tf2::toMsg(myQuaternion);

                odom_ins_.pose.covariance[0] = pow(lonstd, 2);
                odom_ins_.pose.covariance[7] = pow(latstd, 2);
                odom_ins_.pose.covariance[14] = pow(hstd, 2);
                odom_ins_.pose.covariance[21] = pow(rollstd, 2);
                odom_ins_.pose.covariance[28] = pow(pitchstd, 2);
                odom_ins_.pose.covariance[35] = pow(yawstd, 2);

                odometry_pub.publish(odom_ins_);
              }
              input.erase(0, Length);
            } else {
              input.erase(0, 1);
            }
          }
        }
      }

      else {
        try {
          ser.setPort(port);
          ser.setBaudrate(230400);  // 115200  230400
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        } catch (serial::IOException& e) {
          ROS_ERROR_STREAM("Unable to open serial port "
                           << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if (ser.isOpen()) {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort()
                                          << " initialized and opened.");
        }
      }
    } catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Error reading from the serial port "
                       << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }

  debug_file1.close();
  debug_file2.close();
}
