# 导远组合导航（发布带有带有协方差的组合导航数据）
## 1、话题列表
```bash
/imu_correct     # sensor_msgs::Imu, 坐标系为"base_link" 坐标系和激光雷达一致；
/imu_raw         # sensor_msgs::Imu, 坐标系为"imu_link" 坐标系和激光雷达一致；
/gps_NavSatFix   # sensor_msgs::NavSatFix,坐标系为:navsat_link；
/odometry_gpsins # sensor_msgs::Odometry,参考坐标系/map，起点；目标坐标系/gps，当前位置和方向；
```
## 2、安装依赖
```
换成自己的ros版本
sudo apt install ros-melodic-serial
```
## 2、编译
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="ins"
```
## 3、运行
```bash
sudo chmod 777 /dev/ttyUSB0
roslaunch ins demo.launch
```

