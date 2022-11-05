# 简介
这是惯性导航系统接收与消息发布模块.
    主要完成了
*   接收惯性导航系统(dy-INS570D)信息并发布当前位姿

# 依赖
*   操作系统 ubuntu 18.04LTS
*   ROS melodic
*   cmake 3.17.0
*   make 4.1
*   gcc/g++ 7.5.0
*   pack serial

# 项目的构建与运行
## 依赖
    sudo apt-get install ros-melodic-serial
## 构建
    cp -r ${PROJECT_SOURCE_DIR} ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make -DCATKIN_WHITELIST_PACKAGES="xw_rec"
## 运行
未设置串口权限
```
cd /dev/
sudo chmod 666 ttyUSB0
roslaunch xw_rec xw_rec.launch
```
或
```
cd ~/catkin_ws/src/xw_rec
sh executable.sh
```
若设置完毕串口权限可直接
```
roslaunch xw_rec xw_rec.launch
```
## 设置串口权限的方法
```
sudo gedit /etc/udev/rules.d/70-ttyusb.rules
#在文件内输入一行
KERNEL=="ttyUSB[0-9]*",MODE="0666"
```
# 输入与输出

## 输入
惯导接口
    
## 输出
```
    topic:/current_pose msg:geometry_msgs/PoseStamped 
    information:以车体原点(启动惯导时)建系，xy轴见总览图片
         pose.position.x与pose.position.y为惯导坐标系xy坐标
         pose.position.z为惯导坐标系下车体航向角
         pose.oritention.xyz没用
         pose.oritention.w为速度
    topic:/vis_pose msg:geometry_msgs/PoseStamped
    information:车体位姿可视化/未转换信息即原始数据
    topic:/vis_path msg:nag_msgs/Path
    information:轨迹可视化
    topic:/imu_vel msg:geometry_msgs/TwistStamped
    information:imu与速度信息，屏蔽了Z方向速度与RP方向角速度
```
# 注意
惯导在收到resgo后开始建系，每次都要重开节点以重新建系

# 隐疾与改进
> 570D的单点输出是没有经纬度的，且直接通过serial读出来的是乱码

> 误差经过测试大概在3-10cm

# 作者
HRT21D-gzy
