# 导远电子的组合导航
## 1、编译 gps_ros_driver功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES=“gps_ros_driver”
## 2、编译 gps_msgs功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES="gps_msgs"
## 3、编译 daoyuan_integrated_navigation功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES="daoyuan_integrated_navigation"

## 4、运行组合导航节点
sudo chmod 777 /dev/ttyUSB0
roslaunch daoyuan_integrated_navigation daoyuan_integrated_navigation.launch



# 简介
这是惯性导航系统接收与消息发布模块.
    主要完成了
*   接收惯性导航系统(dy-INS570D)信息并发布当前位姿

## 依赖
    sudo apt install ros-melodic-serial


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
570D的单点输出是没有经纬度的，且直接通过serial读出来的是乱码
误差经过测试大概在3-10cm
# 作者
HRT21D-gzy
