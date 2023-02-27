#ifndef DY_REC_H
#define DY_REC_H

#include "ros/ros.h"
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <serial/serial.h>
#include <iomanip>
#include <tf/transform_broadcaster.h>
#include <gps_msgs/gps.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
namespace daoyuan
{
    class INS570D
    {
    public:
        INS570D();
        // ~INS570D();
        // 函数
        void run();

    private:
        // 句柄
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        // 变量
        serial::Serial Serial_Port;                  // 串口
        geometry_msgs::PoseStamped ENU_zero;         // 存放ENU坐标系原点
        double lat_rad_zero, lon_rad_zero, alt_zero; // 存放经纬高原点(弧度制)
        double current_ENU_x, current_ENU_y, current_ENU_z;   // 存放当前在ENU坐标系下的坐标
        double Init_theta;                           // 车体初始偏航角
        bool init_flag;                              // 初始化标志
        int single_msg_length;                       // 单个信息长度
        int data_packet_start;                       // 单个信息起始bit
        double relativeX, relativeY, relativeZ;      // TODO:将ENU的坐标系转换到小车起点坐标系，似乎小车控制用？

        int LOOP_RATE;
        geometry_msgs::PoseStamped currentpose;
        geometry_msgs::TwistStamped imu_vel;
        long double Roll, Pitch, Yaw;   // 小车的横滚、俯仰、偏航角
        long double Latitude, Longitude, Altitude;   //RTK的纬度、经度、高度
        long double w_Y, w_X, w_Z, a_Y, a_X, a_Z;   //惯导角速度、加速度
        long double Ve, Vn, Vg;     //北向东向地向速度
        long double gps_status;     //gps状态

        // 发布者
        ros::Publisher gps_pub;     // 发布gps信息
        ros::Publisher imu_pub;     // 发布imu信息
        ros::Publisher gps_nav_pub; // 发布gps_nav_pub信息

        ros::Publisher rear_post_pub; // 发布位姿信息
        ros::Publisher vel_pub;       // 发布速度信息
        ros::Publisher acc_pub;       // 发布加速度信息
        // 函数
        void initForROS();                                          // 初始化ROS
        bool init_current_pose();                                   // 初始化世界坐标系
        void publish_gps(const Eigen::Quaterniond &rotation_quaternion);    // 发布gps信息
        void publish_imu(const Eigen::Quaterniond &rotation_quaternion);    // 发布惯导信息
        void set_zero(double, double, double);                      // 设置当前位置为地球平面坐标系原点
        void LLA2ENU();                                             // 将经纬高转换成ENU东北天坐标系
        void TransformCoordinate(geometry_msgs::PoseStamped &pose); // 地球平面坐标系转化到世界坐标系
        void init_open_serial();                                    // 初始化并打开串口
        void read_serial();                                         // 对串口数据进行处理
    };
}
#endif
