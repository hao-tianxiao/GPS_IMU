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
namespace dy_rec
{
    
    class INS570D
    {
        public:
        INS570D();
        ~INS570D();
        //函数
        void run();
        
        private:
        // 句柄
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        //变量
        serial::Serial sp;//串口
        geometry_msgs::PoseStamped initpose;//存放初始位姿
        double lat_rad_zero,lon_rad_zero,alt_zero;//存放原点弧度制经纬度
        double current_x,current_y,current_z;//存放当前在地球平面坐标系下的坐标
        double Init_theta;//车体初始偏航角
        bool init_flag;//初始化标志
        int length;//单个信息长度
        int data_packet_start;//单个信息长度
        double relativeX,relativeY,relativeZ;
        geometry_msgs::PoseStamped currentpose;
        geometry_msgs::TwistStamped imu_vel;
        int LOOP_RATE;
        long double Roll,Pitch,Yaw,Latitude,Longitude,Altitude,w_Y,w_X,w_Z,a_Y,a_X,a_Z,Ve,Vn,Vg,l;
        geometry_msgs::PoseStamped l_pose;
        std::string file_path;
        std::ofstream f;
        //发布者
        ros::Publisher gps_pub;//发布gps信息
        ros::Publisher imu_pub;//发布imu信息
        ros::Publisher gps_nav_pub;//发布gps_nav_pub信息

        ros::Publisher rear_post_pub;//发布位姿信息
        ros::Publisher vel_pub;//发布速度信息
        ros::Publisher acc_pub;//发布加速度信息
        //函数
        void initForROS();//初始化ROS
        bool init_current_pose();//初始化世界坐标系
        void publish_slam();//发布建图用信息
        void publish_control();//发布控制用信息
        void set_zero(double,double,double);//设置当前位置为地球平面坐标系原点
        void conv_llh2xyz();//将经纬度与海拔转换为xyz
        void TransformCoordinate(geometry_msgs::PoseStamped &pose);//地球平面坐标系转化到世界坐标系
        void init_open_serial();//初始化并打开串口
        void read_serial();//对串口数据进行处理
    };
}
#endif
