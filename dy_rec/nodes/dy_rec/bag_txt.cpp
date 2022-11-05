#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <iostream>

#include<sstream>
#include<string>
#include <unistd.h>
using namespace std;



void record_path(const geometry_msgs::PoseStamped::ConstPtr current_pose){
    //   double raw, pitch, theta;
    //  tf::Quaternion q;
    //  tf::quaternionMsgToTF(current_pose->pose.position.x, q);
    //  tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
     fp_s = fopen("/home/hao/imu_ws_jyk/src/dy_rec/nodes/12345.txt","a");
      fprintf(fp_s, "%lf %lf", current_pose->pose.position.x, current_pose->pose.position.y);                   // Angle
      fprintf(fp_s, "\r\n");  
     fclose(fp_s);
}

// void nodeStart(int argc, char **argv){

// }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "record_path");
    ros::NodeHandle nc;
    // 订阅相关节点
    //this->control_timer_ = nc.createTimer(ros::Duration(0.01), &pure_pursuit::ControlCallBack, this);
    // ros::Subscriber sub_multi_path = nc.subscribe("multi_path", 1, &pure_pursuit::multipathGetCallBack, this);
//  ros::Subscriber sub_odom = nc.subscribe("/aft_mapped_to_init", 1, &pure_pursuit::control_odometryGetCallBack, this);
    ros::Subscriber sub_odom = nc.subscribe("/rear_post", 1, &record_path);//录制轨迹话题
    // 发布相关节点
    ros::spin();
    return(0);
}
