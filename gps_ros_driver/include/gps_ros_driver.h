#pragma once

#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Geometry> 

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "utils.h"
#include "gps_msgs/gps.h"

using namespace GPSDriverROS ;
using namespace std;

string name, frame_id_;
static int data_length = 100;

boost::asio::serial_port* serial_port = 0;

static uint8_t data_raw[200];

ros::Publisher gps_pub_,imu_pub_,gps_nav_pub_;

static int fd_ = -1;
static uint8_t tmp[100];

gps_msgs::gps gps_msg;

sensor_msgs::Imu imu_msg;

sensor_msgs::NavSatFix nav_msg;

static int rtk_gps_num = 63;

