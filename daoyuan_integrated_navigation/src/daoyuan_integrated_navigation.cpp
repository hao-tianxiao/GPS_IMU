/***********************************************************************************************
*文件功能描述 :INS570D组合惯性导航系统数据接收与设置写入(核心部分)
*文件主体框架:
类
--构造函数
----初始化
------ROS初始化
------串口设置初始化
------地理零点初始化
--析构函数
--运行函数
----循环
------读取串口
------数据处理
--------地理坐标系转平面坐标系
--------平面坐标系转惯导坐标系
--------发布当前位姿与速度,发布轨迹可视化,发布原始位姿数据
--------发布IMU、速度数据，供雷达消除畸变使用
***********************************************************************************************/

#include "../include/daoyuan_integrated_navigation.h"
#include <cstdio>
#include <cmath>
#include <sstream>

namespace daoyuan
{
    INS570D::INS570D()
        : private_nh("~"), init_flag(false), single_msg_length(58), data_packet_start(0), 
                           relativeX(0), relativeY(0), relativeZ(0), 
                           LOOP_RATE(100), 
                           Roll(0), Pitch(0), Yaw(0), 
                           Latitude(0), Longitude(0), Altitude(0), 
                           w_Z(0), Ve(0), Vn(0), Vg(0), gps_status(0)
    {
        initForROS();       //ROS初始化
        init_open_serial(); //打开串口
    }

    // INS570D::~INS570D()
    // {
    //     // write_path.close();
    // }

    /***********************************************************************************************
    *函数名 ：    void INS570D::run()
    *函数类型：运行函数
    *函数功能描述 ：启动运行函数，从串口读入惯导数据，进行地理坐标系转化，建立惯导坐标系，发出当前位姿信息与可视化信息
    ***********************************************************************************************/
    void INS570D::run()
    {
        ros::Rate loop_rate(LOOP_RATE);
        while (ros::ok())
        {
            read_serial();
            loop_rate.sleep();
        }
        Serial_Port.close();
    }

    void INS570D::initForROS()
    {
        gps_pub = nh.advertise<gps_msgs::gps>("/gps", 1);
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_correct", 1);
        gps_nav_pub = nh.advertise<sensor_msgs::NavSatFix>("odometry/gpsz", 1);

        rear_post_pub = nh.advertise<geometry_msgs::PoseStamped>("rear_post", 10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
        acc_pub = nh.advertise<geometry_msgs::AccelStamped>("accel", 10);
    }

    void INS570D::init_open_serial()
    {
        static int i = 0;
        // 设置要打开的串口名称
        // 因为可能会出现串口占用的问题，所以从ttyUSB0开始开串口，开成为止，直到读不到东西的串口节点即读取失败就关闭
        Serial_Port.setPort("/dev/ttyUSB" + std::to_string(i));
        std::cerr << "/dev/ttyUSB" + std::to_string(i) << std::endl;
        // 设置串口通信的波特率
        Serial_Port.setBaudrate(230400);
        // 串口设置timeout
        serial::Timeout out = serial::Timeout::simpleTimeout(1000);
        Serial_Port.setTimeout(out);
        try
        {
            // 打开串口
            Serial_Port.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }
        // 判断串口是否打开成功
        std::string buffer_s;
        buffer_s.reserve(single_msg_length);
        bool init_header_flag = false;
        double init_time = clock();
        while (!init_header_flag)
        {
            buffer_s += Serial_Port.read(Serial_Port.available());
            if (buffer_s.find(0xBD))
                init_header_flag = true;
            double delta_time = clock() - init_time; // 记录当前时间
            if (delta_time > 500)                    // 当时间间隔大于0.5s,切换下一个串口
            {
                Serial_Port.close();
                i++;
                init_header_flag = true;
                init_open_serial();
            }
        }
    }

    void INS570D::set_zero(double longitude, double latitude, double altitude)
    {
        lat_rad_zero = M_PI * latitude / 180.0;     //当前位置纬度的弧度制
        lon_rad_zero = M_PI * longitude / 180.0;    //当前位置经度的弧度制
        alt_zero = Altitude;
    }

    void print8bin16(char num)
    {
        char front = 0;
        char back = 0;
        front = (num & (15 << 4)) >> 4;
        back = num & 15;
    }

    bool INS570D::init_current_pose()
    {
        // init vehicle xyz
        static int count = 0, preset_num = 20;
        if (count < preset_num)
        {
            if (Longitude <= 0 || Latitude <= 0)
            {
                ROS_WARN("gps未初始化");
                return false;
            }
            count++;
            init_flag = true;
            set_zero(Longitude, Latitude, Altitude);
            LLA2ENU();
            ENU_zero.pose.position.x = current_ENU_x;
            ENU_zero.pose.position.y = current_ENU_y;
            ENU_zero.pose.position.z = current_ENU_z;
            Init_theta = Yaw * M_PI / 180;
            ROS_WARN("Init vehicle xyz %d %%", count * 100 / preset_num);
            if (count == preset_num)
                ROS_WARN("Init finished");
            return false;
        }
        return true;
    }

    /***********************************************************************************************
     *函数名: void INS570D::publish_imu()
     *函数类型:功能函数/发布函数
     *函数功能描述:发布惯导数据
     ***********************************************************************************************/
    void INS570D::publish_imu(const Eigen::Quaterniond &rotation_quaternion){
        // imu_msg
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "base_link";

        // 线速度(rad)与加速度(m/s^2)
        imu_msg.angular_velocity.x = w_X * M_PI / 180.0;
        imu_msg.angular_velocity.y = -1.0 * w_Y * M_PI / 180.0;
        imu_msg.angular_velocity.z = -1.0 * w_Z * M_PI / 180.0;

        imu_msg.linear_acceleration.x = a_X * 9.80511;
        imu_msg.linear_acceleration.y = -1.0 * a_Y * 9.80511;
        imu_msg.linear_acceleration.z = -1.0 * a_Z * 9.80511;

        imu_msg.orientation.w = rotation_quaternion.w();
        imu_msg.orientation.x = rotation_quaternion.x();
        imu_msg.orientation.y = rotation_quaternion.y();
        imu_msg.orientation.z = rotation_quaternion.z();

        imu_msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
        imu_pub.publish(imu_msg);
    }

    /***********************************************************************************************
     *函数名: void INS570D::publish_gps()
     *函数类型:功能函数/发布函数
     *函数功能描述:发布gps,这里的数据都是坐标转换前的
     ***********************************************************************************************/
    void INS570D::publish_gps(const Eigen::Quaterniond &rotation_quaternion)
    {
        // gps_msg
        gps_msgs::gps gps_msg;

        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "world";

        // 线速度(°)与加速度(m/s^2)
        gps_msg.imu.angular.x = w_X;
        gps_msg.imu.angular.y = w_Y;
        gps_msg.imu.angular.z = w_Z;

        gps_msg.imu.linear.x = a_X * 9.8062; // 输出g,转换为m/s^2
        gps_msg.imu.linear.y = a_Y * 9.8062;
        gps_msg.imu.linear.z = a_Z * 9.8062;

        // 经纬高
        gps_msg.lla.x = Latitude;
        gps_msg.lla.y = Longitude;
        gps_msg.lla.z = Altitude;

        // RPY
        gps_msg.euler.x = Roll;
        gps_msg.euler.y = Pitch;
        gps_msg.euler.z = Yaw;

        // // Quaternion    坐标系方向x前y左z上
        // Eigen::Vector3d eular_angle_zyx(-1.0 * Yaw * M_PI / 180.0,
        //                     -1.0 * Pitch * M_PI / 180.0,
        //                     Roll * M_PI / 180.0);
        // Eigen::Matrix3d rotation_matrix;
        // rotation_matrix = Eigen::AngleAxisd(eular_angle_zyx[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eular_angle_zyx[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eular_angle_zyx[2], Eigen::Vector3d::UnitX());

        // Eigen::Quaterniond rotation_quaternion;
        // rotation_quaternion = rotation_matrix;

        gps_msg.q_w_from_b.w = rotation_quaternion.w();
        gps_msg.q_w_from_b.x = rotation_quaternion.x();
        gps_msg.q_w_from_b.y = rotation_quaternion.y();
        gps_msg.q_w_from_b.z = rotation_quaternion.z();

        // NED velocity
        gps_msg.vel.x = Vn;
        gps_msg.vel.y = Ve;
        gps_msg.vel.z = Vg;

        // status
        gps_msg.status = gps_status;

        // nav_msg
        sensor_msgs::NavSatFix nav_msg;

        nav_msg.header.stamp = ros::Time::now();
        nav_msg.header.frame_id = "navsat_link";

        nav_msg.status.status = -1;     // 大于等于0是gps-fix
        nav_msg.status.service = 1;     // 1-GPS，2-GLONASS，4-COMPASS（北斗）

        nav_msg.latitude = Latitude;
        nav_msg.longitude = Longitude;
        nav_msg.altitude = Altitude;

        nav_msg.position_covariance[0] = 18446746124288.0;  // 是标定后得到的吗？
        nav_msg.position_covariance[4] = 18446746124288.0;  // 是标定后得到的吗？
        nav_msg.position_covariance[8] = 14062673985536.0;  // 是标定后得到的吗？
        nav_msg.position_covariance_type = 2;   // 协方差矩阵对角线已知

        gps_pub.publish(gps_msg);
        gps_nav_pub.publish(nav_msg);
    }

    /***********************************************************************************************
     *函数名:       void INS570D::LLA2ENU()
     *函数类型：
     *函数功能描述：将经纬高转换到ENU坐标系下
     ***********************************************************************************************/
    void INS570D::LLA2ENU()
    {
        double R = 6378101.0;   //地球半径
        // x东(经度) y北(纬度) z天(高度)
        current_ENU_x = cos(lat_rad_zero) * (Longitude * M_PI / 180 - lon_rad_zero) * (R + Altitude);
        current_ENU_y = (Latitude * M_PI / 180 - lat_rad_zero) * (R + Altitude);
        current_ENU_z = (Altitude - alt_zero);
    }

    /***********************************************************************************************
     *函数名:       void INS570D::TransformCoordinate()
     *函数类型：
     *函数功能描述：
     ***********************************************************************************************/
    void INS570D::TransformCoordinate(geometry_msgs::PoseStamped &pose) // 所求坐标系xy转化
    {
        double x = pose.pose.position.x, y = pose.pose.position.y;
        // x前y右z地,和惯导相同,待定
        // 此处假设roll和pitch可以忽略不计
        pose.pose.position.x = x * sin(Init_theta) + y * cos(Init_theta);
        pose.pose.position.y = x * cos(Init_theta) - y * sin(Init_theta);
    }

    /***********************************************************************************************
     *函数名 ：    void INS570D::read_serial()
     *函数类型：输入/读入函数
     *函数功能描述 ：将串口中的消息读入,放入字符串中
     ***********************************************************************************************/
    void INS570D::read_serial()
    {
        std::string buffer_s;
        char xorcheck = 0;
        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();  //定义旋转矩阵，并初始化成单位阵
        Eigen::Quaterniond rotation_quaternion = Eigen::Quaterniond::Identity();  //定义旋转四元数，并初始化

        if (Serial_Port.isOpen())
        {
            // read string from serial device
            if (Serial_Port.available())
            {
                buffer_s += Serial_Port.read(Serial_Port.available()); // read all data from available
                while (buffer_s.length() >= single_msg_length)  // while there might be a complete package in buffer_s
                {
                    // parse for data packets
                    data_packet_start = buffer_s.find(0xBD);

                    if (data_packet_start != std::string::npos)
                    {
                        if (buffer_s.find(0x0B) - data_packet_start == 2)
                        {
                            xorcheck = 0;
                            for (int i = 0; i < single_msg_length - 1; i++)
                            {
                                xorcheck = xorcheck ^ buffer_s[data_packet_start + i]; // 异或校验
                            }
                            if (buffer_s[data_packet_start + single_msg_length - 1] == xorcheck)
                            {
                                ros::Time current_time_ = ros::Time::now();
                                // get RPY
                                short int roll = ((0xff & (char)buffer_s[data_packet_start + 4]) << 8) | (0xff & (char)buffer_s[data_packet_start + 3]);
                                short int pitch = ((0xff & (char)buffer_s[data_packet_start + 6]) << 8) | (0xff & (char)buffer_s[data_packet_start + 5]);
                                short int yaw = ((0xff & (char)buffer_s[data_packet_start + 8]) << 8) | (0xff & (char)buffer_s[data_packet_start + 7]);

                                // calculate RPY in rad
                                short int *temp = (short int *)&roll;
                                Roll = (*temp) * (360.0 / 32768);
                                temp = (short int *)&pitch;
                                Pitch = (*temp) * (360.0 / 32768);
                                short int *temp_yaw = (short int *)&yaw;
                                Yaw = (*temp_yaw) * (360.0 / 32768);

                                // get omega
                                short int w_x = ((0xff & (char)buffer_s[data_packet_start + 10]) << 8) | (0xff & (char)buffer_s[data_packet_start + 9]);
                                short int w_y = ((0xff & (char)buffer_s[data_packet_start + 12]) << 8) | (0xff & (char)buffer_s[data_packet_start + 11]);
                                short int w_z = ((0xff & (char)buffer_s[data_packet_start + 14]) << 8) | (0xff & (char)buffer_s[data_packet_start + 13]);

                                // calculate omega in rad/s
                                short int *temp_z = (short int *)&w_x;
                                w_X = (*temp_z) * (300.0 / 32768);
                                temp_z = (short int *)&w_y;
                                w_Y = (*temp_z) * (300.0 / 32768);
                                temp_z = (short int *)&w_z;
                                w_Z = (*temp_z) * (300.0 / 32768);

                                // get acc
                                short int a_x = ((0xff & (char)buffer_s[data_packet_start + 16]) << 8) | (0xff & (char)buffer_s[data_packet_start + 15]);
                                short int a_y = ((0xff & (char)buffer_s[data_packet_start + 18]) << 8) | (0xff & (char)buffer_s[data_packet_start + 17]);
                                short int a_z = ((0xff & (char)buffer_s[data_packet_start + 20]) << 8) | (0xff & (char)buffer_s[data_packet_start + 19]);

                                // calculate acc in g
                                short int *temp_a = (short int *)&a_x;
                                a_X = (*temp_a) * (12.0 / 32768);
                                temp_a = (short int *)&a_x;
                                a_Y = (*temp_a) * (12.0 / 32768);
                                temp_a = (short int *)&a_z;
                                a_Z = (*temp_a) * (12.0 / 32768);

                                // get gps values

                                int latitude_ = (((0xff & (char)buffer_s[data_packet_start + 24]) << 24) | ((0xff & (char)buffer_s[data_packet_start + 23]) << 16) | ((0xff & (char)buffer_s[data_packet_start + 22]) << 8) | 0xff & (char)buffer_s[data_packet_start + 21]);
                                int longitude_ = (((0xff & (char)buffer_s[data_packet_start + 28]) << 24) | ((0xff & (char)buffer_s[data_packet_start + 27]) << 16) | ((0xff & (char)buffer_s[data_packet_start + 26]) << 8) | 0xff & (char)buffer_s[data_packet_start + 25]);
                                int altitude_ = (((0xff & (char)buffer_s[data_packet_start + 32]) << 24) | ((0xff & (char)buffer_s[data_packet_start + 31]) << 16) | ((0xff & (char)buffer_s[data_packet_start + 30]) << 8) | 0xff & (char)buffer_s[data_packet_start + 29]);

                                int *tempA = (int *)&latitude_;
                                Latitude = *tempA * 1e-7L;  //纬度
                                tempA = (int *)&longitude_;
                                Longitude = *tempA * 1e-7L; //经度
                                tempA = (int *)&altitude_;
                                Altitude = *tempA * 1e-3L;  //高度

                                short int north_vel_ = ((0xff & (char)buffer_s[data_packet_start + 34]) << 8) | (0xff & (char)buffer_s[data_packet_start + 33]);
                                short int east_vel_ = ((0xff & (char)buffer_s[data_packet_start + 36]) << 8) | (0xff & (char)buffer_s[data_packet_start + 35]);
                                short int ground_vel_ = ((0xff & (char)buffer_s[data_packet_start + 38]) << 8) | (0xff & (char)buffer_s[data_packet_start + 37]);

                                short int *temp_vel = (short int *)&north_vel_;
                                Vn = *temp_vel * 100.0 / 32768;
                                temp_vel = (short int *)&east_vel_;
                                Ve = *temp_vel * 100.0 / 32768;
                                temp_vel = (short int *)&ground_vel_;
                                Vg = *temp_vel * 100.0 / 32768;

                                gps_status = int(0xff & (char)buffer_s[data_packet_start + 39]);
                                
                                // Quaternion   坐标系方向x前y左z上
                                rotation_matrix = Eigen::AngleAxisd(-1.0 * Yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()) 
                                                * Eigen::AngleAxisd(-1.0 * Pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) 
                                                * Eigen::AngleAxisd(Roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
                                rotation_quaternion = Eigen::Quaterniond(rotation_matrix);
                            }
                            buffer_s.erase(0, data_packet_start + single_msg_length); // delete everything up to and including the processed packet
                        }
                        else
                            buffer_s.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                    }
                    else
                        buffer_s.clear(); // no start character found in buffer_s, so delete everything

                    // 发布imu
                    publish_imu(rotation_quaternion);
                    if (init_current_pose())
                    {
                        publish_gps(rotation_quaternion);
                    }
                }
            }
        }

    }

}
