/***********************************************************************************************
*dy_rec.cpp
*文件类型:cpp/core
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

#include "dy_rec.h"
#include <cstdio>
#include <cmath>
#include <sstream>

namespace dy_rec
{
    INS570D::INS570D()
    :private_nh("~")
    ,init_flag(false)
    ,length(58)
    ,data_packet_start(0)
    ,relativeX(0)
    ,relativeY(0)
    ,relativeZ(0)
    ,LOOP_RATE(100)
    ,Roll(0)
    ,Pitch(0)
    ,Yaw(0)
    ,Latitude(0)
    ,Longitude(0)
    ,Altitude(0)
    ,w_Z(0)
    ,Ve(0)
    ,Vn(0)
    ,Vg(0)
    ,l(0)
    {
        initForROS();
        init_open_serial();
        f.open(file_path, std::ios::trunc);
    }

    INS570D::~INS570D()
    {
        f.close();
    }

    void INS570D::initForROS()
    {
        ros::param::get("~file_path",file_path);

        gps_pub = nh.advertise<gps_msgs::gps>("/gps",1);
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_correct",1);
        gps_nav_pub = nh.advertise<sensor_msgs::NavSatFix>("odometry/gpsz",1);

        rear_post_pub = nh.advertise<geometry_msgs::PoseStamped>("rear_post",10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity",10);
        acc_pub = nh.advertise<geometry_msgs::AccelStamped>("accel",10);
    }

    void INS570D::init_open_serial()
    {
        static int i = 0;
        //设置要打开的串口名称
        //因为可能会出现串口占用的问题，所以从ttyUSB0开始开串口，开成为止，直到读不到东西的串口节点即读取失败就关闭
        sp.setPort("/dev/ttyUSB"+std::to_string(i));
        std::cerr<<"/dev/ttyUSB"+std::to_string(i)<<std::endl;
        //设置串口通信的波特率
        sp.setBaudrate(230400);
        //串口设置timeout
        serial::Timeout out=serial::Timeout::simpleTimeout(1000);
        sp.setTimeout(out);
        try
        {
            //打开串口
            sp.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }
        //判断串口是否打开成功
        std::string buffer_s;
        buffer_s.reserve(length);
        bool init_header_flag=false;
        double init_time = clock();
        while(!init_header_flag)
        {
            buffer_s += sp.read(sp.available());
            if(buffer_s.find(0xBD))
                init_header_flag = true;
            double delta_time = clock() - init_time;//记录当前时间                
            if(delta_time>500)//当时间间隔大于0.5s,切换下一个串口
            {
            sp.close();
            i++;
            init_header_flag = true;
            init_open_serial();
            } 
        }
    }

    void INS570D::set_zero(double longitude, double latitude, double altitude)
    {
        lat_rad_zero = M_PI * latitude/ 180.0;//当前位置经纬度的弧度制
        lon_rad_zero= M_PI * longitude / 180.0;
        alt_zero = Altitude;
    }

    /***********************************************************************************************
    *函数名 ：    void INS570D::run()
    *函数类型：运行函数
    *函数功能描述 ：启动运行函数，从串口读入惯导数据，进行地理坐标系转化，建立惯导坐标系，发出当前位
    姿信息与可视化信息
    ***********************************************************************************************/
    void INS570D::run()
    {
        ros::Rate loop_rate(LOOP_RATE);
        while(ros::ok())
        {
            read_serial();
            loop_rate.sleep();
        }
        sp.close();
    }

    /***********************************************************************************************
    *函数名 ：    void INS570D::read_serial()
    *函数类型：输入/读入函数
    *函数功能描述 ：将串口中的消息读入,放入字符串中
    ***********************************************************************************************/

   void print8bin16(char num){
        char front = 0;
        char back = 0;
        front = (num & (15<<4) )>> 4;
        back = num &  15;
    }

    void INS570D::read_serial()
    {
        std::string buffer_s; 
        char xorcheck = 0;   
        if (sp.isOpen())
        {
            // read string from serial device
            if(sp.available())
            {
                buffer_s += sp.read(sp.available());//read all data from available
                while (buffer_s.length() >= length) // while there might be a complete package in buffer_s
                {
                    //parse for data packets
                    data_packet_start = buffer_s.find(0xBD);
                    
                    if (data_packet_start != std::string::npos)
                    {
                            if (buffer_s.find(0x0B)-data_packet_start == 2)
                            {
                                xorcheck=0;
                                for(int i=0;i<length-1;i++)  
                                {
                                    xorcheck=xorcheck^buffer_s[data_packet_start +i];//异或校验
                                }
                                if (buffer_s[data_packet_start +length-1]==xorcheck )
                                {
                                    ros::Time current_time_ = ros::Time::now();
                                    // get RPY
                                    short int roll = ((0xff &(char)buffer_s[data_packet_start + 4]) << 8) | (0xff &(char)buffer_s[data_packet_start + 3]);
                                    short int pitch = ((0xff &(char)buffer_s[data_packet_start + 6]) << 8) | (0xff &(char)buffer_s[data_packet_start + 5]);
                                    short int yaw = ((0xff &(char)buffer_s[data_packet_start + 8]) << 8) | (0xff &(char)buffer_s[data_packet_start + 7]);
                                    
                                    // calculate RPY in rad
                                    short int * temp = (short int*)& roll;
                                    Roll = (*temp)*(360.0/32768);
                                    temp = (short int*)& pitch;
                                    Pitch = (*temp)*(360.0/32768);
                                    short int * temp_yaw = (short int*)& yaw;
                                    Yaw = (*temp_yaw)*(360.0/32768);

                                    //get omega
                                    short int w_x = ((0xff &(char)buffer_s[data_packet_start + 10]) << 8) | (0xff &(char)buffer_s[data_packet_start + 9]);
                                    short int w_y = ((0xff &(char)buffer_s[data_packet_start + 12]) << 8) | (0xff &(char)buffer_s[data_packet_start + 11]);
                                    short int w_z = ((0xff &(char)buffer_s[data_packet_start + 14]) << 8) | (0xff &(char)buffer_s[data_packet_start + 13]);

                                    // calculate omega in rad/s
                                    short int * temp_z = (short int*)& w_x;
                                    w_X= (*temp_z)*(300.0/32768);
                                    temp_z = (short int*)& w_y;
                                    w_Y = (*temp_z)*(300.0/32768);
                                    temp_z = (short int*)& w_z;
                                    w_Z = (*temp_z)*(300.0/32768);

                                    // get acc
                                    short int a_x = ((0xff &(char)buffer_s[data_packet_start + 16]) << 8) | (0xff &(char)buffer_s[data_packet_start + 15]);
                                    short int a_y = ((0xff &(char)buffer_s[data_packet_start + 18]) << 8) | (0xff &(char)buffer_s[data_packet_start + 17]);
                                    short int a_z = ((0xff &(char)buffer_s[data_packet_start + 20]) << 8) | (0xff &(char)buffer_s[data_packet_start + 19]);

                                    // calculate acc in g
                                    short int * temp_a = (short int*)& a_x;
                                    a_X = (*temp_a)*(12.0/32768);
                                    temp_a = (short int*)& a_x;
                                    a_Y = (*temp_a)*(12.0/32768);
                                    temp_a = (short int*)& a_z;
                                    a_Z = (*temp_a)*(12.0/32768);

                                    // get gps values
                                    
                                    int latitude_ = (((0xff &(char)buffer_s[data_packet_start + 24]) << 24) |((0xff &(char)buffer_s[data_packet_start + 23]) << 16) |((0xff &(char)buffer_s[data_packet_start + 22]) << 8) | 0xff &(char)buffer_s[data_packet_start + 21]);
                                    int longitude_ = (((0xff &(char)buffer_s[data_packet_start + 28]) << 24) |((0xff &(char)buffer_s[data_packet_start + 27]) << 16) |((0xff &(char)buffer_s[data_packet_start + 26]) << 8) | 0xff &(char)buffer_s[data_packet_start + 25]);
                                    int altitude_ = (((0xff &(char)buffer_s[data_packet_start + 32]) << 24) |((0xff &(char)buffer_s[data_packet_start + 31]) << 16) |((0xff &(char)buffer_s[data_packet_start + 30]) << 8) | 0xff &(char)buffer_s[data_packet_start + 29]);

                                    int* tempA = (int*)& latitude_;
                                    Latitude = *tempA*1e-7L;
                                    tempA = ( int*)& longitude_;
                                    Longitude = *tempA*1e-7L;
                                    tempA = ( int*)& altitude_;
                                    Altitude = *tempA*1e-3L;
                                    
                                    short int north_vel_ = ((0xff &(char)buffer_s[data_packet_start + 34]) << 8) | (0xff &(char)buffer_s[data_packet_start + 33]);
                                    short int east_vel_ = ((0xff &(char)buffer_s[data_packet_start + 36]) << 8) | (0xff &(char)buffer_s[data_packet_start + 35]);
                                    short int ground_vel_ = ((0xff &(char)buffer_s[data_packet_start + 38]) << 8) | (0xff &(char)buffer_s[data_packet_start + 37]);
                                                
                                    short int* temp_vel = (short int*)& north_vel_;
                                    Vn = *temp_vel*100.0/32768;
                                    temp_vel = (short int*)& east_vel_;
                                    Ve = *temp_vel*100.0/32768;
                                    temp_vel = (short int*)& ground_vel_;
                                    Vg = *temp_vel*100.0/32768;

                                    l = int(0xff &(char)buffer_s[data_packet_start + 39]);

                                }
                                buffer_s.erase(0, data_packet_start + length); // delete everything up to and including the processed packet
                            }
                            else
                                buffer_s.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                    }
                else
                        buffer_s.clear(); // no start character found in buffer_s, so delete everything
                }
            }
        }
        if(init_current_pose())
        {
            publish_slam();
            publish_control();
        }
    }

    bool INS570D::init_current_pose()
    {
        // init vehicle xyz
        static int count=0,preset_num=20;
        if(count<preset_num)
        {
            if(Longitude<=0||Latitude<=0)
            {
                ROS_WARN("error msgs");
                return false;
            }
            count++;
            init_flag=true;
            set_zero(Longitude,Latitude,Altitude);
            conv_llh2xyz();
            initpose.pose.position.x = current_x;
            initpose.pose.position.y = current_y;
            initpose.pose.position.z = current_z;
            Init_theta = Yaw*M_PI/180;
            ROS_WARN("Init vehicle xyz %d %%",count*100/preset_num);
            if(count==preset_num) ROS_WARN("Init finished");
            return false;
        }
        return true;
    }

/***********************************************************************************************
*函数名: void INS570D::publish_slam()
*函数类型:功能函数/发布函数
*函数功能描述:发布slam需要的数据,这里的数据都是坐标转换前的
***********************************************************************************************/
    void INS570D::publish_slam()
    {   
        //gps_msg
        gps_msgs::gps gps_msg;

        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "world";

        //线速度(°)与加速度(m/s^2)
        gps_msg.imu.angular.x = w_X;
        gps_msg.imu.angular.y = w_Y;
        gps_msg.imu.angular.z = w_Z;

        gps_msg.imu.linear.x = a_X * 9.8062; //输出g,转换为m/s^2
        gps_msg.imu.linear.y = a_Y * 9.8062;
        gps_msg.imu.linear.z = a_Z * 9.8062;

        // 经纬高
        gps_msg.lla.x = Latitude;
        gps_msg.lla.y = Longitude ;
        gps_msg.lla.z = Altitude ;	

        //RPY
        gps_msg.euler.x = Roll ;
        gps_msg.euler.y = Pitch ;
        gps_msg.euler.z = Yaw ;

        //Quaternion    坐标系方向x前y左z上
        Eigen::Vector3d ea0(-1.0 * Yaw * M_PI / 180.0,
                            -1.0 * Pitch * M_PI / 180.0,
                            Roll * M_PI / 180.0);  
        Eigen::Matrix3d R;
        R =   Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())  
            * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())  
            * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());  

        Eigen::Quaterniond q;   
        q = R; 

        gps_msg.q_w_from_b.w = q.w();
        gps_msg.q_w_from_b.x = q.x();
        gps_msg.q_w_from_b.y = q.y();
        gps_msg.q_w_from_b.z = q.z();

        //NED velocity 
        gps_msg.vel.x = Vn ;
        gps_msg.vel.y = Ve ;
        gps_msg.vel.z = Vg ;

        //status
        gps_msg.status = l;

        //nav_msg
        sensor_msgs::NavSatFix nav_msg;

        nav_msg.header.stamp = ros::Time::now();
        nav_msg.header.frame_id = "navsat_link";

        nav_msg.status.status = -1;

        nav_msg.status.service = 1;

        nav_msg.latitude = Latitude;
        nav_msg.longitude = Longitude;
        nav_msg.altitude = Altitude;	

        // nav_msg.position_covariance[0] =1.44;
        // nav_msg.position_covariance[4] =1.44;
        // nav_msg.position_covariance[8] =5.76;
        // nav_msg.position_covariance_type = 1;

        nav_msg.position_covariance[0] =18446746124288.0;
        nav_msg.position_covariance[4] =18446746124288.0;
        nav_msg.position_covariance[8] =14062673985536.0;
        nav_msg.position_covariance_type = 2;
        
        //imu_msg
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "base_link";

        //线速度(rad)与加速度(m/s^2)
        imu_msg.angular_velocity.x = w_X * M_PI/180.0;
        imu_msg.angular_velocity.y = -1.0 * w_Y * M_PI/180.0;
        imu_msg.angular_velocity.z = -1.0 * w_Z * M_PI/180.0;

        imu_msg.linear_acceleration.x = a_X * 9.80511;
        imu_msg.linear_acceleration.y = -1.0 * a_Y * 9.80511;
        imu_msg.linear_acceleration.z = -1.0 * a_Z * 9.80511;

        imu_msg.orientation.w = q.w();
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();

        imu_msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

        // std::cout << gps_msg << std::endl;     
        gps_pub.publish(gps_msg);
        gps_nav_pub.publish(nav_msg);
        imu_pub.publish(imu_msg);
    }


/***********************************************************************************************
*函数名: void INS570D::publish_control()
*函数类型:功能函数/发布函数
*函数功能描述:进行总处理，发布相应的话题
***********************************************************************************************/
    void INS570D::publish_control()
    {   
        // 控制使用的车体坐标系和惯导默认一致(待定)
        // get current_pose
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp=ros::Time::now();
        current_pose.header.frame_id="world";
        conv_llh2xyz();
        try
        {
            // cal current_pose&transform current_pose from plane2vehicle
            relativeX = current_x-initpose.pose.position.x;
            relativeY = current_y-initpose.pose.position.y;
            if(Yaw*M_PI/180-Init_theta<-M_PI)
                relativeZ = Yaw*M_PI/180-Init_theta+2*M_PI;
            else if(Yaw*M_PI/180-Init_theta>M_PI)
                relativeZ = Yaw*M_PI/180-Init_theta-2*M_PI;
            else
                relativeZ = Yaw*M_PI/180-Init_theta;
            if(fabs(relativeX) > 10000.0 || fabs(relativeY) > 10000.0 || fabs(relativeZ) > 1000.0)
            {ROS_WARN("relative position error");return;}
            else
            {
                current_pose.pose.position.x = relativeX;
                current_pose.pose.position.y = relativeY;
                current_pose.pose.position.z = relativeZ;
            }
            TransformCoordinate(current_pose);
        }
        catch(const char* msg)
        {
            std::cerr << msg << '\n';
            abort();
        }

        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(relativeZ, ::Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0, ::Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0, ::Eigen::Vector3d::UnitX());
        current_pose.pose.orientation.x=q.x();
        current_pose.pose.orientation.y=q.y();
        current_pose.pose.orientation.z=q.z();
        current_pose.pose.orientation.w=q.w();

        //get velocity
        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp=ros::Time::now();
        velocity.header.frame_id="world";
        velocity.twist.linear.x = Ve * sin(Init_theta) + Vn * cos(Init_theta);
        velocity.twist.linear.y = Ve * cos(Init_theta) - Vn * sin(Init_theta);
        velocity.twist.linear.z = 0;
        velocity.twist.angular.x = 0;
        velocity.twist.angular.y = 0;
        velocity.twist.angular.z = w_Z;

        //get acc
        geometry_msgs::AccelStamped acc;
        acc.header.stamp=ros::Time::now();
        acc.header.frame_id="world";
        acc.accel.linear.x = a_X * 9.8062;
        acc.accel.linear.y = a_Y * 9.8062;
        acc.accel.linear.z = a_Z * 9.8062;//有重力加速度1g
        acc.accel.angular.x = 0;
        acc.accel.angular.y = 0;
        acc.accel.angular.z = 0;

        static int count = 0;
        static bool sflag = true;
        // std::cerr<<pow(l_pose.pose.position.x-current_pose.pose.position.x,2)\
            // +pow(l_pose.pose.position.y-current_pose.pose.position.y,2)<<std::endl;
        if(sflag || (!f.eof() && count>10 && sqrt(pow(l_pose.pose.position.x-current_pose.pose.position.x,2)\
            +pow(l_pose.pose.position.y-current_pose.pose.position.y,2))>1.5))
        {
            f<<double(current_pose.pose.position.x)<<" "<<double(current_pose.pose.position.y)<<" "<<\
                double(current_pose.pose.position.z)<<std::endl;
            count = 0;
            sflag = false;
            l_pose.pose.position.x = current_pose.pose.position.x;
            l_pose.pose.position.y= current_pose.pose.position.y;
        }
        count += 1;

        rear_post_pub.publish(current_pose);
        vel_pub.publish(velocity);
        acc_pub.publish(acc);
        std::cout << "111" << current_pose << std::endl << "222" << velocity << std::endl << "333" << acc  << std::endl;
    }

    void INS570D::conv_llh2xyz()
    {
        double current_lat_rad = Latitude * M_PI / 180;
        double current_lon_rad = Longitude * M_PI / 180;
        double R = 6378101.0;
        //x东y北z天
        current_x = cos(lat_rad_zero) * (current_lon_rad - lon_rad_zero) * (R+Altitude);
        current_y = (current_lat_rad - lat_rad_zero) * (R+Altitude);
        current_z = (Altitude-alt_zero);
    }

    void INS570D::TransformCoordinate(geometry_msgs::PoseStamped &pose)//所求坐标系xy转化
    {
        double x = pose.pose.position.x, y = pose.pose.position.y;
        //x前y右z地,和惯导相同,待定
        //此处假设roll和pitch可以忽略不计
        pose.pose.position.x = x*sin(Init_theta)+y*cos(Init_theta);
        pose.pose.position.y = x*cos(Init_theta)-y*sin(Init_theta);
    }

}
