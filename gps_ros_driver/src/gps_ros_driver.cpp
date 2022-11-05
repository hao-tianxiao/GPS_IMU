#include "gps_ros_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_driver");
  ros::NodeHandle nh("~");
  
  //aquire the name of INSD_odom
  name = ros::this_node::getName();
  
  std::string port;
  if (nh.hasParam("port"))
    nh.getParam("port", port);
  else
    {
      ROS_ERROR("%s: must provide a port", name.c_str());
      return -1;
    }   
  
  //defalt baudrate 230400 , sample period : 100Hz
  int baud;
  if (nh.hasParam("baud"))
    nh.getParam("baud", baud);
  else
    {
      ROS_ERROR("%s: must provide a baudrate", name.c_str());
      return -1;
    }
  
  ROS_WARN("Baudrate set to %d", baud);

  if (nh.hasParam("frame_id"))
    nh.getParam("frame_id", frame_id_);
  else
    {
      ROS_ERROR("%s: must provide frame_id", name.c_str());
      return -1;
    }

  double delay;
  if (nh.hasParam("delay"))
    nh.getParam("delay", delay);
  else
    {
       nh.param("delay", delay, 0.0);
       ROS_ERROR("%s: delay is 0.0 . ", name.c_str());
    }
  
  boost::asio::io_service io_service;
  serial_port = new boost::asio::serial_port(io_service);
  try
  {
    serial_port->open(port);
  } 
  catch (boost::system::system_error &error)
  {
    ROS_ERROR(" Failed to open port %s with error %s", port.c_str(), error.what());
    return -1;
  }
  
  if (!serial_port->is_open())
  {
    ROS_ERROR("%s: failed to open serial port %s",name.c_str(), port.c_str());
    return -1;
  }
  
  typedef boost::asio::serial_port_base sb;
  
  sb::baud_rate baud_option(baud);
  sb::flow_control flow_control(sb::flow_control::none);
  sb::parity parity(sb::parity::none);
  sb::stop_bits stop_bits(sb::stop_bits::one);
  
  serial_port->set_option(baud_option);
  serial_port->set_option(flow_control);
  serial_port->set_option(parity);
  serial_port->set_option(stop_bits);
 
  const char *path = port.c_str();
  fd_ = open(path, O_RDWR);
  if(fd_ < 0)
  {    
      ROS_ERROR("Port Error!: %s", path);
      return -1;
  }  

  gps_pub_ = nh.advertise<gps_msgs::gps>("/gps",1);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_correct",1);
  gps_nav_pub_ = nh.advertise<sensor_msgs::NavSatFix>("odometry/gpsz",1);

  usleep(1000 * 1000);
  data_length = 500;

  int kk = 0;
  ROS_WARN("Streaming Data...");
  
  while (nh.ok())
  {
    ssize_t a = read(fd_, tmp, sizeof(uint8_t) * data_length);

    memcpy(data_raw, tmp, sizeof(uint8_t) * data_length);

    bool found = false;
    for(kk = 0; kk < data_length - 1; ++kk)
    {
      
      if(data_raw[kk] == 0xBD && data_raw[kk + 1] == 0xDB && data_raw[kk + 2] == 0x0B )
      { 
        if(kk >= 435)
        {
          // ROS_WARN("Not enough space to solve the rtk gps raw data.");
          continue;
        }

        else{
        unsigned char *data = data_raw + kk;

          // xor check
          unsigned char check_out;
          check_out=0x00; 

          for(int ii=0; ii < rtk_gps_num -1 ; ii++)
          {
            check_out ^= data[ii]; 
          }
          // std::cout<<"check_out result : " << hex << (unsigned int)check_out <<std::endl;
          // std::cout<<"rtk gps number 63 bit : " << hex << (unsigned int)(data[rtk_gps_num -1] )<<std::endl;

          if(check_out == data[rtk_gps_num -1])
          { 
            // std::cout<< "check out sucess." << std::endl;

            gps_msg.header.stamp = ros::Time::now();
            gps_msg.header.frame_id = frame_id_;

            // std::cout<< "gyro_x : "   <<  i2d_gyro(data + 9) << std::endl;
            // std::cout<< "gyro_y : "   <<  i2d_gyro(data + 11) << std::endl;
            // std::cout<< "gyro_z : "   <<  i2d_gyro(data + 13) <<std::endl; 

            // std::cout<< "acc_x : "   <<  i2d_acc(data + 15) << std::endl;
            // std::cout<< "acc_y : "   <<  i2d_acc(data + 17) << std::endl;
            // std::cout<< "acc_z : "   <<  i2d_acc(data + 19) <<std::endl; 

            gps_msg.imu.angular.x = i2d_gyro(data + 9);
            gps_msg.imu.angular.y = i2d_gyro(data + 11);
            gps_msg.imu.angular.z = i2d_gyro(data + 13);

            gps_msg.imu.linear.x = i2d_acc(data + 15) * 9.8062;
            gps_msg.imu.linear.y = i2d_acc(data + 17) * 9.8062;
            gps_msg.imu.linear.z = i2d_acc(data + 19) * 9.8062;

            // latitude, longitude, altitude.   

            gps_msg.lla.x = i2d_latlon(data + 21) ;
            gps_msg.lla.y = i2d_latlon(data + 25) ;
            gps_msg.lla.z = i2d_alt(data + 29) ;	

            {
              nav_msg.header.stamp = ros::Time::now();
              nav_msg.header.frame_id = "navsat_link";

              // # 当状态 >= STATUS_FIX 时，此定位才有效。
              // int8 STATUS_NO_FIX =  -1        # unable to fix position 不能定位
              // int8 STATUS_FIX =      0        # unaugmented fix        未增强的定位
              // int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation　
              // int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation   

              nav_msg.status.status = -1;

              nav_msg.status.service = 1;

              nav_msg.latitude = i2d_latlon(data + 21) ;
              nav_msg.longitude = i2d_latlon(data + 25) ;
              nav_msg.altitude = i2d_alt(data + 29) ;	

              // nav_msg.position_covariance[0] =1.44;
              // nav_msg.position_covariance[4] =1.44;
              // nav_msg.position_covariance[8] =5.76;
              // nav_msg.position_covariance_type = 1;

              nav_msg.position_covariance[0] =18446746124288.0;
              nav_msg.position_covariance[4] =18446746124288.0;
              nav_msg.position_covariance[8] =14062673985536.0;
              nav_msg.position_covariance_type = 2;
              

              gps_nav_pub_.publish(nav_msg);
            }
            
            // std::cout<< "latitude : "   <<  i2d_latlon(data + 21) << std::endl;
            // std::cout<< "longitude : "  <<  i2d_latlon(data + 25) << std::endl;
            // std::cout<< "altitude : "   <<  i2d_alt(data + 29)    <<std::endl; 
            
            //Euler Angle
            double yaw,pitch,roll;
            roll  = i2d_euler(data + 3);
            pitch = i2d_euler(data + 5);
            yaw   = i2d_euler(data + 7);

            gps_msg.euler.x = roll ;
            gps_msg.euler.y = pitch ;
            gps_msg.euler.z = yaw ;

            // std::cout<< "roll : "  << roll <<std::endl;
            // std::cout<< "pitch : " << pitch  << std::endl;
            // std::cout<< "yaw : "   << yaw  << std::endl;

            //Quaternion    
            Eigen::Vector3d ea0(-1.0 * yaw * M_PI / 180.0,
                                -1.0 * pitch * M_PI / 180.0,
                                roll * M_PI / 180.0);  
            Eigen::Matrix3d R;  
            R =   Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())  
                * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())  
                * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());  
            
            // R =   Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX())  
            //     * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())  
            //     * Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()); 

            Eigen::Quaterniond q;   
            q = R; 

            gps_msg.q_w_from_b.w = q.w();
            gps_msg.q_w_from_b.x = q.x();
            gps_msg.q_w_from_b.y = q.y();
            gps_msg.q_w_from_b.z = q.z();

            {
              imu_msg.header.stamp = ros::Time::now();
              imu_msg.header.frame_id = "base_link";

              imu_msg.angular_velocity.x = i2d_gyro(data + 9) * M_PI/180.0;
              imu_msg.angular_velocity.y = -1.0 * i2d_gyro(data + 11) * M_PI/180.0;
              imu_msg.angular_velocity.z = -1.0 * i2d_gyro(data + 13) * M_PI/180.0;

              imu_msg.linear_acceleration.x = i2d_acc(data + 15) * 9.80511;
              imu_msg.linear_acceleration.y = -1.0 * i2d_acc(data + 17) * 9.80511;
              imu_msg.linear_acceleration.z = -1.0 * i2d_acc(data + 19) * 9.80511;

              imu_msg.orientation.w = q.w();
              imu_msg.orientation.x = q.x();
              imu_msg.orientation.y = q.y();
              imu_msg.orientation.z = q.z();

              imu_msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

              imu_pub_.publish(imu_msg);
            }
                
            //NED velocity 
            gps_msg.vel.x = i2d_vel(data + 33) ;
            gps_msg.vel.y = i2d_vel(data + 35) ;
            gps_msg.vel.z = i2d_vel(data + 37) ;

            // std::cout<< "vel_N : "  << i2d_vel(data + 33) << std::endl;
            // std::cout<< "vel_E : "  << i2d_vel(data + 35) << std::endl;
            // std::cout<< "vel_D : "  << i2d_vel(data + 37) <<std::endl; 

            gps_msg.status = i2i_status(data + 39);
            // std::cout<<"gps_status ： " << i2i_status(data + 39) << std::endl;
            // std::cout<< "-----------------------------------"<<std::endl;

            gps_pub_.publish(gps_msg);

            found = true;
          }
        }
      }
    
    } 
  }  
  
  // Stop continous and close device
  ROS_WARN("Wait 0.01s"); 
  ros::Duration(0.01).sleep();
  close(fd_);
  
  return 0;
}
