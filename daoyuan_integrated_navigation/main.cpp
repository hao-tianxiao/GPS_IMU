/***********************************************************************************************
*dy_rec.cpp
*文件类型:cpp/core
*文件功能描述 :INS570D组合惯性导航系统数据接收与设置写入(启动部分)
***********************************************************************************************/

#include "include/daoyuan_integrated_navigation.h"

int main(int argc,char**argv)
{
    ros::init(argc,argv,"INS570D");
    setlocale(LC_ALL, "");
    daoyuan::INS570D INS;
    INS.run();
    return 0;
}