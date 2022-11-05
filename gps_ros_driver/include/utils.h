#pragma once

namespace GPSDriverROS{

//unit : deg
static inline double i2d_euler(uint8_t a[2])
{
    int16_t euler = a[1];
    euler = (euler << 8) | a[0];
    
    return ((double) euler) * 360.0/32768.0;
}

//unit : deg/s
static inline double i2d_gyro(uint8_t a[2])
{
    int16_t gyro = a[1];
    gyro = (gyro << 8) | a[0];
    
    return ((double) gyro) * 300.0/32768.0;
}

//unit : g
static inline double i2d_acc(uint8_t a[2])
{
    int16_t acc = a[1];
    acc = (acc << 8) | a[0];
    
    return ((double) acc) * 12.0/32768.0;
    // return ((double) acc) * 12.0/32768.0;    //datasheet use 12.0/32768.0;
}

//unit : deg
static inline double i2d_latlon(uint8_t a[4])
{
    int64_t high = a[3];
    high = (high << 8) | a[2];

    int64_t low = a[1];
    low = (low << 8) | a[0];
    return ((double)((high << 16) | low)) * 1E-7 ;
}

//unit : m
static inline double i2d_alt(uint8_t a[4])
{
    int64_t high = a[3];
    high = (high << 8) | a[2];

    int64_t low = a[1];
    low = (low << 8) | a[0];
    return ((double)((high << 16) | low)) * 1E-3 ;
}

//unit : m/s
static inline double i2d_vel(uint8_t a[2])
{
    int16_t vel = a[1];
    vel = (vel << 8) | a[0];
    
    return ((double) vel) * 100.0/32768.0;
}


static inline int i2i_status(uint8_t a[1])
{
    uint8_t length = a[0];
    return ((int) length) ;
}

}