
//
#pragma once
//
#include "Fusion/Fusion.h"
#include <iostream>
#include <string>
#include <cctype>
//
#define SAMPLE_RATE ( 100 )  // replace this with actual sample rate
//
//
struct SENSOR_DB
{
    float time;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float roll;
    float pitch;
    float yaw;
    float pos_x;
    float pos_y;
    float pos_z;
};
// 验证字符串是否为数字
static bool isNumber( const std::string& str )
{
    for ( char c : str )
    {
        if ( ! std::isdigit( c ) )
        {
            return false;
        }
    }
    return true;
}
//
class AhrsCalculation
{
private:
    /* data */
public:
    AhrsCalculation( /* args */ );
    ~AhrsCalculation();
public:
    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment     = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    const FusionVector gyroscopeSensitivity      = { 1.0f, 1.0f, 1.0f };
    const FusionVector gyroscopeOffset           = { 0.0f, 0.0f, 0.0f };
    const FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    const FusionVector accelerometerSensitivity  = { 1.0f, 1.0f, 1.0f };
    const FusionVector accelerometerOffset       = { 0.0f, 0.0f, 0.0f };
    const FusionMatrix softIronMatrix            = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    const FusionVector hardIronOffset            = { 0.0f, 0.0f, 0.0f };
public:
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .convention            = FusionConventionNwu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
public:
    void SolveAnCalculation( SENSOR_DB* sensor_data );
};
