#include "AhrsCalculation.h"
#include <cstdio>
#include <time.h>
//
AhrsCalculation::AhrsCalculation( /* args */ )
{
    FusionAhrsSetSettings( &ahrs, &settings );
}
//
AhrsCalculation::~AhrsCalculation()
{
    //
}
//
void AhrsCalculation::SolveAnCalculation( SENSOR_DB* sensor_data )
{
    // Acquire latest sensor data
    const clock_t timestamp     = sensor_data->time;
    FusionVector  gyroscope     = { sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z };
    FusionVector  accelerometer = { sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z };
    FusionVector  magnetometer  = { sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z };
    //
    // Apply calibration
    gyroscope     = FusionCalibrationInertial( gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset );
    accelerometer = FusionCalibrationInertial( accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset );
    magnetometer  = FusionCalibrationMagnetic( magnetometer, softIronMatrix, hardIronOffset );

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate( &offset, gyroscope );

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static clock_t previousTimestamp;
    const float    deltaTime = ( float )( timestamp - previousTimestamp ) / ( float )CLOCKS_PER_SEC;
    previousTimestamp        = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    const FusionEuler  euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( &ahrs ) );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
    //
    sensor_data->roll  = euler.angle.roll;
    sensor_data->pitch = euler.angle.pitch;
    sensor_data->yaw   = euler.angle.yaw;
    sensor_data->pos_x = earth.axis.x;
    sensor_data->pos_y = earth.axis.y;
    sensor_data->pos_z = earth.axis.z;
    //
    // printf( "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, earth.axis.x, earth.axis.y, earth.axis.z );
}