#pragma once
//
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
//

struct sensor_db
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
};
struct sensor_device
{
    MMC56x3*   sensor_mmc;
    ICM42670*  sensor_imu;
    sensor_db* sensor_data;
};

// ----------------------------------------------------------------------
static void* read_sensor( void* arg )
{
    struct sensor_device* pArg = ( struct sensor_device* )arg;
    // MMC56x3
    float x, y, z;
    if ( pArg->sensor_mmc->getEvent( x, y, z ) )
    {
        pArg->sensor_data->mag_x = x;
        pArg->sensor_data->mag_y = y;
        pArg->sensor_data->mag_z = z;
        //
        // std::cout << "Magnetic field: x = " << x << " uT, y = " << y << " uT, z = " << z << " uT, all = " << std::sqrt( std::pow( x, 2 ) + std::pow( y, 2 ) + std::pow( z, 2 ) ) << " uT" << std::endl;
    }

    float temp = pArg->sensor_mmc->readTemperature();
    if ( ! std::isnan( temp ) )
    {
        std::cout << "Temperature: " << temp << " C" << std::endl;
    }
    else
    {
        std::cout << "Cannot read temperature in continuous mode" << std::endl;
    }
    // TDK42607
    inv_imu_sensor_event_t imu_event;

    // Get last event
    pArg->sensor_imu->getDataFromRegisters( imu_event );
    //
    pArg->sensor_data->acc_x  = imu_event.accel[ 0 ] / 2048.0;
    pArg->sensor_data->acc_y  = imu_event.accel[ 1 ] / 2048.0;
    pArg->sensor_data->acc_z  = imu_event.accel[ 2 ] / 2048.0;
    pArg->sensor_data->gyro_x = imu_event.gyro[ 0 ] / 16.4;
    pArg->sensor_data->gyro_y = imu_event.gyro[ 1 ] / 16.4;
    pArg->sensor_data->gyro_z = imu_event.gyro[ 2 ] / 16.4;
    // Format data for Serial Plotter
    // printf( "AccelX:%f,", imu_event.accel[ 0 ] / 2048.0 );
    // printf( "AccelY:%f,", imu_event.accel[ 1 ] / 2048.0 );
    // printf( "AccelZ:%f,", imu_event.accel[ 2 ] / 2048.0 );
    // printf( "GyroX:%f,", imu_event.gyro[ 0 ] / 16.4 );
    // printf( "GyroY:%f,", imu_event.gyro[ 1 ] / 16.4 );
    // printf( "GyroZ:%f,", imu_event.gyro[ 2 ] / 16.4 );
    // printf( "Temperature:%f", ( imu_event.temperature / 128.0 ) + 25.0 );
    // printf( "\n" );

    // Run @ ODR 100Hz
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
}