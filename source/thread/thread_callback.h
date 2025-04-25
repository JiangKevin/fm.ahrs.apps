#pragma once
//
#include "Calculation/AhrsCalculation.h"
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"

struct sensor_device
{
    MMC56x3*         sensor_mmc;
    ICM42670*        sensor_imu;
    AhrsCalculation* ahrs_calculation;
    //
    moodycamel::ConcurrentQueue< SENSOR_DB >* sensor_data_queue;
};
//

// ----------------------------------------------------------------------
static void* read_sensor( void* arg )
{
    struct sensor_device* pArg = ( struct sensor_device* )arg;
    //

    //
    while ( true )
    {
        String    info = "Info:\n";
        SENSOR_DB sensor_data;
        //
        sensor_data.time = clock();
        info += "Time: " + String( sensor_data.time ) + "\n";
        // MMC56x3
        float x, y, z;
        if ( pArg->sensor_mmc->getEvent( x, y, z ) )
        {
            sensor_data.mag_x = x;
            sensor_data.mag_y = y;
            sensor_data.mag_z = z;
            //
            info += "Magnetic field: x = " + String( x ) + " uT, y = " + String( y ) + " uT, z = " + String( z ) + " uT\n";
            //
            // std::cout << "Magnetic field: x = " << x << " uT, y = " << y << " uT, z = " << z << " uT, all = " << std::sqrt( std::pow( x, 2 ) + std::pow( y, 2 ) + std::pow( z, 2 ) ) << " uT" << std::endl;
        }

        float temp = pArg->sensor_mmc->readTemperature();
        if ( ! std::isnan( temp ) )
        {
            info += "Magnetic Temperature: " + String( temp ) + " C\n";
            // std::cout << "Temperature: " << temp << " C" << std::endl;
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
        sensor_data.acc_x  = imu_event.accel[ 0 ] / 2048.0;
        sensor_data.acc_y  = imu_event.accel[ 1 ] / 2048.0;
        sensor_data.acc_z  = imu_event.accel[ 2 ] / 2048.0;
        sensor_data.gyro_x = imu_event.gyro[ 0 ] / 16.4;
        sensor_data.gyro_y = imu_event.gyro[ 1 ] / 16.4;
        sensor_data.gyro_z = imu_event.gyro[ 2 ] / 16.4;
        info += "Accelerometer: x = " + String( sensor_data.acc_x ) + " g, y = " + String( sensor_data.acc_y ) + " g, z = " + String( sensor_data.acc_z ) + " g\n";
        info += "Gyroscope: x = " + String( sensor_data.gyro_x ) + " dps, y = " + String( sensor_data.gyro_y ) + " dps, z = " + String( sensor_data.gyro_z ) + " dps\n";

        //
        pArg->ahrs_calculation->SolveAnCalculation( &sensor_data );
        //
        info += "Roll: " + String( sensor_data.roll ) + " Pitch: " + String( sensor_data.pitch ) + " Yaw: " + String( sensor_data.yaw ) + "\n";
        info += "Pos X: " + String( sensor_data.pos_x ) + " Pos Y: " + String( sensor_data.pos_y ) + " Pos Z: " + String( sensor_data.pos_z ) + "\n";
        //
        sensor_data.info = info;
        // 生成数据
        pArg->sensor_data_queue->enqueue( sensor_data );
        // Run @ ODR 100Hz
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        //
    }

    //
    return nullptr;
}