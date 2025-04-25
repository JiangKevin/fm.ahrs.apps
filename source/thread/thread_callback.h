#pragma once
//
#include "Calculation/AhrsCalculation.h"
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"

struct sensor_device
{
    MMC56x3*         sensor_mmc;
    ICM42670*        sensor_imu;
    SENSOR_DB*       sensor_data;
    AhrsCalculation* ahrs_calculation;
    //
    Text* infoText;
};
//

// ----------------------------------------------------------------------
static void* read_sensor( void* arg )
{
    struct sensor_device* pArg = ( struct sensor_device* )arg;
    //
    String info = "Info:\n";
    //
    pArg->sensor_data->time = clock();
    info += "Time: " + String( pArg->sensor_data->time ) + "\n";
    // MMC56x3
    float x, y, z;
    if ( pArg->sensor_mmc->getEvent( x, y, z ) )
    {
        pArg->sensor_data->mag_x = x;
        pArg->sensor_data->mag_y = y;
        pArg->sensor_data->mag_z = z;
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
    pArg->sensor_data->acc_x  = imu_event.accel[ 0 ] / 2048.0;
    pArg->sensor_data->acc_y  = imu_event.accel[ 1 ] / 2048.0;
    pArg->sensor_data->acc_z  = imu_event.accel[ 2 ] / 2048.0;
    pArg->sensor_data->gyro_x = imu_event.gyro[ 0 ] / 16.4;
    pArg->sensor_data->gyro_y = imu_event.gyro[ 1 ] / 16.4;
    pArg->sensor_data->gyro_z = imu_event.gyro[ 2 ] / 16.4;
    info += "Accelerometer: x = " + String( pArg->sensor_data->acc_x ) + " g, y = " + String( pArg->sensor_data->acc_y ) + " g, z = " + String( pArg->sensor_data->acc_z ) + " g\n";
    info += "Gyroscope: x = " + String( pArg->sensor_data->gyro_x ) + " dps, y = " + String( pArg->sensor_data->gyro_y ) + " dps, z = " + String( pArg->sensor_data->gyro_z ) + " dps\n";

    //
    pArg->ahrs_calculation->SolveAnCalculation( pArg->sensor_data );
    //
    info += "Roll: " + String( pArg->sensor_data->roll ) + " Pitch: " + String( pArg->sensor_data->pitch ) + " Yaw: " + String( pArg->sensor_data->yaw ) + "\n";
    info += "Pos X: " + String( pArg->sensor_data->pos_x ) + " Pos Y: " + String( pArg->sensor_data->pos_y ) + " Pos Z: " + String( pArg->sensor_data->pos_z ) + "\n";
    //
    pArg->infoText->SetText( info );
    // Run @ ODR 100Hz
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    //
    return nullptr;
}