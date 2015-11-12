/// -*- tab-width: ???; Mode: C++; c-basic-offset: ???; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_LSM9DS1_H__
#define __AP_INERTIAL_SENSOR_LSM9DS1_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

/* enable debug to see a register dump on startup */
#define LSM9DS1_DEBUG 0

class AP_InertialSensor_LSM9DS1 : public AP_InertialSensor_Backend
{
public:
    enum gyro_scale
    {
        G_SCALE_245DPS = 0,
        G_SCALE_500DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale
    {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
    };

    AP_InertialSensor_LSM9DS1(AP_InertialSensor &imu);

    bool update();

    bool gyro_sample_available() { return _gyro_sample_available; }
    bool accel_sample_available() { return _accel_sample_available; }

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    bool _init_sensor();
    void _read_data_transaction_a();
    void _read_data_transaction_g();
    bool _accel_data_ready();
    bool _gyro_data_ready();
    void _poll_data();
    uint8_t _register_read_xg(uint8_t reg);
    void _register_write_xg(uint8_t reg, uint8_t val);
    bool _hardware_init();
    void _gyro_init();
    void _accel_init();

    bool _gyro_sample_available;
    bool _accel_sample_available;

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    float _gyro_scale, _accel_scale;
    void _set_gyro_scale(gyro_scale scale);
    void _set_accel_scale(accel_scale scale);

    void _accel_raw_data(struct sensor_raw_data *raw_data);
    void _gyro_raw_data(struct sensor_raw_data *raw_data);

    /* support for updating filter at runtime */
    int16_t _last_gyro_filter_hz;
    int16_t _last_accel_filter_hz;

    /* change the filter frequency */
    void _set_accel_filter(uint8_t filter_hz);
    void _set_gyro_filter(uint8_t filter_hz);

    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;

    /* Low Pass filters for gyro and accel */
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;

    #if LSM9DS1_DEBUG
      static void _dump_registers();
    #endif
};

#endif /* __AP_INERTIAL_SENSOR_LSM9DS1_H__ */
