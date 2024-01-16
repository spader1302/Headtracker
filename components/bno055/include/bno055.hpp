#ifndef __BNO_055__HPP__
#define __BNO_055__HPP__

#include "esp_err.h"
#include "limited_int.hpp"

namespace bno_ranges
{

    static constexpr uint16_t EULER_BITS_PER_DEGREE     = 16;
    static constexpr uint16_t EULER_BITS_PER_RADIAN     = 900;
    static constexpr uint64_t QUA_BITS_PER_QUA          = 2e14;


    static constexpr int32_t PITCH_DEGREE_MIN           = -90;
    static constexpr int32_t PITCH_DEGREE_MAX           = 90;
    static constexpr int32_t ROLL_DEGREE_MIN            = -90;
    static constexpr int32_t ROLL_DEGREE_MAX            = 90;
    static constexpr int32_t HEADING_DEGREE_MIN         = -90;
    static constexpr int32_t HEADING_DEGREE_MAX         = 90;

    static constexpr int32_t PITCH_DEGREE_MIN_FULL      = -180;
    static constexpr int32_t PITCH_DEGREE_MAX_FULL      = 180;
    static constexpr int32_t ROLL_DEGREE_MIN_FULL       = -90;
    static constexpr int32_t ROLL_DEGREE_MAX_FULL       = 90;
    static constexpr int32_t HEADING_DEGREE_MIN_FULL    = 0;
    static constexpr int32_t HEADING_DEGREE_MAX_FULL    = 360;

}; // bno_ranges

// rotational axes
typedef enum {
    HEADING = 0,
    YAW = 0,
    ROLL,
    PITCH,
} bno_rotation_axis_t;

// lateral axes
typedef enum {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
} bno_axis_t;

typedef enum {
    // config mode
    CONFIG_MODE = 0,        // only mode where registers are writable, sensor fusion is halted, output reset to zero
    //non-fusion modes
    ACC_ONLY,       // stand-alone accelerometer data
    MAG_ONLY,       // stand-alone magnetometer
    GYRO_ONLY,      // stand-alone gyroscope
    ACC_MAG,        // accelerometer and magnetometer switched on
    ACC_GYRO,       // accelerometer and gyroscope switched on
    MAG_GYRO,       // magnetometer and gyroscope switched on
    ACC_MAG_GYRO,   // accelerometer, magnetometer and gyroscope switched on
    //fusion modes
    IMU,            // relative orientation calculated from accelerometer and gyroscope data. fast calculation
    COMPASS,        // for measuring magnetic earth field and calculating geographic direction
    M4G,            // similar to IMU, uses magnetometer instead of gyroscope. lower power consumption and no drift
    NDOF_FMC_OFF,   // same as NDOF, fast magnetometer calibration turned off
    NDOF,           // absolute orientation calculated from all sensors, fast, quick calibration
} bno_operating_mode_t;

typedef enum {
    NORMAL = 0,     // all sensors from current operating mode switched on
    LOW_POWER,      // only accelerometer active, automatically switches to NORMAL, when motion is detected
    SUSPEND         // system paused, all sensors off
} bno_power_mode_t;

typedef enum {
    ACC_M_P_SS,     // acceleration in [m/s²]
    ACC_MG,         // acceleration in [mg]
    ANG_DPS,        // angular rate in [°/s]
    ANG_RPS,        // angular rate in [rad/s]
    EUL_DEG,        // euler orientation in [°]
    EUL_RAD,        // euler orientation in [rad]
    TEMP_C,         // temperature in [°C]
    TEMP_F,         // temperature in [°F]
    DATA_WIN,       // fusion data output format [Windows]
    DATA_AND,       // fusion data output format [Android]
} bno_unit_selection_t;

typedef lim_int16_t<bno_ranges::PITCH_DEGREE_MIN_FULL * bno_ranges::EULER_BITS_PER_DEGREE,
    bno_ranges::PITCH_DEGREE_MAX_FULL * bno_ranges::EULER_BITS_PER_DEGREE> pitch_int_t;

typedef lim_int16_t<bno_ranges::ROLL_DEGREE_MIN_FULL * bno_ranges::EULER_BITS_PER_DEGREE,
    bno_ranges::ROLL_DEGREE_MAX_FULL * bno_ranges::EULER_BITS_PER_DEGREE> roll_int_t;

typedef lim_int16_t<bno_ranges::HEADING_DEGREE_MIN_FULL * bno_ranges::EULER_BITS_PER_DEGREE,
    bno_ranges::HEADING_DEGREE_MAX_FULL * bno_ranges::EULER_BITS_PER_DEGREE> heading_int_t;

class BNOSensor {

public:

    BNOSensor(uint8_t i2c_addr) :
        _i2c_addr{i2c_addr}
        {}
    
    // takes ~3s to initialize i2c bus
    esp_err_t begin();

    esp_err_t setOpMode(bno_operating_mode_t mode);
    esp_err_t setPwrMode(bno_power_mode_t mode);
    esp_err_t setUnitMode(bno_unit_selection_t unit_mode);

    esp_err_t remapAxes(bno_axis_t x_axis, bno_axis_t y_axis, bno_axis_t z_axis);

    //true for inverted axes
    esp_err_t invertAxes(bool x_axis_inv, bool y_axis_inv, bool z_axis_inv);

    pitch_int_t getPitch();
    roll_int_t getRoll();
    heading_int_t getHeading();

    // output in 1m/s² / 100bit or 1mg / 1bit
    int16_t get_grav_x();
    int16_t get_grav_y();
    int16_t get_grav_z();

    //output in 1° / 16bit or 1rad / 900bit
    int16_t get_eul_heading();
    int16_t get_eul_roll();
    int16_t get_eul_pitch();

    // output in 1quat / 2^14bit
    int16_t get_qua_w();
    int16_t get_qua_x();
    int16_t get_qua_y();
    int16_t get_qua_z();

private:

    int16_t _readRegister2ByteSigned(uint8_t reg_addr);
    uint16_t _readRegister2ByteUnsigned(uint8_t reg_addr);
    int8_t _readRegisterByteSigned(uint8_t reg_addr);
    uint8_t _readRegisterByteUnsigned(uint8_t reg_addr);

    esp_err_t _i2c_init();
    esp_err_t _readRegister(uint8_t reg_addr, uint8_t *data, size_t len);
    esp_err_t _writeRegister(uint8_t reg_addr, uint8_t data);

    uint8_t _i2c_addr;

    union regBuffer {        
        int16_t s_bytes;
        uint16_t u_bytes;
        int8_t s_byte[2];
        uint8_t u_byte[2];
    } _reg_buffer{0};

    float _float_buffer{0.f};

    esp_err_t _err{ESP_OK};

    bno_operating_mode_t _mode{CONFIG_MODE};

};

#endif
