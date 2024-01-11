#ifndef __BNO_055__HPP__
#define __BNO_055__HPP__

#include "esp_err.h"

typedef enum {
    HEADING = 0,
    YAW = 0,
    ROLL,
    PITCH,
} bno_rotation_axis_t;

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

class BNOSensor {

public:

    BNOSensor(uint8_t i2c_addr) :
        _i2c_addr{i2c_addr}
        {}
    
    esp_err_t begin();

    float eulByte2FloatDegrees(int16_t euler_byte);
    float eulByte2FloatRadians(int16_t euler_byte);
    float quaByte2Float(int16_t qua_byte);

    uint8_t pitch2Joy();
    uint8_t roll2Joy();
    uint8_t heading2Joy();

    esp_err_t setOpMode(bno_operating_mode_t mode);
    esp_err_t setPwrMode(bno_power_mode_t mode);
    esp_err_t setUnitMode(bno_unit_selection_t unit_mode);

    int16_t get_eul_heading();
    int16_t get_eul_roll();
    int16_t get_eul_pitch();
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

    uint8_t mapSensor2JoyRange(int32_t value, int32_t min_in, int32_t max_in);

    uint8_t _i2c_addr;

    union regBuffer {        
        int16_t s_bytes;
        uint16_t u_bytes;
        int8_t s_byte[2];
        uint8_t u_byte[2];
    } _reg_buffer{0};

    float _float_buffer{0.f};

    esp_err_t _err{ESP_OK};

};

#endif
