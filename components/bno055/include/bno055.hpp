#ifndef __BNO_055__HPP__
#define __BNO_055__HPP__

#include "esp_err.h"

typedef enum {
    HEADING = 0,
    YAW = 0,
    ROLL,
    PITCH,
} bno_rotation_axis_t;

class BNOSensor {

public:

    BNOSensor(uint8_t i2c_addr) :
        _i2c_addr{i2c_addr}
        {}

    float eulByte2FloatDegrees(int16_t euler_byte);
    float eulByte2FloatRadians(int16_t euler_byte);
    float quaByte2Float(int16_t qua_byte);

    uint8_t pitch2Joy();
    uint8_t roll2Joy();
    uint8_t heading2Joy();

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
