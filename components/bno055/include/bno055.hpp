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

    float eulByte2FloatDegrees(int16_t euler_byte);
    float eulByte2FloatRadians(int16_t euler_byte);
    float quaByte2Float(int16_t qua_byte);

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

};

#endif
