#include <inttypes.h>
#include "driver/i2c.h"
#include "esp_log.h"

#include "bno055.hpp"

#define BNO_055_TAG     "BNO055"

// PAGE 0 REGISTER ADDRESS VALUES
static constexpr uint8_t MAG_RADIUS_MSB_ADDR    = 0x6A;
static constexpr uint8_t MAG_RADIUS_LSB_ADDR    = 0x69;
static constexpr uint8_t ACC_RADIUS_MSB_ADDR    = 0x68;
static constexpr uint8_t ACC_RADIUS_LSB_ADDR    = 0x67;
static constexpr uint8_t GYR_OFFSET_Z_MSB_ADDR  = 0x66;
static constexpr uint8_t GYR_OFFSET_Z_LSB_ADDR  = 0x65;
static constexpr uint8_t GYR_OFFSET_Y_MSB_ADDR  = 0x64;
static constexpr uint8_t GYR_OFFSET_Y_LSB_ADDR  = 0x63;
static constexpr uint8_t GYR_OFFSET_X_MSB_ADDR  = 0x62;
static constexpr uint8_t GYR_OFFSET_X_LSB_ADDR  = 0x61;
static constexpr uint8_t MAG_OFFSET_Z_MSB_ADDR  = 0x60;
static constexpr uint8_t MAG_OFFSET_Z_LSB_ADDR  = 0x5F;
static constexpr uint8_t MAG_OFFSET_Y_MSB_ADDR  = 0x5E;
static constexpr uint8_t MAG_OFFSET_Y_LSB_ADDR  = 0x5D;
static constexpr uint8_t MAG_OFFSET_X_MSB_ADDR  = 0x5C;
static constexpr uint8_t MAG_OFFSET_X_LSB_ADDR  = 0x5B;
static constexpr uint8_t ACC_OFFSET_Z_MSB_ADDR  = 0x5A;
static constexpr uint8_t ACC_OFFSET_Z_LSB_ADDR  = 0x59;
static constexpr uint8_t ACC_OFFSET_Y_MSB_ADDR  = 0x58;
static constexpr uint8_t ACC_OFFSET_Y_LSB_ADDR  = 0x57;
static constexpr uint8_t ACC_OFFSET_X_MSB_ADDR  = 0x56;
static constexpr uint8_t ACC_OFFSET_X_LSB_ADDR  = 0x55;
static constexpr uint8_t SIC_MATRIX_MSB8_ADDR   = 0x54;
static constexpr uint8_t SIC_MATRIX_LSB8_ADDR   = 0x53;
static constexpr uint8_t SIC_MATRIX_MSB7_ADDR   = 0x52;
static constexpr uint8_t SIC_MATRIX_LSB7_ADDR   = 0x51;
static constexpr uint8_t SIC_MATRIX_MSB6_ADDR   = 0x50;
static constexpr uint8_t SIC_MATRIX_LSB6_ADDR   = 0x4F;
static constexpr uint8_t SIC_MATRIX_MSB5_ADDR   = 0x4E;
static constexpr uint8_t SIC_MATRIX_LSB5_ADDR   = 0x4D;
static constexpr uint8_t SIC_MATRIX_MSB4_ADDR   = 0x4C;
static constexpr uint8_t SIC_MATRIX_LSB4_ADDR   = 0x4B;
static constexpr uint8_t SIC_MATRIX_MSB3_ADDR   = 0x4A;
static constexpr uint8_t SIC_MATRIX_LSB3_ADDR   = 0x49;
static constexpr uint8_t SIC_MATRIX_MSB2_ADDR   = 0x48;
static constexpr uint8_t SIC_MATRIX_LSB2_ADDR   = 0x47;
static constexpr uint8_t SIC_MATRIX_MSB1_ADDR   = 0x46;
static constexpr uint8_t SIC_MATRIX_LSB1_ADDR   = 0x45;
static constexpr uint8_t SIC_MATRIX_MSB0_ADDR   = 0x44;
static constexpr uint8_t SIC_MATRIX_LSB0_ADDR   = 0x43;
static constexpr uint8_t AXIS_MAP_SIGN_ADDR     = 0x42;
static constexpr uint8_t AXIS_MAP_CONFIG_ADDR   = 0x41;
static constexpr uint8_t TEMP_SOURCE_ADDR       = 0x40;
static constexpr uint8_t SYS_TRIGGER_ADDR       = 0x3F;
static constexpr uint8_t PWR_MODE_ADDR          = 0x3E;
static constexpr uint8_t OPR_MODE_ADDR          = 0x3D;
static constexpr uint8_t UNIT_SEL_ADDR          = 0x3B;
static constexpr uint8_t SYS_ERR_ADDR           = 0x3A;
static constexpr uint8_t SYS_STATUS_ADDR        = 0x39;
static constexpr uint8_t SYS_CLK_STATUS_ADDR    = 0x38;
static constexpr uint8_t INT_STA_ADDR           = 0x37;
static constexpr uint8_t ST_RESULT_ADDR         = 0x36;
static constexpr uint8_t CALIB_STAT_ADDR        = 0x35;
static constexpr uint8_t TEMP_ADDR              = 0x34;
static constexpr uint8_t GRV_Data_Z_MSB_ADDR    = 0x33;
static constexpr uint8_t GRV_Data_Z_LSB_ADDR    = 0x32;
static constexpr uint8_t GRV_Data_Y_MSB_ADDR    = 0x31;
static constexpr uint8_t GRV_Data_Y_LSB_ADDR    = 0x30;
static constexpr uint8_t GRV_Data_X_MSB_ADDR    = 0x2F;
static constexpr uint8_t GRV_Data_X_LSB_ADDR    = 0x2E;
static constexpr uint8_t LIA_Data_Z_MBS_ADDR    = 0x2D;
static constexpr uint8_t LIA_Data_Z_LSB_ADDR    = 0x2C;
static constexpr uint8_t LIA_Data_Y_MBS_ADDR    = 0x2B;
static constexpr uint8_t LIA_Data_Y_LSB_ADDR    = 0x2A;
static constexpr uint8_t LIA_Data_X_MBS_ADDR    = 0x29;
static constexpr uint8_t LIA_Data_X_LSB_ADDR    = 0x28;
static constexpr uint8_t QUA_Data_z_MSB_ADDR    = 0x27;
static constexpr uint8_t QUA_Data_z_LSB_ADDR    = 0x26;
static constexpr uint8_t QUA_Data_y_MSB_ADDR    = 0x25;
static constexpr uint8_t QUA_Data_y_LSB_ADDR    = 0x24;
static constexpr uint8_t QUA_Data_x_MSB_ADDR    = 0x23;
static constexpr uint8_t QUA_Data_x_LSB_ADDR    = 0x22;
static constexpr uint8_t QUA_Data_w_MSB_ADDR    = 0x21;
static constexpr uint8_t QUA_Data_w_LSB_ADDR    = 0x20;
static constexpr uint8_t EUL_Pitch_MSB_ADDR     = 0x1F;
static constexpr uint8_t EUL_Pitch_LSB_ADDR     = 0x1E;
static constexpr uint8_t EUL_Roll_MSB_ADDR      = 0x1D;
static constexpr uint8_t EUL_Roll_LSB_ADDR      = 0x1C;
static constexpr uint8_t EUL_Heading_MSB_ADDR   = 0x1B;
static constexpr uint8_t EUL_Heading_LSB_ADDR   = 0x1A;
static constexpr uint8_t GYR_DATA_Z_MSB_ADDR    = 0x19;
static constexpr uint8_t GYR_DATA_Z_LSB_ADDR    = 0x18;
static constexpr uint8_t GYR_DATA_Y_MSB_ADDR    = 0x17;
static constexpr uint8_t GYR_DATA_Y_LSB_ADDR    = 0x16;
static constexpr uint8_t GYR_DATA_X_MSB_ADDR    = 0x15;
static constexpr uint8_t GYR_DATA_X_LSB_ADDR    = 0x14;
static constexpr uint8_t MAG_DATA_Z_MSB_ADDR    = 0x13;
static constexpr uint8_t MAG_DATA_Z_LSB_ADDR    = 0x12;
static constexpr uint8_t MAG_DATA_Y_MSB_ADDR    = 0x11;
static constexpr uint8_t MAG_DATA_Y_LSB_ADDR    = 0x10;
static constexpr uint8_t MAG_DATA_X_MSB_ADDR    = 0x0F;
static constexpr uint8_t MAG_DATA_X_LSB_ADDR    = 0x0E;
static constexpr uint8_t ACC_DATA_Z_MSB_ADDR    = 0x0D;
static constexpr uint8_t ACC_DATA_Z_LSB_ADDR    = 0x0C;
static constexpr uint8_t ACC_DATA_Y_MSB_ADDR    = 0x0B;
static constexpr uint8_t ACC_DATA_Y_LSB_ADDR    = 0x0A;
static constexpr uint8_t ACC_DATA_X_MSB_ADDR    = 0x09;
static constexpr uint8_t ACC_DATA_X_LSB_ADDR    = 0x08;
static constexpr uint8_t PAGE_ID_ADDR           = 0x07;
static constexpr uint8_t BL_Rev_ID_ADDR         = 0x06;
static constexpr uint8_t SW_REV_ID_MSB_ADDR     = 0x05;
static constexpr uint8_t SW_REV_ID_LSB_ADDR     = 0x04;
static constexpr uint8_t GYR_ID_ADDR            = 0x03;
static constexpr uint8_t MAG_ID_ADDR            = 0x02;
static constexpr uint8_t ACC_ID_ADDR            = 0x01;
static constexpr uint8_t CHIP_ID_ADDR           = 0x00;

// PAGE 1 REGISTER ADDRESS VALUES
static constexpr uint8_t UNIQUE_ID_ADDR         = 0x5F;      //5F - 50
static constexpr uint8_t GYR_AM_SET_ADDR        = 0x1F;
static constexpr uint8_t GYR_AM_THRES_ADDR      = 0x1E;
static constexpr uint8_t GYR_DUR_Z_ADDR         = 0x1D;
static constexpr uint8_t GYR_HR_Z_SET_ADDR      = 0x1C;
static constexpr uint8_t GYR_DUR_Y_ADDR         = 0x1B;
static constexpr uint8_t GYR_HR_Y_SET_ADDR      = 0x1A;
static constexpr uint8_t GYR_DUR_X_ADDR         = 0x19;
static constexpr uint8_t GYR_HR_X_SET_ADDR      = 0x18;
static constexpr uint8_t GYR_INT_SETING_ADDR    = 0x17;
static constexpr uint8_t ACC_NM_SET_ADDR        = 0x16;
static constexpr uint8_t ACC_NM_THRE_ADDR       = 0x15;
static constexpr uint8_t ACC_HG_THRES_ADDR      = 0x14;
static constexpr uint8_t ACC_HG_DURATION_ADDR   = 0x13;
static constexpr uint8_t ACC_INT_Settings_ADDR  = 0x12;
static constexpr uint8_t ACC_AM_THRES_ADDR      = 0x11;
static constexpr uint8_t INT_EN_ADDR            = 0x10;
static constexpr uint8_t INT_MSK_ADDR           = 0x0F;
static constexpr uint8_t GYR_Sleep_Config_ADDR  = 0x0D;
static constexpr uint8_t ACC_Sleep_Config_ADDR  = 0x0C;
static constexpr uint8_t GYR_Config_1_ADDR      = 0x0B;
static constexpr uint8_t GYR_Config_0_ADDR      = 0x0A;
static constexpr uint8_t MAG_Config_ADDR        = 0x09;
static constexpr uint8_t ACC_Config_ADDR        = 0x08;
// static constexpr uint8_t PAGE_ID_ADDR        = 0x07;        // same as in page 0

// I2C
#define I2C_MASTER_SCL_IO           1                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           2                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                  /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// Conversion Constants
static constexpr float EULER_BITS_PER_DEGREE_F  = 16.f;
static constexpr float EULER_BITS_PER_RADIAN_F  = 900.f;
static constexpr float QUA_BITS_PER_QUA_F       = 2e14f;

static constexpr uint8_t EULER_BITS_PER_DEGREE_I   = 16;
static constexpr uint16_t EULER_BITS_PER_RADIAN_I  = 900;
static constexpr uint64_t QUA_BITS_PER_QUA_I       = 2e14;

static constexpr int8_t PITCH_DEGREE_MIN        = -180;
static constexpr int8_t PITCH_DEGREE_MAX        = 180;
static constexpr int8_t ROLL_DEGREE_MIN         = -90;
static constexpr int8_t ROLL_DEGREE_MAX         = 90;
static constexpr int8_t HEADING_DEGREE_MIN      = 0;
static constexpr int8_t HEADING_DEGREE_MAX      = 360;

esp_err_t BNOSensor::begin()
{
    return _i2c_init();
}

float BNOSensor::eulByte2FloatDegrees(int16_t euler_byte)
{
    return static_cast<float>(euler_byte / EULER_BITS_PER_DEGREE_F);
}

float BNOSensor::eulByte2FloatRadians(int16_t euler_byte)
{
    return static_cast<float>(euler_byte / EULER_BITS_PER_RADIAN_F);
}

float BNOSensor::quaByte2Float(int16_t qua_byte)
{
    return static_cast<float>(qua_byte / QUA_BITS_PER_QUA_F);
}

uint8_t BNOSensor::pitch2Joy()
{
    // -180°..180° -> -2880..2880 -> 0..255
    // sensor output has to be configured to euler angles
    // ESP_LOGI(BNO_055_TAG, "%s raw pitch: %d", __func__, get_eul_pitch());
    return mapSensor2JoyRange(get_eul_pitch(), EULER_BITS_PER_DEGREE_I * PITCH_DEGREE_MIN,
        EULER_BITS_PER_DEGREE_I * PITCH_DEGREE_MAX);
}

uint8_t BNOSensor::roll2Joy()
{
    // -90°..90° -> -1440..1400 -> 0..255
    // sensor output has to be configured to euler angles
    return mapSensor2JoyRange(get_eul_roll(), EULER_BITS_PER_DEGREE_I * ROLL_DEGREE_MIN,
        EULER_BITS_PER_DEGREE_I * ROLL_DEGREE_MAX);
}

uint8_t BNOSensor::heading2Joy()
{
    // 0°..360° -> 0..5760 -> 0..255
    // sensor output has to be configured to euler angles
    return mapSensor2JoyRange(get_eul_heading(), EULER_BITS_PER_DEGREE_I * HEADING_DEGREE_MIN,
        EULER_BITS_PER_DEGREE_I * HEADING_DEGREE_MAX);
}

esp_err_t BNOSensor::setOpMode(bno_operating_mode_t mode)
{
    return _writeRegister(OPR_MODE_ADDR, static_cast<uint8_t>(mode));
}

esp_err_t BNOSensor::setPwrMode(bno_power_mode_t mode)
{
    return _writeRegister(PWR_MODE_ADDR, static_cast<uint8_t>(mode));
}

esp_err_t BNOSensor::setUnitMode(bno_unit_selection_t unit_mode)
{
    uint8_t unit_reg;

    esp_err_t err = _readRegister(UNIT_SEL_ADDR, &unit_reg, 1);

    if (err != ESP_OK) {
        ESP_LOGE(BNO_055_TAG, "%s error while reading from register\n", __func__);
        return err;
    }

    switch (unit_mode)
    {
        case ACC_M_P_SS:
            unit_reg &= ~(1<<0);
            break;
        case ACC_MG:
            unit_reg |= (1<<0);
            break;
        case ANG_DPS:
            unit_reg &= ~(1<<1);
            break;
        case ANG_RPS:
            unit_reg |= (1<<1);
            break;
        case EUL_DEG:
            unit_reg &= ~(1<<2);
            break;
        case EUL_RAD:
            unit_reg |= (1<<2);
            break;
        case TEMP_C:
            unit_reg &= ~(1<<4);
            break;
        case TEMP_F:
            unit_reg |= (1<<4);
            break;
        case DATA_WIN:
            unit_reg &= ~(1<<7);
            break;
        case DATA_AND:
            unit_reg |= (1<<7);
            break;
    }

    return _writeRegister(UNIT_SEL_ADDR, unit_reg);
}

int16_t BNOSensor::get_eul_heading()
{
    return _readRegister2ByteSigned(EUL_Heading_LSB_ADDR);
}

int16_t BNOSensor::get_eul_roll()
{
    return _readRegister2ByteSigned(EUL_Roll_LSB_ADDR);
}

int16_t BNOSensor::get_eul_pitch()
{
    return _readRegister2ByteSigned(EUL_Pitch_LSB_ADDR);
}

int16_t BNOSensor::get_qua_w()
{
    return _readRegister2ByteSigned(QUA_Data_w_LSB_ADDR);
}

int16_t BNOSensor::get_qua_x()
{
    return _readRegister2ByteSigned(QUA_Data_x_LSB_ADDR);
}

int16_t BNOSensor::get_qua_y()
{
    return _readRegister2ByteSigned(QUA_Data_y_LSB_ADDR);
}

int16_t BNOSensor::get_qua_z()
{
    return _readRegister2ByteSigned(QUA_Data_z_LSB_ADDR);
}

int16_t BNOSensor::_readRegister2ByteSigned(uint8_t reg_addr)
{
    _err = _readRegister(reg_addr, &_reg_buffer.u_byte[0], 2);
    if (_err == ESP_OK) 
    {
        return _reg_buffer.s_bytes;
    }
    else 
    {
        return 0;
    }
}

uint16_t BNOSensor::_readRegister2ByteUnsigned(uint8_t reg_addr)
{
    _err = _readRegister(reg_addr, &_reg_buffer.u_byte[0], 2);
    if (_err == ESP_OK) 
    {
        return _reg_buffer.u_bytes;
    }
    else 
    {
        return 0;
    }
}

int8_t BNOSensor::_readRegisterByteSigned(uint8_t reg_addr)
{
    _err = _readRegister(reg_addr, &_reg_buffer.u_byte[0], 1);
    if (_err == ESP_OK) 
    {
        return _reg_buffer.s_byte[0];
    }
    else 
    {
        return 0;
    }
}

uint8_t BNOSensor::_readRegisterByteUnsigned(uint8_t reg_addr)
{
    _err = _readRegister(reg_addr, &_reg_buffer.u_byte[0], 1);
    if (_err == ESP_OK) 
    {
        return _reg_buffer.u_byte[0];
    }
    else 
    {
        return 0;
    }
}

esp_err_t BNOSensor::_i2c_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = I2C_MASTER_FREQ_HZ,           // .master.clk_speed is the only struct member
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t BNOSensor::_readRegister(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, _i2c_addr, &reg_addr, 1, data, len,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t BNOSensor::_writeRegister(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, _i2c_addr, write_buf, sizeof(write_buf),
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

uint8_t BNOSensor::mapSensor2JoyRange(int32_t value, int32_t min_in, int32_t max_in)
{
    return static_cast<uint8_t>((value - min_in) * __UINT8_MAX__ / (max_in - min_in));
}