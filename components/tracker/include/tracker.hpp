#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <stdint.h>

class BNOSensor;

class Tracker
{
public:

    Tracker(BNOSensor *bno) :
        _bno{bno}
        {}

    void init();
    void center();
    void updateAxisValues();
    uint8_t getX();
    uint8_t getY();
    uint8_t getZ();    

private:

    uint8_t _pitch2JoyAxis();
    uint8_t _roll2JoyAxis();
    uint8_t _heading2JoyAxis();

    uint8_t _mapSensor2JoyRange(int32_t value, int32_t min_in, int32_t max_in);

    struct joy_axes
    {
        uint8_t x;
        uint8_t y;
        uint8_t z;
    } _joy_axes{UINT8_MAX / 2};

    struct center_offset
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } _center_offset{0};

    BNOSensor *_bno;

};

#endif // __TRACKER_HPP__