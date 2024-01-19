#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <stdint.h>
#include "bno055.hpp"

class Tracker
{
public:

    Tracker(BNOSensor *bno) :
        _bno{bno}
        {}

    void init();
    void center();
    uint8_t getX();
    uint8_t getY();
    uint8_t getZ();    

private:

    template<class T>
    uint8_t _mapSensor2JoyRange(T value);
    uint8_t _mapSensor2JoyRange(int32_t value, int32_t min_in, int32_t max_in);

    struct center_offset
    {
        pitch_int_t pitch;
        roll_int_t roll;
        heading_int_t heading;
    } _center_offset{0};

    BNOSensor *_bno;

};

#endif // __TRACKER_HPP__