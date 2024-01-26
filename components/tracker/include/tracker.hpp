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
    uint16_t getX();
    uint16_t getY();
    uint16_t getZ();    

private:

    template<class T>
    uint16_t _mapSensor2JoyRange(T value);
    uint16_t _mapSensor2JoyRange(int64_t value, int64_t min_in, int64_t max_in);

    /*
    struct center_offset
    {
        pitch_int_t pitch;
        roll_int_t roll;
        heading_int_t heading;
    } _center_offset{0};
    */
   struct center_offset
   {
        int16_t x{0};
        int16_t y{0};
        int16_t z{0};
   } _center_offset;

    BNOSensor *_bno;

};

#endif // __TRACKER_HPP__