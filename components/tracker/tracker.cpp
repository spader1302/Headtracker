#include "tracker.hpp"
#include "bno055.hpp"

void Tracker::init()
{
    
}

void Tracker::center()
{

}

void Tracker::updateAxisValues()
{
    _joy_axes.x = _heading2JoyAxis();
    _joy_axes.y = _pitch2JoyAxis();
    _joy_axes.z = _roll2JoyAxis();
}

uint8_t Tracker::getX()
{
    return _joy_axes.x;
}

uint8_t Tracker::getY()
{
    return _joy_axes.y;
}

uint8_t Tracker::getZ()
{
    return _joy_axes.z;
}

uint8_t Tracker::_pitch2JoyAxis()
{
    return _mapSensor2JoyRange(_bno->get_eul_pitch(), 
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::PITCH_DEGREE_MIN,
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::PITCH_DEGREE_MAX);
}

uint8_t Tracker::_roll2JoyAxis()
{
    return _mapSensor2JoyRange(_bno->get_eul_roll(), 
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::ROLL_DEGREE_MIN,
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::ROLL_DEGREE_MAX);
}

uint8_t Tracker::_heading2JoyAxis()
{   
    //Heading turns over from 360° to 0° at the center point
    //shift sensor value by -180°:
    int16_t heading_value = _bno->get_eul_heading() - 
        bno_ranges::EULER_BITS_PER_DEGREE * 
        (bno_ranges::HEADING_DEGREE_MAX_FULL - bno_ranges::HEADING_DEGREE_MIN) / 2;

    //map shifted value to joy range
    return _mapSensor2JoyRange(heading_value, 
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::HEADING_DEGREE_MIN,
        bno_ranges::EULER_BITS_PER_DEGREE * bno_ranges::HEADING_DEGREE_MAX);
}

uint8_t Tracker::_mapSensor2JoyRange(int32_t sensor_value, int32_t min_in, int32_t max_in)
{
    if (sensor_value > max_in)
    {
        return UINT8_MAX;
    } 
    else if (sensor_value < min_in)
    {
        return 0;
    }
    else
    {
        return static_cast<uint8_t>((sensor_value - min_in) * __UINT8_MAX__ / (max_in - min_in));
    }    
}
    