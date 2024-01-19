#include "tracker.hpp"
#include "bno055.hpp"

void Tracker::init()
{
    
}

void Tracker::center()
{
    _center_offset.pitch = _bno->getPitch();
    _center_offset.roll = _bno->getRoll();
    _center_offset.heading = _bno->getHeading();
}

uint8_t Tracker::getX()
{
    return _mapSensor2JoyRange(_bno->getHeading() - _center_offset.heading);
}

uint8_t Tracker::getY()
{
    return _mapSensor2JoyRange(_bno->getPitch() - _center_offset.pitch);
}

uint8_t Tracker::getZ()
{
    return _mapSensor2JoyRange(_bno->getRoll() - _center_offset.roll);
}

template<class T>
uint8_t Tracker::_mapSensor2JoyRange(T value)
{
    return _mapSensor2JoyRange(value,
        bno_ranges::EULER_BITS_PER_DEGREE * value.MIN,
        bno_ranges::EULER_BITS_PER_DEGREE * value.MAX);
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
    