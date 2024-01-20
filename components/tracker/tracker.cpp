#include "tracker.hpp"
#include "bno055.hpp"

void Tracker::init()
{
    
}

void Tracker::center()
{
    _center_offset.y = _mapSensor2JoyRange(_bno->getPitch());
    _center_offset.z = _mapSensor2JoyRange(_bno->getRoll());
    _center_offset.x = _mapSensor2JoyRange(_bno->getHeading());
}

uint16_t Tracker::getX()
{
    return _mapSensor2JoyRange(_bno->getHeading()) - _center_offset.x;
}

uint16_t Tracker::getY()
{
    return _mapSensor2JoyRange(_bno->getPitch()) - _center_offset.y;
}

uint16_t Tracker::getZ()
{
    return _mapSensor2JoyRange(_bno->getRoll()) - _center_offset.z;
}

template<class T>
uint16_t Tracker::_mapSensor2JoyRange(T value)
{
    return _mapSensor2JoyRange(static_cast<int32_t>(value), value.MIN, value.MAX);
}

uint16_t Tracker::_mapSensor2JoyRange(int64_t sensor_value, int64_t min_in, int64_t max_in)
{
    if (sensor_value > max_in)
    {
        return UINT16_MAX;
    } 
    else if (sensor_value < min_in)
    {
        return 0;
    }
    else
    {
        return static_cast<uint16_t>((sensor_value - min_in) * __UINT16_MAX__ / (max_in - min_in));
    }    
}
    