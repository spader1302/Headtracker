#ifndef __LIMITED_INT__HPP__
#define __LIMITED_INT__HPP__

#include <stdint.h>

template<int16_t min, int16_t max>
class lim_int16_t
{
public:
    // implicit conversion constructor, making sure the value is in the range [min, max]
    lim_int16_t(int16_t v = 0) {
        data = v;
        while (_checkRange());
    }

    // implicit user defined conversion operator
    operator int16_t () const { return data; }

    // pre increment and decrement operators
    lim_int16_t& operator++() { if(data==max) data = min; else ++data; return *this; }
    lim_int16_t& operator--() { if(data==min) data = max; else --data; return *this; }

    // post increment and decrement operators
    lim_int16_t operator++(int) { lim_int16_t rv(*this); ++*this; return rv; }
    lim_int16_t operator--(int) { lim_int16_t rv(*this); --*this; return rv; }

    // common arithmetic operators
    lim_int16_t& operator+=(const lim_int16_t& rhs) {
        data += rhs.data;
        while (_checkRange());
        return *this;
    }
    
    lim_int16_t& operator-=(const lim_int16_t& rhs) {
        data -= rhs.data;
        while (_checkRange());
        return *this;
    }

    lim_int16_t& operator*=(const lim_int16_t& rhs) {
        data *= rhs.data;
        while (_checkRange());
        return *this;
    }
    
    lim_int16_t& operator/=(const lim_int16_t& rhs) {
        data /= rhs.data;
        while (_checkRange());
        return *this;
    }
private:

    bool _checkRange() {
        if (data > max) { data += min - max; return true; }
        else if (data < min) { data += max - min; return true; }
        return false;
    }

    int16_t data;
};

// free arithmetic functions
template<int16_t min, int16_t max>
auto operator+(const lim_int16_t<min, max>& lhs, const lim_int16_t<min, max>& rhs) {
    lim_int16_t rv(lhs);
    rv += rhs;
    return rv;
}

template<int16_t min, int16_t max>
auto operator-(const lim_int16_t<min, max>& lhs, const lim_int16_t<min, max>& rhs) {
    lim_int16_t rv(lhs);
    rv -= rhs;
    return rv;
}

template<int16_t min, int16_t max>
auto operator*(const lim_int16_t<min, max>& lhs, const lim_int16_t<min, max>& rhs) {
    lim_int16_t rv(lhs);
    rv *= rhs;
    return rv;
}

template<int16_t min, int16_t max>
auto operator/(const lim_int16_t<min, max>& lhs, const lim_int16_t<min, max>& rhs) {
    lim_int16_t rv(lhs);
    rv /= rhs;
    return rv;
}

#endif // __LIMITED_INT__HPP__