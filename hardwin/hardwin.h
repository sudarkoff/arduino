#ifndef __HARDWIN_H
#define __HARDWIN_H

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <EEPROM.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

namespace hardwin {

    void flashLed(int pin, int times, int wait);

    template <typename T>
    bool equal(T const& lhs, T const& rhs, const double epsilon)
    {
        return (fabs(lhs - rhs) < epsilon);
    }

    template <typename T>
    int EEPROM_write(int addr, T const& value)
    {
        const byte* p = reinterpret_cast<byte const* const>(reinterpret_cast<void const* const>(&value));
        int i;
        for (i = 0; i < sizeof(value); i++) {
            EEPROM.write(addr++, *p++);
        }
        return i;
    }

    template <typename T>
    int EEPROM_read(int addr, T& value)
    {
        byte* p = reinterpret_cast<byte*>(reinterpret_cast<void*>(&value));
        int i;
        for (i = 0; i < sizeof(value); i++) {
            *p++ = EEPROM.read(addr++);
        }
        return i;
    }

    template <typename T>
    float EEPROM_read(int addr, T& value, T const& minVal, T const& maxVal, T const& defaultVal)
    {
        EEPROM_read(addr, value);
        // if the value is out of range ...
        if ((value < minVal) || (value > maxVal)) {
            // ... replace it with default value
            value = defaultVal;
            EEPROM_write(addr, value);
        }
        return value;
    }

} // namespace hardwin

#endif // __HARDWIN_H
