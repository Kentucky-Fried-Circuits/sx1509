// Wrapper.hpp
// C wrapper so I can set SX1509 pins from C libraries. From portserial_m.c in espressif__esp-modbus, in particular
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    // Define a prototype for the wrapper function
    void ioDigitalWrite(uint8_t pin, uint8_t highLow);
    
#ifdef __cplusplus
}
#endif