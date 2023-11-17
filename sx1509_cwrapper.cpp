// Wrapper.hpp
// wrapper so I can set SX1509 pins from C libraries. From portserial_m.c in espressif__esp-modbus, in particular

#include <stdint.h>


#include "sx1509_cwrapper.h"
#include "sx1509.hpp"

// io must be a public sx1509 object for this to work
extern SX1509 io;

// Define a prototype for the wrapper function
void ioDigitalWrite(uint8_t pin, uint8_t highLow) {
    io.digitalWrite(pin, highLow);
}
    
