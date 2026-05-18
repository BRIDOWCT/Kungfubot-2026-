// AS5600_Reader.h
// Baca sensor as5600. Raw angle ke sudut 
#pragma once

#include <Arduino.h>

class AS5600_Reader {
public:
    AS5600_Reader(uint8_t addr);

    // Initialize the sensor (currently a no-op). Returns true on success.
    bool begin();

    // Read the raw 12-bit angle from the AS5600. Returns true if the
    // reading was successful. The raw value will be placed in `raw`.
    bool readRawAngle(uint16_t &raw);

    // Read the angle and convert to degrees. Returns true on success.
    bool readAngleDeg(float &deg);

private:
    uint8_t _address;
};