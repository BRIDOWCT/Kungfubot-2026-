// AS5600_Reader.cpp
// implementasi 
#include "AS5600_Reader.h"
#include "Config.h"
#include <Wire.h>

AS5600_Reader::AS5600_Reader(uint8_t addr)
    : _address(addr) {}

bool AS5600_Reader::begin() {
    // Nothing special to initialise for AS5600; return true.
    return true;
}

bool AS5600_Reader::readRawAngle(uint16_t &raw) {
    // Request 2 bytes starting from the raw angle register
    Wire.beginTransmission(_address);
    Wire.write(Config::AS5600_REG_RAW_ANGLE);
    uint8_t err = Wire.endTransmission(false); // repeated start
    if (err != 0) {
        return false;
    }
    // Request two bytes
    uint8_t bytesRead = Wire.requestFrom(_address, (uint8_t)2);
    if (bytesRead != 2) {
        return false;
    }
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    raw = ((uint16_t)highByte << 8) | lowByte;
    // Ensure raw is within 0–4095 (12 bits)
    raw &= 0x0FFF;
    return true;
}

bool AS5600_Reader::readAngleDeg(float &deg) {
    uint16_t raw;
    if (!readRawAngle(raw)) {
        return false;
    }
    deg = ((float)raw) * 360.0f / (float)Config::AS5600_RESOLUTION;
    return true;
}