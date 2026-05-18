// PCA9548A_Manager.cpp
// IMPLEMENTASI. basic control buat milih downstream sensornya yang mau komunikasi

#include "PCA9548A_Manager.h"
#include <Wire.h>

PCA9548A_Manager::PCA9548A_Manager(uint8_t addr)
    : _address(addr), _currentChannel(0xFF) {}

bool PCA9548A_Manager::begin() {
    // Ensure Wire is already initialized in setup()
    // Optionally, test communication by disabling all channels.
    return disableAll();
}

bool PCA9548A_Manager::selectChannel(uint8_t channel) {
    if (channel > 7) {
        return false;
    }
    // Write a single byte where only the bit corresponding to the channel is set.
    uint8_t data = 1 << channel;
    Wire.beginTransmission(_address);
    Wire.write(data);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        _currentChannel = channel;
        return true;
    }
    return false;
}

bool PCA9548A_Manager::disableAll() {
    Wire.beginTransmission(_address);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        _currentChannel = 0xFF;
        return true;
    }
    return false;
}

uint8_t PCA9548A_Manager::getCurrentChannel() const {
    return _currentChannel;
}