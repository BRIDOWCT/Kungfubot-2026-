// PCA9548A_Manager.h
// enkuapsulasi operasi kontrol multiplexer PCA9548A I2C. channel aktif satu"

#pragma once

#include <Arduino.h>

class PCA9548A_Manager {
public:
    PCA9548A_Manager(uint8_t addr);

    // Initialize the multiplexer; returns true on success
    bool begin();

    // Select a channel (0–7). Returns true if the channel was set.
    bool selectChannel(uint8_t channel);

    // Disable all channels
    bool disableAll();

    // Return the currently selected channel (0–7) or 0xFF if unknown
    uint8_t getCurrentChannel() const;

private:
    uint8_t _address;
    uint8_t _currentChannel;
};