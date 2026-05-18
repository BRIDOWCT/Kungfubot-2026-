// IncrementalOdomSensor.h
// konvert raw angle ke incremental jarak buat as5600

#pragma once

#include <Arduino.h>

class IncrementalOdomSensor {
public:
    IncrementalOdomSensor(float meterPerCount, bool invert);

    // Initialise the sensor with an initial raw value from the encoder
    void begin(uint16_t initialRaw);

    // Update the sensor with the current raw value. Returns the
    // delta count computed. Always call update before reading deltas.
    int32_t update(uint16_t currentRaw);

    // Retrieve the last delta count (since the previous update).
    int32_t getDeltaCount() const;

    // Retrieve the total count accumulated since reset
    int64_t getTotalCount() const;

    // Retrieve the delta distance in meters since last update
    float getDeltaDistanceMeter() const;

    // Reset the accumulated values
    void reset(uint16_t currentRaw);

private:
    float _meterPerCount;
    bool  _invert;

    uint16_t _prevRaw;
    int32_t _deltaCount;
    int64_t _totalCount;
    float   _deltaDistance;

    int32_t unwrapDelta(int32_t rawDelta) const;
};