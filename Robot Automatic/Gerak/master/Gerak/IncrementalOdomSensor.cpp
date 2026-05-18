// IncrementalOdomSensor.cpp
// implementsi
#include "IncrementalOdomSensor.h"
#include "Config.h"

IncrementalOdomSensor::IncrementalOdomSensor(float meterPerCount, bool invert)
    : _meterPerCount(meterPerCount), _invert(invert), _prevRaw(0), _deltaCount(0), _totalCount(0), _deltaDistance(0.0f) {}

void IncrementalOdomSensor::begin(uint16_t initialRaw) {
    _prevRaw      = initialRaw & 0x0FFF;
    _deltaCount   = 0;
    _totalCount   = 0;
    _deltaDistance = 0.0f;
}

int32_t IncrementalOdomSensor::update(uint16_t currentRaw) {
    // Mask to 12 bits
    currentRaw &= 0x0FFF;
    int32_t rawDelta = (int32_t)currentRaw - (int32_t)_prevRaw;
    rawDelta = unwrapDelta(rawDelta);

    // Apply direction inversion
    if (_invert) {
        rawDelta = -rawDelta;
    }
    _deltaCount = rawDelta;
    _totalCount += _deltaCount;
    _deltaDistance = (float)_deltaCount * _meterPerCount;
    _prevRaw = currentRaw;
    return _deltaCount;
}

int32_t IncrementalOdomSensor::unwrapDelta(int32_t rawDelta) const {
    // Half of the resolution used for wrap detection
    const int32_t halfRes = (int32_t)Config::AS5600_RESOLUTION / 2;
    if (rawDelta > halfRes) {
        rawDelta -= (int32_t)Config::AS5600_RESOLUTION;
    } else if (rawDelta < -halfRes) {
        rawDelta += (int32_t)Config::AS5600_RESOLUTION;
    }
    return rawDelta;
}

int32_t IncrementalOdomSensor::getDeltaCount() const {
    return _deltaCount;
}

int64_t IncrementalOdomSensor::getTotalCount() const {
    return _totalCount;
}

float IncrementalOdomSensor::getDeltaDistanceMeter() const {
    return _deltaDistance;
}

void IncrementalOdomSensor::reset(uint16_t currentRaw) {
    begin(currentRaw);
}