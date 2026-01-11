#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "lora_protocol.h"

// Initialize BME280 sensor
bool initSensor();

// Read sensor data and populate readings payload
// Returns true on success, false on failure
bool readSensorData(ReadingsPayload* readings);

// Get current pressure baseline (hPa)
float getPressureBaseline();

// Set pressure baseline (hPa)
void setPressureBaseline(float baseline);

// Calibrate baseline to current pressure
void calibrateBaseline();

// Clear baseline tracking
void clearBaseline();

#endif // SENSOR_MANAGER_H
