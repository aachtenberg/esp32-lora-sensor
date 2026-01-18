#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "lora_protocol.h"

// Initialize sensor (BME280 or DS18B20 based on compile-time flag)
bool initSensor();

// Read sensor data and populate readings payload
// Returns true on success, false on failure
bool readSensorData(ReadingsPayload* readings);

// Get sensor failure count
uint16_t getSensorFailures();

#ifdef SENSOR_TYPE_BME280
// BME280-specific functions (pressure baseline tracking)

// Get current pressure baseline (hPa)
float getPressureBaseline();

// Set pressure baseline (hPa)
void setPressureBaseline(float baseline);

// Calibrate baseline to current pressure
void calibrateBaseline();

// Clear baseline tracking
void clearBaseline();

#endif // SENSOR_TYPE_BME280

#endif // SENSOR_MANAGER_H
