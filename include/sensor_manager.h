/**
 * Sensor Manager - BME280 Environmental Sensor
 * Public interface for BME280 sensor operations
 */

#pragma once

#include <Arduino.h>
#include "lora_protocol.h"

// Initialize BME280 sensor
bool initSensor();

// Read current sensor data into payload structure
bool readSensorData(ReadingsPayload* readings);

// Pressure baseline management
float getPressureBaseline();
void setPressureBaseline(float baseline);
void calibrateBaseline();  // Set baseline to current pressure
void clearBaseline();      // Disable baseline tracking

// SPIFFS persistence (internal use)
float loadPressureBaseline();
void savePressureBaseline(float baseline);
