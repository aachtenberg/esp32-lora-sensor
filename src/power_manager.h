#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// Initialize power management
void initPowerManager();

// Read battery voltage
float readBatteryVoltage();

// Calculate battery percentage
uint8_t calculateBatteryPercent(float voltage);

// Enter deep sleep for specified seconds
void enterDeepSleep(uint32_t seconds);

// Get configured deep sleep interval
uint32_t getDeepSleepSeconds();

// Set deep sleep interval (persists to SPIFFS)
void setDeepSleepSeconds(uint32_t seconds);

#endif // POWER_MANAGER_H
