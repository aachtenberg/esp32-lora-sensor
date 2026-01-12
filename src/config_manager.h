#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>

// Initialize configuration manager (loads from LittleFS)
bool initConfigManager();

// Serial configuration menu
void checkSerialConfig();

// Get device name
String getDeviceName();

// Set device name (persists to LittleFS)
void setDeviceName(const String& name);

// Get deep sleep interval in seconds
uint32_t getDeepSleepSeconds();

// Set deep sleep interval (persists to LittleFS)
void setDeepSleepSeconds(uint32_t seconds);

// Get sensor interval in seconds
uint32_t getSensorIntervalSeconds();

// Set sensor interval (persists to LittleFS)
void setSensorIntervalSeconds(uint32_t seconds);

// Get pressure baseline in hPa
float getConfigPressureBaseline();

// Set pressure baseline (persists to LittleFS)
void setConfigPressureBaseline(float baseline);

// Load configuration from LittleFS
bool loadConfig();

// Save configuration to LittleFS
bool saveConfig();

#endif // CONFIG_MANAGER_H
