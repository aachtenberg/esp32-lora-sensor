#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>

// Initialize configuration manager (loads from SPIFFS)
bool initConfigManager();

// Serial configuration menu
void checkSerialConfig();

// Get device name
String getDeviceName();

// Set device name (persists to SPIFFS)
void setDeviceName(const String& name);

// Load configuration from SPIFFS
bool loadConfig();

// Save configuration to SPIFFS
bool saveConfig();

#endif // CONFIG_MANAGER_H
