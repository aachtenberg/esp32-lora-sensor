#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include "lora_protocol.h"

// Initialize OLED display
void initDisplay();

// Display startup screen
void displayStartup(const char* version);

// Display sensor readings
void displayReadings(const ReadingsPayload* readings);

// Display status message
void displayStatus(const char* message);

// Display error message
void displayError(const char* error);

// Update TX statistics for display
void updateTxStats(uint32_t count, int16_t rssi);

// Clear display
void clearDisplay();

#endif // DISPLAY_MANAGER_H
