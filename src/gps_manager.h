#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>

// GPS data structure for NEO-6M module
typedef struct {
    bool hasfix;           // Whether we have a GPS fix
    float latitude;        // Latitude in decimal degrees
    float longitude;       // Longitude in decimal degrees
    float altitude_m;      // Altitude in meters
    uint8_t satellites;    // Number of satellites used
    float hdop;            // Horizontal dilution of precision
    uint32_t last_fix_ms;  // Timestamp of last fix (milliseconds)
} GPSData;

// Initialize GPS module (UART on pins 3 and 1)
bool initGPS();

// Check for GPS data and update GPSData structure
// Returns true if new fix acquired
bool updateGPS(GPSData* data);

// Get current GPS data
void getGPSData(GPSData* data);

// Format GPS data as string for display
String formatGPSString();

// Is GPS fix valid
bool hasGPSFix();

#endif
