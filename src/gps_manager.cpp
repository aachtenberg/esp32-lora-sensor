#include "gps_manager.h"
#include "device_config.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// GPS module on UART1 (RX on GPIO 3, TX on GPIO 1)
// Note: GPIO1 is TX shared with DHT22 module in sensor builds
#define GPS_RX_PIN 3
#define GPS_TX_PIN 1
#define GPS_BAUD 9600

// TinyGPS++ library for NMEA parsing
static TinyGPSPlus gps;
static HardwareSerial gpsSerial(1);  // UART1

// GPS data storage
static GPSData currentGPS = {
    .hasfix = false,
    .latitude = 0.0,
    .longitude = 0.0,
    .altitude_m = 0.0,
    .satellites = 0,
    .hdop = 999.0,
    .last_fix_ms = 0
};

/**
 * Initialize GPS module
 * Opens UART1 connection to NEO-6M at 9600 baud
 */
bool initGPS() {
    Serial.println("[GPS] Initializing NEO-6M GPS module...");
    
    // Start UART1 for GPS (RX=GPIO3, TX=GPIO1)
    // Note: GPIO1 conflicts with Serial debug output if DHT22 build
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    if (!gpsSerial) {
        Serial.println("[GPS] ❌ Failed to initialize UART1");
        return false;
    }
    
    Serial.println("[GPS] ✅ UART1 initialized at 9600 baud");
    Serial.println("[GPS] Waiting for GPS fix (initial acquisition may take 30-60s)...");
    
    return true;
}

/**
 * Update GPS data from module
 * Reads available NMEA sentences and updates GPS fix
 * Returns true if new fix acquired this call
 */
bool updateGPS(GPSData* data) {
    bool newFix = false;
    
    // Read available bytes from GPS module
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        
        // Feed characters to TinyGPS++ parser
        if (gps.encode(c)) {
            // Complete sentence received
            
            // Check for new position fix
            if (gps.location.isUpdated()) {
                if (gps.location.isValid()) {
                    data->latitude = gps.location.lat();
                    data->longitude = gps.location.lng();
                    data->hasfix = true;
                    data->last_fix_ms = millis();
                    newFix = true;
                } else {
                    data->hasfix = false;
                }
            }
            
            // Update altitude if available
            if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
                data->altitude_m = gps.altitude.meters();
            }
            
            // Update satellite count if available
            if (gps.satellites.isUpdated()) {
                data->satellites = gps.satellites.value();
            }
            
            // Update HDOP (horizontal precision) if available
            if (gps.hdop.isUpdated() && gps.hdop.isValid()) {
                data->hdop = gps.hdop.hdop();
            }
        }
    }
    
    // Update current data
    currentGPS = *data;
    
    return newFix;
}

/**
 * Get current GPS data
 */
void getGPSData(GPSData* data) {
    *data = currentGPS;
}

/**
 * Check if GPS has a valid fix
 */
bool hasGPSFix() {
    return currentGPS.hasfix;
}

/**
 * Format GPS data as human-readable string
 */
String formatGPSString() {
    if (!currentGPS.hasfix) {
        return "No GPS fix (searching...)";
    }
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "GPS: %.6f,%.6f Alt:%.0fm Sats:%d HDOP:%.1f",
             currentGPS.latitude,
             currentGPS.longitude,
             currentGPS.altitude_m,
             currentGPS.satellites,
             currentGPS.hdop);
    
    return String(buffer);
}
