#include "gps_manager.h"
#include "device_config.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

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
 * Auto-detects baud rate (some NEO-6M clones use 115200 instead of 9600)
 */
bool initGPS() {
    Serial.println("[GPS] Initializing NEO-6M GPS module...");
    Serial.printf("[GPS] RX=GPIO%d, TX=GPIO%d\n", GPS_RX_PIN, GPS_TX_PIN);
    Serial.flush();
    
    // Try only 9600 baud first (most common) with very short timeout
    uint32_t baud = 9600;
    Serial.printf("[GPS] Trying %d baud...\n", baud);
    Serial.flush();
    
    gpsSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Very short test - 500ms only
    unsigned long start = millis();
    int bytesReceived = 0;
    
    while (millis() - start < 500) {
        yield();  // Feed watchdog every loop
        
        while (gpsSerial.available() > 0) {
            char c = gpsSerial.read();
            bytesReceived++;
            
            if (gps.encode(c)) {
                Serial.printf("[GPS] ✅ Found GPS at %d baud! (received %d bytes)\n", baud, bytesReceived);
                Serial.flush();
                return true;
            }
            
            yield();  // Feed watchdog after each byte
        }
        
        delay(10);  // Small delay between checks
    }
    
    Serial.printf("[GPS] No data at %d baud (received %d bytes)\n", baud, bytesReceived);
    Serial.flush();
    
    // Keep serial open anyway - GPS might start transmitting later
    Serial.println("[GPS] ⚠️ GPS not detected, but keeping serial open");
    Serial.flush();
    return false;  // GPS not found but serial is open
}

/**
 * Update GPS data from module
 * Reads available NMEA sentences and updates GPS fix
 * Returns true if new fix acquired this call
 */
bool updateGPS(GPSData* data) {
    bool newFix = false;
    int bytesRead = 0;
    int sentencesDecoded = 0;
    
    // Check how many bytes are available
    int available = gpsSerial.available();
    
    // Read available bytes from GPS module
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        bytesRead++;
        
        // Feed characters to TinyGPS++ parser
        if (gps.encode(c)) {
            // Complete sentence received and parsed
            sentencesDecoded++;
            
            // Check for new position fix
            if (gps.location.isUpdated()) {
                Serial.printf("[GPS] Location updated: valid=%d lat=%.6f lon=%.6f\n",
                             gps.location.isValid(), gps.location.lat(), gps.location.lng());
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
                Serial.printf("[GPS] Satellites visible: %d\n", data->satellites);
            }
            
            // Update HDOP (horizontal precision) if available
            if (gps.hdop.isUpdated() && gps.hdop.isValid()) {
                data->hdop = gps.hdop.hdop();
            }
        }
    }
    
    // Log GPS activity (even without fix)
    if (bytesRead > 0) {
        Serial.printf("[GPS] Read %d bytes (had %d available), decoded %d sentences. Sats: %d, HDOP: %.1f, HasFix: %d\n",
                     bytesRead, available, sentencesDecoded, gps.satellites.value(), 
                     gps.hdop.isValid() ? gps.hdop.hdop() : 999.0, data->hasfix);
    } else {
        Serial.println("[GPS] ⚠️  No data available from GPS module - check wiring!");
    }
    
    // Report failed sentences for debugging
    if (gps.failedChecksum() > 0) {
        Serial.printf("[GPS] ⚠️  Failed checksums: %d (possible noise/wiring issue)\n", gps.failedChecksum());
    }
    
    // Report characters processed
    Serial.printf("[GPS] Total chars processed: %d, sentences with fix: %d\n", 
                 gps.charsProcessed(), gps.sentencesWithFix());
    
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
        char buffer[80];
        snprintf(buffer, sizeof(buffer),
                 "No GPS fix (Sats:%d HDOP:%.1f)",
                 currentGPS.satellites,
                 currentGPS.hdop);
        return String(buffer);
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
