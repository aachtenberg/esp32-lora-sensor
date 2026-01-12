#include "display_manager.h"

#ifdef OLED_ENABLED

#include "device_config.h"
#include <U8g2lib.h>
#include <Wire.h>

// OLED display instance (128x64 SSD1306)
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C* display = nullptr;

// Display state
static uint32_t txCount = 0;
static int16_t lastRssi = 0;

/**
 * Initialize OLED display
 */
void initDisplay() {
    Serial.print("Checking for I2C display... ");

    // Ensure peripherals are powered (Heltec V3: OLED is on VEXT)
    // Note: VEXT might already be on from LoRa init, but we ensure it here
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    
    // Manually reset OLED
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(50);
    digitalWrite(OLED_RST, HIGH);
    delay(50);
    
    // Initialize I2C
    Wire.begin(OLED_SDA, OLED_SCL);
    // Wire.setClock(100000); 
    delay(100);
    
    // Check if display is present
    Wire.beginTransmission(OLED_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("Warning: Display not responding to I2C scan (attempting init anyway)");
    } else {
        Serial.println("Found!");
    }
    
    // Create display instance
    // Configure U8g2 to use the existing Wire instance (initialized above)
    // We pass U8X8_PIN_NONE for clock/data to prevent U8g2 from re-initializing Wire with defaults
    display = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, OLED_RST);
    
    // Set I2C address
    display->setI2CAddress(OLED_ADDR * 2); // U8g2 uses 8-bit address
    
    // Initialize display
    Serial.print("Starting display driver... ");
    if (!display->begin()) {
        Serial.println("❌ Failed");
        delete display;
        display = nullptr;
        return;
    }
    Serial.println("✅");
    
    display->clearBuffer();
    display->setFont(u8g2_font_ncenB08_tr);
    
    Serial.println("✅ Display initialized");
}

/**
 * Display startup screen
 */
void displayStartup(const char* version) {
    if (display == nullptr) return;
    
    display->clearBuffer();
    
    // Title
    display->setFont(u8g2_font_ncenB10_tr);
    display->drawStr(15, 15, "LoRa Sensor");
    
    // Version
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(20, 35, version);
    
    // Status
    display->drawStr(30, 55, "Starting...");
    
    display->sendBuffer();
}

/**
 * Display sensor readings
 */
void displayReadings(const ReadingsPayload* readings) {
    if (display == nullptr || readings == nullptr) return;
    
    display->clearBuffer();
    display->setFont(u8g2_font_6x10_tr);
    
    // Temperature
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "T: %.1fC", readings->temperature / 100.0);
    display->drawStr(0, 10, buffer);
    
    // Humidity
    snprintf(buffer, sizeof(buffer), "H: %.1f%%", readings->humidity / 100.0);
    display->drawStr(0, 22, buffer);
    
    // Pressure
    snprintf(buffer, sizeof(buffer), "P: %.0fhPa", readings->pressure / 100.0);
    display->drawStr(0, 34, buffer);
    
    // Battery
    snprintf(buffer, sizeof(buffer), "Bat: %.2fV (%d%%)", 
             readings->batteryVoltage / 1000.0, readings->batteryPercent);
    display->drawStr(0, 46, buffer);
    
    // TX count and RSSI
    snprintf(buffer, sizeof(buffer), "TX:%lu RSSI:%d", txCount, lastRssi);
    display->drawStr(0, 58, buffer);
    
    display->sendBuffer();
}

/**
 * Display status message
 */
void displayStatus(const char* message) {
    if (display == nullptr || message == nullptr) return;
    
    display->clearBuffer();
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(0, 32, message);
    display->sendBuffer();
}

/**
 * Display error message
 */
void displayError(const char* error) {
    if (display == nullptr || error == nullptr) return;
    
    display->clearBuffer();
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(0, 20, "ERROR:");
    display->drawStr(0, 35, error);
    display->sendBuffer();
}

/**
 * Update TX statistics for display
 */
void updateTxStats(uint32_t count, int16_t rssi) {
    txCount = count;
    lastRssi = rssi;
}

/**
 * Clear display
 */
void clearDisplay() {
    if (display == nullptr) return;
    display->clearBuffer();
    display->sendBuffer();
}

#else

// Stub implementations when OLED is disabled
void initDisplay() {}
void displayStartup(const char* version) {}
void displayReadings(const ReadingsPayload* readings) {}
void displayStatus(const char* message) {}
void displayError(const char* error) {}
void updateTxStats(uint32_t count, int16_t rssi) {}
void clearDisplay() {}

#endif // OLED_ENABLED
