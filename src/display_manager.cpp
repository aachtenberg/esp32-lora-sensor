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
static uint64_t g_deviceId = 0;  // Device ID passed from main.cpp

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
    display->setFont(u8g2_font_5x7_tf);
    
    // Temperature (larger)
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "T:%.1fC", readings->temperature / 100.0);
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(0, 11, buffer);
    
    // Humidity and Pressure (small)
    display->setFont(u8g2_font_5x7_tf);
    snprintf(buffer, sizeof(buffer), "H:%.0f%% P:%.0fhPa", readings->humidity / 100.0, readings->pressure / 100.0);
    display->drawStr(0, 21, buffer);
    
    // Battery voltage and percent
    snprintf(buffer, sizeof(buffer), "Bat:%.2fV (%d%%)", readings->batteryVoltage / 1000.0, readings->batteryPercent);
    display->drawStr(0, 31, buffer);
    
    // TX status, RSSI, and wake count
    snprintf(buffer, sizeof(buffer), "TX:%lu RSSI:%d", txCount, lastRssi);
    display->drawStr(0, 41, buffer);
    
    // Device ID (truncated)
    snprintf(buffer, sizeof(buffer), "ID:...%04llX", (unsigned long long)(g_deviceId & 0xFFFF));
    display->drawStr(0, 51, buffer);
    
    // Status line (frequency or uptime)
    display->drawStr(0, 61, "OK");
    
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
 * Set device ID for display
 */
void setDisplayDeviceId(uint64_t deviceId) {
    g_deviceId = deviceId;
}

/**
 * Display configuration and status
 */
void displayConfig(uint32_t sleepSeconds, uint32_t intervalSeconds, uint32_t wakeCount) {
    if (display == nullptr) return;
    
    display->clearBuffer();
    display->setFont(u8g2_font_5x7_tf);
    
    char buffer[32];
    
    // Title
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(35, 12, "Config");
    
    // Settings
    display->setFont(u8g2_font_5x7_tf);
    snprintf(buffer, sizeof(buffer), "Sleep:%lds", sleepSeconds);
    display->drawStr(0, 24, buffer);
    
    snprintf(buffer, sizeof(buffer), "Interval:%lds", intervalSeconds);
    display->drawStr(0, 34, buffer);
    
    snprintf(buffer, sizeof(buffer), "Wake:%lu", wakeCount);
    display->drawStr(0, 44, buffer);
    
    snprintf(buffer, sizeof(buffer), "Heap:%lu KB", ESP.getFreeHeap() / 1024);
    display->drawStr(0, 54, buffer);
    
    // Status indicator
    display->drawStr(0, 64, "Ready");
    
    display->sendBuffer();
}
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
