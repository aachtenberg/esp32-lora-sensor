#include "config_manager.h"
#include <LittleFS.h>

// Configuration variables
static String deviceName = "ESP32-LoRa";
static uint32_t deepSleepSeconds = 900;  // 15 minutes default
static float pressureBaseline = 0.0;

// File paths in LittleFS
static const char* DEVICE_NAME_FILE = "/device_name.txt";
static const char* DEEP_SLEEP_FILE = "/deep_sleep_seconds.txt";
static const char* PRESSURE_BASELINE_FILE = "/pressure_baseline.txt";

/**
 * Initialize configuration manager
 * Mounts LittleFS and loads configuration
 */
bool initConfigManager() {
    Serial.println("\n=== Config Manager Initialization ===");
    
    // Mount LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("❌ LittleFS mount failed!");
        return false;
    }
    Serial.println("✅ LittleFS mounted");
    
    // Load configuration from files
    bool success = loadConfig();
    
    Serial.println("Device Name: " + deviceName);
    Serial.println("Deep Sleep: " + String(deepSleepSeconds) + " seconds");
    Serial.println("Pressure Baseline: " + String(pressureBaseline, 2) + " hPa");
    Serial.println("=====================================\n");
    
    return success;
}

/**
 * Load configuration from LittleFS
 */
bool loadConfig() {
    bool allSuccess = true;
    
    // Load device name
    if (LittleFS.exists(DEVICE_NAME_FILE)) {
        File file = LittleFS.open(DEVICE_NAME_FILE, "r");
        if (file) {
            deviceName = file.readStringUntil('\n');
            deviceName.trim();
            file.close();
            Serial.println("✅ Loaded device name: " + deviceName);
        } else {
            Serial.println("⚠️  Failed to open device_name.txt");
            allSuccess = false;
        }
    } else {
        Serial.println("⚠️  device_name.txt not found, using default");
        allSuccess = false;
    }
    
    // Load deep sleep interval
    if (LittleFS.exists(DEEP_SLEEP_FILE)) {
        File file = LittleFS.open(DEEP_SLEEP_FILE, "r");
        if (file) {
            String value = file.readStringUntil('\n');
            deepSleepSeconds = value.toInt();
            file.close();
            Serial.println("✅ Loaded deep sleep: " + String(deepSleepSeconds) + " seconds");
        } else {
            Serial.println("⚠️  Failed to open deep_sleep_seconds.txt");
            allSuccess = false;
        }
    } else {
        Serial.println("⚠️  deep_sleep_seconds.txt not found, using default");
        allSuccess = false;
    }
    
    // Load pressure baseline
    if (LittleFS.exists(PRESSURE_BASELINE_FILE)) {
        File file = LittleFS.open(PRESSURE_BASELINE_FILE, "r");
        if (file) {
            String value = file.readStringUntil('\n');
            pressureBaseline = value.toFloat();
            file.close();
            Serial.println("✅ Loaded pressure baseline: " + String(pressureBaseline, 2) + " hPa");
        } else {
            Serial.println("⚠️  Failed to open pressure_baseline.txt");
            allSuccess = false;
        }
    } else {
        Serial.println("⚠️  pressure_baseline.txt not found, using default");
        allSuccess = false;
    }
    
    return allSuccess;
}

/**
 * Save configuration to LittleFS
 */
bool saveConfig() {
    bool allSuccess = true;
    
    // Save device name
    File file = LittleFS.open(DEVICE_NAME_FILE, "w");
    if (file) {
        file.println(deviceName);
        file.close();
        Serial.println("✅ Saved device name");
    } else {
        Serial.println("❌ Failed to save device name");
        allSuccess = false;
    }
    
    // Save deep sleep interval
    file = LittleFS.open(DEEP_SLEEP_FILE, "w");
    if (file) {
        file.println(deepSleepSeconds);
        file.close();
        Serial.println("✅ Saved deep sleep interval");
    } else {
        Serial.println("❌ Failed to save deep sleep interval");
        allSuccess = false;
    }
    
    // Save pressure baseline
    file = LittleFS.open(PRESSURE_BASELINE_FILE, "w");
    if (file) {
        file.println(pressureBaseline, 2);
        file.close();
        Serial.println("✅ Saved pressure baseline");
    } else {
        Serial.println("❌ Failed to save pressure baseline");
        allSuccess = false;
    }
    
    return allSuccess;
}

/**
 * Get device name
 */
String getDeviceName() {
    return deviceName;
}

/**
 * Set device name and persist to LittleFS
 */
void setDeviceName(const String& name) {
    deviceName = name;
    deviceName.trim();
    
    // Save to file
    File file = LittleFS.open(DEVICE_NAME_FILE, "w");
    if (file) {
        file.println(deviceName);
        file.close();
        Serial.println("✅ Device name updated: " + deviceName);
    } else {
        Serial.println("❌ Failed to save device name");
    }
}

/**
 * Get deep sleep interval in seconds
 */
uint32_t getDeepSleepSeconds() {
    return deepSleepSeconds;
}

/**
 * Set deep sleep interval and persist to LittleFS
 */
void setDeepSleepSeconds(uint32_t seconds) {
    deepSleepSeconds = seconds;
    
    // Save to file
    File file = LittleFS.open(DEEP_SLEEP_FILE, "w");
    if (file) {
        file.println(deepSleepSeconds);
        file.close();
        Serial.println("✅ Deep sleep updated: " + String(deepSleepSeconds) + " seconds");
    } else {
        Serial.println("❌ Failed to save deep sleep interval");
    }
}

/**
 * Get pressure baseline in hPa
 */
float getConfigPressureBaseline() {
    return pressureBaseline;
}

/**
 * Set pressure baseline and persist to LittleFS
 */
void setConfigPressureBaseline(float baseline) {
    pressureBaseline = baseline;
    
    // Save to file
    File file = LittleFS.open(PRESSURE_BASELINE_FILE, "w");
    if (file) {
        file.println(pressureBaseline, 2);
        file.close();
        Serial.println("✅ Pressure baseline updated: " + String(pressureBaseline, 2) + " hPa");
    } else {
        Serial.println("❌ Failed to save pressure baseline");
    }
}

/**
 * Interactive serial configuration menu
 */
void checkSerialConfig() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("config") || input.equalsIgnoreCase("menu")) {
            // Show menu
            Serial.println("\n========== Configuration Menu ==========");
            Serial.println("1. Set Device Name");
            Serial.println("2. Set Deep Sleep Interval (seconds)");
            Serial.println("3. Set Pressure Baseline (hPa)");
            Serial.println("4. Show Current Configuration");
            Serial.println("5. Save All Settings");
            Serial.println("6. Reload Configuration");
            Serial.println("Type 'exit' to close menu");
            Serial.println("========================================\n");
            Serial.print("Enter choice (1-6): ");
            
            // Wait for input
            while (!Serial.available()) {
                delay(10);
            }
            
            String choice = Serial.readStringUntil('\n');
            choice.trim();
            Serial.println(choice);
            
            if (choice == "1") {
                Serial.print("Enter device name: ");
                while (!Serial.available()) delay(10);
                String name = Serial.readStringUntil('\n');
                setDeviceName(name);
                
            } else if (choice == "2") {
                Serial.print("Enter deep sleep seconds: ");
                while (!Serial.available()) delay(10);
                String seconds = Serial.readStringUntil('\n');
                setDeepSleepSeconds(seconds.toInt());
                
            } else if (choice == "3") {
                Serial.print("Enter pressure baseline (hPa): ");
                while (!Serial.available()) delay(10);
                String baseline = Serial.readStringUntil('\n');
                setConfigPressureBaseline(baseline.toFloat());
                
            } else if (choice == "4") {
                Serial.println("\n--- Current Configuration ---");
                Serial.println("Device Name: " + deviceName);
                Serial.println("Deep Sleep: " + String(deepSleepSeconds) + " seconds (" + String(deepSleepSeconds / 60) + " minutes)");
                Serial.println("Pressure Baseline: " + String(pressureBaseline, 2) + " hPa");
                Serial.println("-----------------------------\n");
                
            } else if (choice == "5") {
                if (saveConfig()) {
                    Serial.println("✅ All settings saved!");
                } else {
                    Serial.println("⚠️  Some settings failed to save");
                }
                
            } else if (choice == "6") {
                if (loadConfig()) {
                    Serial.println("✅ Configuration reloaded!");
                } else {
                    Serial.println("⚠️  Some settings failed to load");
                }
            }
        }
    }
}
