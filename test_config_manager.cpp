/**
 * Config Manager Test
 * Tests configuration loading/saving from LittleFS
 */

#include <Arduino.h>
#include <LittleFS.h>
#include "config_manager.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   CONFIG MANAGER TEST                 ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    // Test 1: Initialize config manager
    Serial.println("\n[TEST 1] Initialize Config Manager");
    if (initConfigManager()) {
        Serial.println("✅ Config manager initialized");
    } else {
        Serial.println("⚠️  Config manager initialized with warnings");
    }
    
    // Test 2: Read current configuration
    Serial.println("\n[TEST 2] Read Current Configuration");
    String deviceName = getDeviceName();
    uint32_t sleepSeconds = getDeepSleepSeconds();
    float baseline = getConfigPressureBaseline();
    
    Serial.println("Device Name: " + deviceName);
    Serial.println("Deep Sleep: " + String(sleepSeconds) + " seconds (" + String(sleepSeconds / 60) + " minutes)");
    Serial.println("Pressure Baseline: " + String(baseline, 2) + " hPa");
    
    // Test 3: Update settings
    Serial.println("\n[TEST 3] Update Settings");
    setDeviceName("TestDevice-123");
    setDeepSleepSeconds(600);  // 10 minutes
    setConfigPressureBaseline(1013.25);
    
    // Test 4: Verify updated values
    Serial.println("\n[TEST 4] Verify Updated Values");
    Serial.println("Device Name: " + getDeviceName());
    Serial.println("Deep Sleep: " + String(getDeepSleepSeconds()) + " seconds");
    Serial.println("Pressure Baseline: " + String(getConfigPressureBaseline(), 2) + " hPa");
    
    // Test 5: Reload configuration
    Serial.println("\n[TEST 5] Reload Configuration from Files");
    if (loadConfig()) {
        Serial.println("✅ Configuration reloaded successfully");
    }
    Serial.println("Device Name: " + getDeviceName());
    Serial.println("Deep Sleep: " + String(getDeepSleepSeconds()) + " seconds");
    Serial.println("Pressure Baseline: " + String(getConfigPressureBaseline(), 2) + " hPa");
    
    // Test 6: Restore original values
    Serial.println("\n[TEST 6] Restore Original Values");
    setDeviceName("BME280-LoRa-001");
    setDeepSleepSeconds(900);
    setConfigPressureBaseline(0.0);
    
    Serial.println("\n✅ All tests completed!");
    Serial.println("\nType 'config' or 'menu' to open interactive configuration menu");
}

void loop() {
    // Check for serial configuration commands
    checkSerialConfig();
    delay(100);
}
