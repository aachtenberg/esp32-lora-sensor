/**
 * LoRa Communication Test
 * Tests SX1262 LoRa radio initialization and transmission
 */

#include <Arduino.h>
#include <LittleFS.h>
#include "lora_comm.h"
#include "lora_protocol.h"
#include "sensor_manager.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   LORA COMMUNICATION TEST             ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    // Mount LittleFS for baseline storage
    Serial.println("\n[SETUP] Mounting LittleFS...");
    if (!LittleFS.begin(true)) {
        Serial.println("⚠️  LittleFS mount failed, continuing without filesystem");
    } else {
        Serial.println("✅ LittleFS mounted");
    }
    
    // Test 1: Initialize sensor manager (for test data)
    Serial.println("\n[TEST 1] Initialize Sensor Manager");
    if (initSensor()) {
        Serial.println("✅ Sensor manager initialized");
    } else {
        Serial.println("❌ Sensor manager failed - will use dummy data");
    }
    
    // Test 2: Initialize LoRa
    Serial.println("\n[TEST 2] Initialize LoRa Radio");
    if (initLoRa()) {
        Serial.println("✅ LoRa initialized successfully");
    } else {
        Serial.println("❌ LoRa initialization failed!");
        Serial.println("\nCheck wiring:");
        Serial.println("  SCK:  GPIO 18");
        Serial.println("  MISO: GPIO 19");
        Serial.println("  MOSI: GPIO 27");
        Serial.println("  NSS:  GPIO 5");
        Serial.println("  DIO1: GPIO 34");
        Serial.println("  RST:  GPIO 14");
        Serial.println("  BUSY: GPIO 35");
        while (1) delay(1000);
    }
    
    delay(1000);
    
    // Test 3: Read sensor data
    Serial.println("\n[TEST 3] Read Sensor Data");
    ReadingsPayload readings;
    
    if (readSensorData(&readings)) {
        Serial.println("✅ Sensor data read successfully");
        Serial.printf("  Temperature: %.2f °C\n", readings.temperature / 100.0);
        Serial.printf("  Humidity: %.2f %%\n", readings.humidity / 100.0);
        Serial.printf("  Pressure: %.2f hPa\n", readings.pressure / 100.0);
        Serial.printf("  Altitude: %d m\n", readings.altitude);
    } else {
        Serial.println("⚠️  Using dummy sensor data for test");
        readings.timestamp = millis() / 1000;
        readings.temperature = 2123; // 21.23°C
        readings.humidity = 5512;    // 55.12%
        readings.pressure = 101325;  // 1013.25 hPa
        readings.altitude = 100;
        readings.batteryVoltage = 3700;  // 3.7V
        readings.batteryPercent = 75;
        readings.pressureChange = 0;
        readings.pressureTrend = 1;
    }
    
    // Test 4: Transmit readings packet
    Serial.println("\n[TEST 4] Transmit Readings Packet");
    bool txSuccess = transmitPacket((uint8_t*)&readings, sizeof(readings));
    
    if (txSuccess) {
        Serial.println("✅ Transmission successful!");
        Serial.printf("  RSSI: %d dBm\n", getLastRSSI());
        Serial.printf("  SNR: %d dB\n", getLastSNR());
    } else {
        Serial.println("❌ Transmission failed!");
    }
    
    // Test 5: Wait for ACK (optional - may timeout if no gateway present)
    Serial.println("\n[TEST 5] Wait for ACK from Gateway");
    Serial.println("⚠️  This will timeout if no gateway is listening");
    
    if (waitForAck(2000)) {
        Serial.println("✅ ACK received from gateway!");
    } else {
        Serial.println("⏱️  No ACK received (this is normal if no gateway present)");
    }
    
    // Test 6: Check for commands (optional - may timeout if no gateway present)
    Serial.println("\n[TEST 6] Check for Commands");
    Serial.println("⚠️  This will timeout if no gateway is sending commands");
    
    if (checkForCommands(1000)) {
        Serial.println("✅ Command received!");
    } else {
        Serial.println("⏱️  No commands received (this is normal)");
    }
    
    Serial.println("\n✅ All tests completed!");
    Serial.println("\nRepeating transmission every 10 seconds...");
    Serial.println("(This simulates the main loop behavior)");
}

void loop() {
    delay(10000);  // 10 second interval for testing
    
    Serial.println("\n--- Transmitting Readings ---");
    
    // Read fresh sensor data
    ReadingsPayload readings;
    if (readSensorData(&readings)) {
        Serial.printf("Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2fhPa\n",
                     readings.temperature / 100.0,
                     readings.humidity / 100.0,
                     readings.pressure / 100.0);
        
        // Transmit
        if (transmitPacket((uint8_t*)&readings, sizeof(readings))) {
            Serial.printf("✅ TX OK (RSSI: %d dBm, SNR: %d dB)\n",
                        getLastRSSI(), getLastSNR());
            
            // Optionally wait for ACK
            waitForAck(2000);
        } else {
            Serial.println("❌ TX Failed");
        }
    } else {
        Serial.println("❌ Sensor read failed");
    }
}
