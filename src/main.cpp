#include <Arduino.h>
#include "device_config.h"
#include "lora_config.h"
#include "version.h"
#include "lora_protocol.h"
#include "sensor_manager.h"
#include "lora_comm.h"
#include "config_manager.h"

#ifdef OLED_ENABLED
#include "display_manager.h"
#endif

// Global state
uint16_t g_sequenceNumber = 0;
uint32_t g_wakeCount = 0;
uint64_t g_deviceId = 0;

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n\n====================================");
    Serial.println("ESP32 LoRa Sensor - Startup");
    Serial.println("====================================");
    Serial.printf("Firmware: %s\n", getFirmwareVersion().c_str());
    Serial.printf("Build: %s %s\n", BUILD_DATE, BUILD_TIME);

    // Get unique device ID from ESP32 chip ID
    g_deviceId = ESP.getEfuseMac();
    Serial.printf("Device ID: %016llX\n", g_deviceId);

    // Increment wake counter
    g_wakeCount++;

    // Initialize configuration manager (loads from SPIFFS)
    if (!initConfigManager()) {
        Serial.println("ERROR: Config initialization failed");
        // Continue anyway with defaults
    }

    // Check if user wants to enter config mode
    checkSerialConfig();

    // Initialize BME280 sensor
    Serial.println("\nInitializing BME280 sensor...");
    if (!initSensor()) {
        Serial.println("ERROR: BME280 initialization failed!");
        // TODO: Store error and retry on next wake
    }

    // Initialize LoRa radio
    Serial.println("Initializing LoRa radio...");
    if (!initLoRa()) {
        Serial.println("ERROR: LoRa initialization failed!");
        // TODO: Store error and retry on next wake
    }

#ifdef OLED_ENABLED
    // Initialize OLED display
    Serial.println("Initializing OLED display...");
    initDisplay();
    displayStartup(getFirmwareVersion().c_str());
    delay(2000);
#endif

    Serial.println("\nSetup complete!");
}

void loop() {
    Serial.println("\n--- Wake Cycle Start ---");
    Serial.printf("Wake count: %d\n", g_wakeCount);

    // Read sensor data
    Serial.println("Reading BME280 sensor...");
    ReadingsPayload readings;
    if (!readSensorData(&readings)) {
        Serial.println("ERROR: Failed to read sensor data");
        // TODO: Increment error counter
        // For now, continue with invalid data marked
    }

    // Display on OLED (if enabled)
#ifdef OLED_ENABLED
    displayReadings(&readings);
#endif

    // Build LoRa packet
    Serial.println("Building LoRa packet...");
    uint8_t packet[sizeof(LoRaPacketHeader) + sizeof(ReadingsPayload)];
    LoRaPacketHeader* header = (LoRaPacketHeader*)packet;

    initHeader(header, MSG_READINGS, g_deviceId, g_sequenceNumber++,
               sizeof(ReadingsPayload));

    memcpy(packet + sizeof(LoRaPacketHeader), &readings, sizeof(ReadingsPayload));

    // Transmit via LoRa with retries
    Serial.println("Transmitting packet...");
    bool txSuccess = transmitPacket(packet, sizeof(packet));

    if (txSuccess) {
        Serial.println("Transmission successful!");
        
        // Update display with TX stats
#ifdef OLED_ENABLED
        updateTxStats(g_sequenceNumber, getLastRSSI());
        displayReadings(&readings);
        delay(2000);  // Show results before sleep
#endif

        // Wait for ACK from gateway (unless battery critical)
        if (readings.batteryPercent > BATTERY_CRITICAL_PERCENT) {
            Serial.println("Waiting for ACK...");
            if (waitForAck(LORA_ACK_TIMEOUT_MS)) {
                Serial.println("ACK received!");
#ifdef OLED_ENABLED
                displayStatus("ACK OK!");
                delay(1000);
#endif
            } else {
                Serial.println("No ACK received (timeout)");
#ifdef OLED_ENABLED
                displayStatus("No ACK");
                delay(1000);
#endif
            }
        } else {
            Serial.println("Battery critical - skipping ACK wait");
        }
    } else {
        Serial.println("ERROR: All transmission attempts failed!");
#ifdef OLED_ENABLED
        displayError("TX Failed!");
        delay(2000);
#endif
    }

    // Check for incoming commands from gateway
    Serial.println("Checking for commands...");
    if (checkForCommands(LORA_COMMAND_TIMEOUT_MS)) {
        Serial.println("Command received and processed");
        // Commands are processed in checkForCommands()
    }

    // Check if restart was requested
    if (isRestartRequested()) {
        Serial.println("\n🔄 RESTARTING DEVICE...");
        Serial.flush();
        delay(500);
        ESP.restart();
    }

    // Print status
    Serial.println("\n--- Wake Cycle Complete ---");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("RSSI: %d dBm\n", getLastRSSI());
    Serial.printf("Next transmission in: %d seconds\n", 10);

    // Wait before next transmission (for testing)
    delay(10000);  // 10 seconds

    // Loop will repeat for continuous testing
}
