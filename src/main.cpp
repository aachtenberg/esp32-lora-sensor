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

#ifdef GPS_ENABLED
#include "gps_manager.h"
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

#ifdef GPS_ENABLED
    // Initialize GPS module
    Serial.println("Initializing GPS module...");
    initGPS();
    Serial.println("GPS initialization complete");
#endif

#ifdef OLED_ENABLED
    // Initialize OLED display
    Serial.println("Initializing OLED display...");
    initDisplay();
    setDisplayDeviceId(g_deviceId);
    displayStartup(getFirmwareVersion().c_str());
    delay(2000);
#endif

    Serial.println("\nSetup complete!");
    
    // Send startup event
    sendEventMessage(EVENT_STARTUP, SEVERITY_INFO, "Device started");
}

void loop() {
    Serial.println("\n--- Wake Cycle Start ---");
    Serial.printf("Wake count: %d\n", g_wakeCount);
    
    // Send status message every 5 wake cycles (for faster name propagation)
    if (g_wakeCount > 0 && g_wakeCount % 5 == 0) {
        sendStatusMessage();
        delay(500);  // Brief delay between messages
    }

    // Read sensor data
    Serial.println("Reading BME280 sensor...");
    ReadingsPayload readings;
    if (!readSensorData(&readings)) {
        Serial.println("ERROR: Failed to read sensor data");
        // TODO: Increment error counter
        // For now, continue with invalid data marked
    }

#ifdef GPS_ENABLED
    // Update GPS and collect location data (if available)
    Serial.println("Updating GPS location...");
    GPSData gpsData;
    if (updateGPS(&gpsData)) {
        Serial.printf("GPS: %.6f, %.6f (alt: %.1f m, sats: %d, HDOP: %.1f)\n",
                     gpsData.latitude, gpsData.longitude, 
                     gpsData.altitude_m, gpsData.satellites, gpsData.hdop);
        // TODO: Add GPS data to readings payload for transmission
    } else {
        Serial.println("GPS: No fix yet");
    }
    
    // Display GPS status on OLED
#ifdef OLED_ENABLED
    displayGPS(gpsData.satellites, gpsData.hasfix, 
               gpsData.latitude, gpsData.longitude, 
               gpsData.altitude_m, gpsData.hdop);
    delay(3000);  // Show GPS info for 3 seconds
#endif
#endif

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
    uint32_t txStartMs = millis();
    Serial.printf("[%lu] Transmitting packet...\n", txStartMs);
    bool txSuccess = transmitPacket(packet, sizeof(packet));
    uint32_t txEndMs = millis();

    if (txSuccess) {
        Serial.printf("[%lu] ✅ Transmission successful! (took %lums)\n", txEndMs, txEndMs - txStartMs);
        
        // Update display with TX stats
#ifdef OLED_ENABLED
        updateTxStats(g_sequenceNumber, getLastRSSI());
        displayReadings(&readings);
        delay(2000);  // Show results before sleep
#endif

        // Wait for ACK from gateway (unless battery critical)
        if (readings.batteryPercent > BATTERY_CRITICAL_PERCENT) {
            Serial.printf("[%lu] Waiting for ACK...\n", millis());
            if (waitForAck(LORA_ACK_TIMEOUT_MS)) {
                Serial.printf("[%lu] ACK received!\n", millis());
#ifdef OLED_ENABLED
                displayStatus("ACK OK!");
                delay(1000);
#endif
            } else {
                Serial.printf("[%lu] No ACK received (timeout)\n", millis());
#ifdef OLED_ENABLED
                displayStatus("No ACK");
                delay(1000);
#endif
            }
        } else {
            Serial.printf("[%lu] Battery critical - skipping ACK wait\n", millis());
        }
    } else {
        Serial.printf("[%lu] ERROR: All transmission attempts failed!\n", millis());
#ifdef OLED_ENABLED
        displayError("TX Failed!");
        delay(2000);
#endif
    }

    // Check for incoming commands from gateway
    uint32_t rxReadyMs = millis();
    Serial.printf("[%lu] Checking for commands...\n", rxReadyMs);
    if (checkForCommands(LORA_COMMAND_TIMEOUT_MS)) {
        Serial.printf("[%lu] Command received and processed\n", millis());
        // Commands are processed in checkForCommands()
    }

    // Check if restart was requested
    if (isRestartRequested()) {
        Serial.printf("[%lu] 🔄 RESTARTING DEVICE...\n", millis());
        Serial.flush();
        delay(500);
        ESP.restart();
    }

    // Display configuration screen
#ifdef OLED_ENABLED
    displayConfig(getDeepSleepSeconds(), getSensorIntervalSeconds(), g_wakeCount);
    delay(3000);  // Show config for 3 seconds
#endif

    // Print status
    Serial.printf("[%lu] --- Wake Cycle Complete ---\n", millis());
    Serial.printf("[%lu] Free heap: %d bytes\n", millis(), ESP.getFreeHeap());
    Serial.printf("[%lu] RSSI: %d dBm\n", millis(), getLastRSSI());
    
    // Get configured sensor interval
    uint16_t intervalSeconds = getSensorIntervalSeconds();
    Serial.printf("Next transmission in: %d seconds\n", intervalSeconds);

    // Wait before next transmission using configured interval
    delay(intervalSeconds * 1000);
    
    // Increment wake count for next cycle
    g_wakeCount++;

    // Loop will repeat for continuous testing
}
