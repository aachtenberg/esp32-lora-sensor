#include <Arduino.h>
#include "device_config.h"
#include "lora_config.h"
#include "version.h"
#include "lora_protocol.h"
#include "sensor_manager.h"
#include "lora_comm.h"
#include "config_manager.h"
#include "power_manager.h"

#ifdef OLED_ENABLED
#include "display_manager.h"
#endif

#ifdef GPS_ENABLED
#include "gps_manager.h"
#endif

// Global state (RTC memory survives deep sleep)
RTC_DATA_ATTR uint16_t g_sequenceNumber = 0;
RTC_DATA_ATTR uint32_t g_wakeCount = 0;
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

    // Initialize selected sensor
    Serial.println("\nInitializing sensor...");
    if (!initSensor()) {
        Serial.println("ERROR: Sensor initialization failed!");
        // TODO: Store error and retry on next wake
    }

    // Initialize LoRa radio
    Serial.println("Initializing LoRa radio...");
    if (!initLoRa()) {
        Serial.println("ERROR: LoRa initialization failed!");
        // TODO: Store error and retry on next wake
    }

    // Initialize power manager (battery monitoring)
    initPowerManager();

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
    Serial.println("Reading sensor data...");
    ReadingsPayload readings = {};
    bool sensorReadOk = readSensorData(&readings);
    if (!sensorReadOk) {
        Serial.println("ERROR: Failed to read sensor data");
        readings.timestamp = millis() / 1000;
        readings.pressureTrend = 1;
    }

    // Read battery status
    float batteryV = readBatteryVoltage();
    readings.batteryVoltage = (uint16_t)(batteryV * 1000);
    readings.batteryPercent = calculateBatteryPercent(batteryV);
    Serial.printf("Battery: %dmV (%d%%)\n", readings.batteryVoltage, readings.batteryPercent);

#ifdef GPS_ENABLED
    // Update GPS and collect location data (if available)
    Serial.println("Updating GPS location...");
    GPSData gpsData;
    getGPSData(&gpsData);  // Initialize with last known state (avoids garbage values)
    updateGPS(&gpsData);   // Update with any new data from GPS module
    
    // Add GPS data to readings (even if no fix - shows 0,0 when invalid)
    if (gpsData.hasfix) {
        Serial.printf("GPS: %.6f, %.6f (alt: %.1f m, sats: %d, HDOP: %.1f)\n",
                     gpsData.latitude, gpsData.longitude, 
                     gpsData.altitude_m, gpsData.satellites, gpsData.hdop);
        readings.gpsLatitude = (int32_t)(gpsData.latitude * 1000000.0);
        readings.gpsLongitude = (int32_t)(gpsData.longitude * 1000000.0);
        readings.gpsAltitude = (int16_t)gpsData.altitude_m;
        readings.gpsSatellites = gpsData.satellites;
        readings.gpsHdop = (uint16_t)(gpsData.hdop * 10.0);
    } else {
        Serial.println("GPS: No fix yet");
        // Set GPS fields to 0 when no fix
        readings.gpsLatitude = 0;
        readings.gpsLongitude = 0;
        readings.gpsAltitude = 0;
        readings.gpsSatellites = gpsData.satellites;  // Still show satellite count
        readings.gpsHdop = (uint16_t)(gpsData.hdop * 10.0);
    }
    
    // Display GPS status on OLED
#ifdef OLED_ENABLED
    displayGPS(gpsData.satellites, gpsData.hasfix, 
               gpsData.latitude, gpsData.longitude, 
               gpsData.altitude_m, gpsData.hdop);
    delay(3000);  // Show GPS info for 3 seconds
#endif
#else
    // No GPS support - set GPS fields to 0
    readings.gpsLatitude = 0;
    readings.gpsLongitude = 0;
    readings.gpsAltitude = 0;
    readings.gpsSatellites = 0;
    readings.gpsHdop = 0;
#endif

    // Display on OLED (if enabled)
#ifdef OLED_ENABLED
    if (sensorReadOk) {
        displayReadings(&readings);
    } else {
        displayError("Sensor Read Failed");
    }
#endif

    if (sensorReadOk) {
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
    } else {
        Serial.println("Skipping readings transmit due to sensor read failure");
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

    // Display battery screen briefly
#ifdef OLED_ENABLED
    displayBattery(readings.batteryVoltage, readings.batteryPercent);
    delay(5000);  // Show battery screen for 5 seconds
    displayReadings(&readings);  // Switch back to readings
#endif

    // Print status
    Serial.printf("[%lu] --- Wake Cycle Complete ---\n", millis());
    Serial.printf("[%lu] Free heap: %d bytes\n", millis(), ESP.getFreeHeap());
    Serial.printf("[%lu] RSSI: %d dBm\n", millis(), getLastRSSI());
    
    // Get configured deep sleep interval
    uint32_t sleepSeconds = getDeepSleepSeconds();
    
    // DEBUG: Skip deep sleep if seconds is 0 or very short (loop mode)
    if (sleepSeconds == 0) {
        Serial.println("[POWER] Deep sleep disabled (0s) - using loop mode");
        delay(30000);  // Wait 30 seconds then loop
        g_wakeCount++;
        return;  // Loop back
    }
    
    Serial.printf("Entering deep sleep for %lu seconds...\n", sleepSeconds);

    // Turn off display before sleep
#ifdef OLED_ENABLED
    displayOff();
#endif

    // Enter deep sleep (will wake and restart)
    enterDeepSleep(sleepSeconds);
    
    // Code below never runs - device restarts after deep sleep
}
