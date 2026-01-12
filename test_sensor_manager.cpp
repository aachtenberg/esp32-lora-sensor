/**
 * Sensor Manager Test
 * Tests the sensor_manager module with ReadingsPayload structure
 */

#include <Arduino.h>
#include <LittleFS.h>
#include "device_config.h"
#include "lora_protocol.h"
#include "sensor_manager.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n================================");
    Serial.println("Sensor Manager Test");
    Serial.println("================================");

    // Initialize LittleFS for baseline storage
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[FS] LittleFS mounted");
    }

    // Initialize sensor
    if (!initSensor()) {
        Serial.println("[ERROR] Sensor initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("\n[SUCCESS] Sensor manager initialized!");
    Serial.println("Reading sensor every 5 seconds...\n");
}

void loop() {
    ReadingsPayload readings;

    Serial.println("--- Reading Sensor ---");

    if (readSensorData(&readings)) {
        // Decode and display payload values
        Serial.println("\nReadingsPayload structure:");
        Serial.printf("  timestamp: %u seconds\n", readings.timestamp);
        Serial.printf("  temperature: %d (%.2f °C)\n", readings.temperature, readings.temperature / 100.0);
        Serial.printf("  humidity: %u (%.2f %%)\n", readings.humidity, readings.humidity / 100.0);
        Serial.printf("  pressure: %u Pa (%.2f hPa)\n", readings.pressure, readings.pressure / 100.0);
        Serial.printf("  altitude: %d m\n", readings.altitude);
        Serial.printf("  batteryVoltage: %u (%.2f V)\n", readings.batteryVoltage, readings.batteryVoltage / 1000.0);
        Serial.printf("  batteryPercent: %u %%\n", readings.batteryPercent);

        if (readings.pressureChange != 0 || getPressureBaseline() > 0) {
            const char* trendStr = (readings.pressureTrend == 0) ? "falling" :
                                  (readings.pressureTrend == 2) ? "rising" : "steady";
            Serial.printf("  pressureChange: %d Pa (%.2f hPa)\n",
                         readings.pressureChange, readings.pressureChange / 100.0);
            Serial.printf("  pressureTrend: %u (%s)\n", readings.pressureTrend, trendStr);
        }

        // Show payload size for LoRa transmission
        Serial.printf("\nPayload size: %d bytes (LoRa-ready)\n", sizeof(ReadingsPayload));

    } else {
        Serial.println("[ERROR] Failed to read sensor!");
    }

    Serial.println();
    delay(5000);
}
