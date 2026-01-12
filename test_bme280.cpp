/**
 * BME280 I2C Test Sketch
 * Tests BME280 connection on GPIO33 (SDA) and GPIO26 (SCL)
 *
 * Upload this to verify your wiring before running the full sensor code.
 *
 * Expected output:
 * - I2C scanner finds device at 0x76 or 0x77
 * - BME280 initialization succeeds
 * - Temperature, humidity, and pressure readings are displayed
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "device_config.h"

Adafruit_BME280 bme;
TwoWire I2C_BME = TwoWire(0);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n================================");
    Serial.println("BME280 I2C Test");
    Serial.println("================================");
    Serial.printf("SDA: GPIO%d (Header J2, pin 12)\n", BME280_SDA);
    Serial.printf("SCL: GPIO%d (Header J2, pin 15)\n", BME280_SCL);
    Serial.printf("Expected Address: 0x%02X\n", BME280_ADDR);
    Serial.println("================================\n");

    // Initialize I2C on custom pins
    I2C_BME.begin(BME280_SDA, BME280_SCL, 100000);
    delay(100);

    // Scan I2C bus
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++) {
        I2C_BME.beginTransmission(address);
        error = I2C_BME.endTransmission();

        if (error == 0) {
            Serial.printf("  Found device at address 0x%02X\n", address);
            nDevices++;
        }
    }

    if (nDevices == 0) {
        Serial.println("  ERROR: No I2C devices found!");
        Serial.println("\nTroubleshooting:");
        Serial.println("  1. Check wiring (VCC, GND, SDA, SCL)");
        Serial.println("  2. Verify BME280 is powered (3.3V)");
        Serial.println("  3. Check for loose connections");
        Serial.println("  4. Try address 0x77 if using different BME280 module");
    } else {
        Serial.printf("  Found %d device(s)\n", nDevices);
    }
    Serial.println();

    // Try to initialize BME280
    Serial.println("Initializing BME280...");

    if (!bme.begin(BME280_ADDR, &I2C_BME)) {
        Serial.println("  ERROR: Could not find BME280 sensor!");
        Serial.println("\nTrying alternate address 0x77...");

        if (!bme.begin(0x77, &I2C_BME)) {
            Serial.println("  ERROR: BME280 not found at 0x77 either!");
            Serial.println("\nFailed to initialize BME280. Check:");
            Serial.println("  - I2C address (0x76 or 0x77)");
            Serial.println("  - Wiring connections");
            Serial.println("  - Sensor power");
            while (1) delay(10);
        } else {
            Serial.println("  SUCCESS! BME280 found at address 0x77");
            Serial.println("  NOTE: Update BME280_ADDR to 0x77 in device_config.h");
        }
    } else {
        Serial.println("  SUCCESS! BME280 found at address 0x76");
    }

    // Configure BME280 sampling
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,   // temperature
                    Adafruit_BME280::SAMPLING_X16,  // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_X4,
                    Adafruit_BME280::STANDBY_MS_1000);

    Serial.println("\nBME280 configured successfully!");
    Serial.println("Reading sensor every 2 seconds...\n");
    delay(1000);
}

void loop() {
    // Read sensor
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;  // Convert Pa to hPa
    float altitude = bme.readAltitude(PRESSURE_SEA_LEVEL / 100.0F);

    // Print readings
    Serial.println("--- BME280 Readings ---");
    Serial.printf("Temperature: %.2f °C\n", temperature);
    Serial.printf("Humidity:    %.2f %%\n", humidity);
    Serial.printf("Pressure:    %.2f hPa\n", pressure);
    Serial.printf("Altitude:    %.2f m\n", altitude);

    // Sanity checks
    if (temperature < -40 || temperature > 85) {
        Serial.println("  WARNING: Temperature out of range!");
    }
    if (humidity < 0 || humidity > 100) {
        Serial.println("  WARNING: Humidity out of range!");
    }
    if (pressure < 300 || pressure > 1100) {
        Serial.println("  WARNING: Pressure out of range!");
    }

    Serial.println();
    delay(2000);
}
