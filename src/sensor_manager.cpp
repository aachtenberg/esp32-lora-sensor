/**
 * Sensor Manager - Multi-sensor support
 * Supports BME280 environmental sensor, DHT22 temperature/humidity sensor, or DS18B20 temperature sensor
 * Selected via compile-time flag: SENSOR_TYPE_BME280, SENSOR_TYPE_DHT22, or SENSOR_TYPE_DS18B20
 */

#include "sensor_manager.h"
#include "device_config.h"
#include "lora_comm.h"
#include <FS.h>
#include <LittleFS.h>

// Sensor failure counter (shared)
static uint16_t sensorFailures = 0;

// ====================================================================
// DS18B20 Temperature Sensor Implementation
// ====================================================================
#ifdef SENSOR_TYPE_DS18B20

#include <OneWire.h>
#include <DallasTemperature.h>

// OneWire bus and DallasTemperature sensor
static OneWire oneWire(DS18B20_PIN);
static DallasTemperature ds18b20(&oneWire);

// Forward declaration
static void resetOneWire();

static void resetOneWire() {
    Serial.println("[SENSOR] Resetting 1-Wire bus...");
    digitalWrite(VEXT_CTRL, HIGH); // Power off
    delay(200);
    digitalWrite(VEXT_CTRL, LOW);  // Power on
    delay(100);
    ds18b20.begin();
    delay(50);
}

bool initSensor() {
    Serial.println("[SENSOR] Initializing DS18B20...");

    ds18b20.begin();
    delay(100);

    int deviceCount = ds18b20.getDeviceCount();
    if (deviceCount == 0) {
        Serial.printf("[SENSOR] ERROR: No DS18B20 sensors found on GPIO%d!\n", DS18B20_PIN);
        Serial.println("[SENSOR] Check wiring: Data pin with 4.7K pull-up to 3.3V");
        return false;
    }

    // Set resolution to 12-bit (default, highest precision)
    ds18b20.setResolution(12);

    Serial.printf("[SENSOR] DS18B20 initialized: %d sensor(s) found on GPIO%d\n",
                  deviceCount, DS18B20_PIN);

    return true;
}

bool readSensorData(ReadingsPayload* readings) {
    if (!readings) {
        Serial.println("[SENSOR] ERROR: NULL readings pointer");
        return false;
    }

    // Request temperature conversion
    ds18b20.requestTemperatures();

    // Read temperature (blocking, but fast with 12-bit resolution ~750ms)
    float tempC = ds18b20.getTempCByIndex(0);

    // Check for read error
    if (tempC == DEVICE_DISCONNECTED_C) {
        Serial.println("[SENSOR] ERROR: DS18B20 read failed (disconnected)");
        sensorFailures++;

        // Attempt recovery
        resetOneWire();
        ds18b20.requestTemperatures();
        tempC = ds18b20.getTempCByIndex(0);

        if (tempC == DEVICE_DISCONNECTED_C) {
            Serial.println("[SENSOR] ERROR: Recovery failed");
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "DS18B20 disconnected - recovery failed");
            return false;
        }
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_WARNING, "DS18B20 recovered");
    }

    // Validate temperature range (-55 to +125°C is DS18B20 spec)
    if (tempC < -55.0 || tempC > 125.0) {
        Serial.printf("[SENSOR] ERROR: Invalid temperature: %.2f°C\n", tempC);
        sensorFailures++;
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "DS18B20 invalid reading");
        return false;
    }

    // Populate payload
    readings->timestamp = millis() / 1000;  // Uptime in seconds
    readings->temperature = (int16_t)(tempC * 100.0);  // °C * 100

    // DS18B20 doesn't have humidity, pressure, or altitude
    readings->humidity = 0;
    readings->pressure = 0;
    readings->altitude = 0;
    readings->pressureChange = 0;
    readings->pressureTrend = 1;  // Steady (N/A for DS18B20)

    // Battery will be filled by power_manager
    readings->batteryVoltage = 0;
    readings->batteryPercent = 0;

    // Debug output
    Serial.println("[SENSOR] DS18B20 Reading:");
    Serial.printf("  Temperature: %.2f°C\n", tempC);

    return true;
}

uint16_t getSensorFailures() {
    return sensorFailures;
}

// ====================================================================
// DHT22 Temperature and Humidity Sensor Implementation
// ====================================================================
#elif defined(SENSOR_TYPE_DHT22)

#include <DHT.h>

// DHT22 sensor instance
static DHT dht22(DHT22_PIN, DHT22);

// Forward declaration
static void resetDHT22();

static void resetDHT22() {
    Serial.println("[SENSOR] Resetting DHT22 power...");
    digitalWrite(VEXT_CTRL, HIGH); // Power off
    delay(500);  // DHT22 needs longer recovery time
    digitalWrite(VEXT_CTRL, LOW);  // Power on
    delay(1000); // DHT22 needs 1 second to stabilize
    dht22.begin();
}

bool initSensor() {
    Serial.println("[SENSOR] Initializing DHT22...");

    dht22.begin();
    delay(2000);  // DHT22 needs 2 seconds initial stabilization

    // Try a test read to verify sensor is connected
    float testTemp = dht22.readTemperature();
    if (isnan(testTemp)) {
        Serial.printf("[SENSOR] ERROR: DHT22 not responding on GPIO%d!\n", DHT22_PIN);
        Serial.println("[SENSOR] Check wiring: Data pin to GPIO, VCC to 3.3V, GND to GND");
        Serial.println("[SENSOR] Note: DHT22 has built-in pull-up, no external resistor needed");
        return false;
    }

    Serial.printf("[SENSOR] DHT22 initialized on GPIO%d\n", DHT22_PIN);
    Serial.printf("[SENSOR] Test reading: %.2f°C\n", testTemp);

    return true;
}

bool readSensorData(ReadingsPayload* readings) {
    if (!readings) {
        Serial.println("[SENSOR] ERROR: NULL readings pointer");
        return false;
    }

    // Read temperature and humidity
    // Note: DHT22 has 2-second minimum sampling period
    float tempC = dht22.readTemperature();
    float humidity = dht22.readHumidity();

    // Check for read errors
    if (isnan(tempC) || isnan(humidity)) {
        Serial.println("[SENSOR] ERROR: DHT22 read failed (NaN)");
        sensorFailures++;

        // Attempt recovery
        resetDHT22();
        delay(2000);  // Wait for sensor to stabilize
        
        tempC = dht22.readTemperature();
        humidity = dht22.readHumidity();

        if (isnan(tempC) || isnan(humidity)) {
            Serial.println("[SENSOR] ERROR: Recovery failed");
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "DHT22 read failed - recovery failed");
            return false;
        }
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_WARNING, "DHT22 recovered");
    }

    // Validate temperature range (-40 to +80°C is DHT22 spec)
    if (tempC < -40.0 || tempC > 80.0) {
        Serial.printf("[SENSOR] ERROR: Invalid temperature: %.2f°C\n", tempC);
        sensorFailures++;
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "DHT22 invalid temperature");
        return false;
    }

    // Validate humidity range (0-100% is DHT22 spec)
    if (humidity < 0.0 || humidity > 100.0) {
        Serial.printf("[SENSOR] ERROR: Invalid humidity: %.2f%%\n", humidity);
        sensorFailures++;
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "DHT22 invalid humidity");
        return false;
    }

    // Populate payload
    readings->timestamp = millis() / 1000;  // Uptime in seconds
    readings->temperature = (int16_t)(tempC * 100.0);  // °C * 100
    readings->humidity = (uint16_t)(humidity * 100.0);  // % * 100

    // DHT22 doesn't have pressure or altitude
    readings->pressure = 0;
    readings->altitude = 0;
    readings->pressureChange = 0;
    readings->pressureTrend = 1;  // Steady (N/A for DHT22)

    // Battery will be filled by power_manager
    readings->batteryVoltage = 0;
    readings->batteryPercent = 0;

    // Debug output
    Serial.println("[SENSOR] DHT22 Reading:");
    Serial.printf("  Temperature: %.2f°C\n", tempC);
    Serial.printf("  Humidity: %.2f%%\n", humidity);

    return true;
}

uint16_t getSensorFailures() {
    return sensorFailures;
}

// ====================================================================
// BME280 Environmental Sensor Implementation
// ====================================================================
#else // SENSOR_TYPE_BME280 (default)

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// BME280 sensor instance
Adafruit_BME280 bme280;

// Custom I2C bus for BME280 (separate from built-in OLED)
// Use I2C1 (bus 1) to avoid conflict with OLED on I2C0 (bus 0/Wire)
TwoWire I2C_BME = TwoWire(1);

// Pressure baseline for weather tracking
static float pressureBaseline = 0.0;  // 0 = disabled

// Forward declarations
static float loadPressureBaseline();
static void savePressureBaseline(float baseline);
static void resetI2C();

// ===================================================================
// I2C Recovery
// ===================================================================

static void resetI2C() {
    Serial.println("[SENSOR] Resetting I2C bus...");
    digitalWrite(VEXT_CTRL, HIGH); // Power off
    delay(200);
    digitalWrite(VEXT_CTRL, LOW);  // Power on
    delay(100);
    I2C_BME.begin(BME280_SDA, BME280_SCL, 100000);
    delay(50);
}

// ===================================================================
// Initialization
// ===================================================================

bool initSensor() {
    Serial.println("[SENSOR] Initializing BME280...");

    // Initialize custom I2C bus
    I2C_BME.begin(BME280_SDA, BME280_SCL, 100000);
    delay(100);

    // Try to initialize BME280
    if (!bme280.begin(BME280_ADDR, &I2C_BME)) {
        Serial.printf("[SENSOR] ERROR: BME280 not found at address 0x%02X!\n", BME280_ADDR);
        Serial.printf("[SENSOR] Check wiring: SDA=GPIO%d, SCL=GPIO%d\n", BME280_SDA, BME280_SCL);
        return false;
    }

    // Configure sensor for weather monitoring
    bme280.setSampling(
        Adafruit_BME280::MODE_NORMAL,        // Operating Mode
        Adafruit_BME280::SAMPLING_X2,        // Temp oversampling
        Adafruit_BME280::SAMPLING_X16,       // Pressure oversampling
        Adafruit_BME280::SAMPLING_X1,        // Humidity oversampling
        Adafruit_BME280::FILTER_X4,          // Filtering
        Adafruit_BME280::STANDBY_MS_1000     // Standby time
    );

    Serial.printf("[SENSOR] BME280 initialized at address 0x%02X\n", BME280_ADDR);
    Serial.printf("[SENSOR] I2C: SDA=GPIO%d, SCL=GPIO%d\n", BME280_SDA, BME280_SCL);

    // Load pressure baseline from SPIFFS
    pressureBaseline = loadPressureBaseline();

    return true;
}

// ===================================================================
// Sensor Reading
// ===================================================================

bool readSensorData(ReadingsPayload* readings) {
    if (!readings) {
        Serial.println("[SENSOR] ERROR: NULL readings pointer");
        return false;
    }

    // Read sensor using Adafruit Unified Sensor interface
    sensors_event_t temp_event, pressure_event, humidity_event;

    Adafruit_Sensor *bme_temp = bme280.getTemperatureSensor();
    Adafruit_Sensor *bme_pressure = bme280.getPressureSensor();
    Adafruit_Sensor *bme_humidity = bme280.getHumiditySensor();

    // Check for I2C errors and recover if needed
    if (!bme_temp || !bme_pressure || !bme_humidity) {
        Serial.println("[SENSOR] ERROR: I2C read failed, resetting...");
        sensorFailures++;
        resetI2C();
        if (!bme280.begin(BME280_ADDR, &I2C_BME)) {
            Serial.println("[SENSOR] ERROR: Recovery failed");
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "BME280 I2C failure - recovery failed");
            return false;
        }
        delay(50);
        // Reapply the same sampling configuration used during initial setup
        bme280.setSampling(
            Adafruit_BME280::MODE_NORMAL,        // Operating Mode
            Adafruit_BME280::SAMPLING_X2,        // Temp oversampling
            Adafruit_BME280::SAMPLING_X16,       // Pressure oversampling
            Adafruit_BME280::SAMPLING_X1,        // Humidity oversampling
            Adafruit_BME280::FILTER_X4,          // Filtering
            Adafruit_BME280::STANDBY_MS_1000     // Standby time
        );
        bme_temp = bme280.getTemperatureSensor();
        bme_pressure = bme280.getPressureSensor();
        bme_humidity = bme280.getHumiditySensor();
        if (!bme_temp || !bme_pressure || !bme_humidity) {
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "BME280 sensors unavailable after reset");
            return false;
        }
        // Recovery successful
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_WARNING, "BME280 I2C recovered");
    }

    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    // Apply calibration offsets and convert to payload format
    float temperature = temp_event.temperature + TEMP_OFFSET_C;
    float humidity = humidity_event.relative_humidity + HUMIDITY_OFFSET_RH;
    float pressure = pressure_event.pressure * 100.0 + PRESSURE_OFFSET_PA;  // Convert hPa to Pa

    // Validate readings - detect I2C communication failures
    bool validReading = true;
    if (temperature < -40.0 || temperature > 85.0) {
        Serial.printf("[SENSOR] ERROR: Invalid temperature: %.2f°C\n", temperature);
        validReading = false;
    }
    if (humidity < 0.0 || humidity > 100.0) {
        Serial.printf("[SENSOR] ERROR: Invalid humidity: %.2f%%\n", humidity);
        validReading = false;
    }
    if (pressure < 30000.0 || pressure > 110000.0) {  // 300-1100 hPa in Pa
        Serial.printf("[SENSOR] ERROR: Invalid pressure: %.2f hPa\n", pressure / 100.0);
        validReading = false;
    }

    if (!validReading) {
        Serial.println("[SENSOR] I2C communication failure detected, attempting recovery...");
        sensorFailures++;
        resetI2C();
        if (!bme280.begin(BME280_ADDR, &I2C_BME)) {
            Serial.println("[SENSOR] ERROR: Recovery failed");
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "BME280 invalid data - recovery failed");
            return false;
        }
        delay(100);
        // Reapply sensor sampling configuration after successful reinitialization
        bme280.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,   // temperature oversampling
            Adafruit_BME280::SAMPLING_X16,  // pressure oversampling
            Adafruit_BME280::SAMPLING_X1,   // humidity oversampling
            Adafruit_BME280::FILTER_X4,
            Adafruit_BME280::STANDBY_MS_1000
        );
        // Update sensor pointers after reinitialization
        bme_temp = bme280.getTemperatureSensor();
        bme_pressure = bme280.getPressureSensor();
        bme_humidity = bme280.getHumiditySensor();
        // Try reading again after recovery
        bme_temp->getEvent(&temp_event);
        bme_pressure->getEvent(&pressure_event);
        bme_humidity->getEvent(&humidity_event);
        
        temperature = temp_event.temperature + TEMP_OFFSET_C;
        humidity = humidity_event.relative_humidity + HUMIDITY_OFFSET_RH;
        pressure = pressure_event.pressure * 100.0 + PRESSURE_OFFSET_PA;
        
        // Check again
        if (temperature < -40.0 || temperature > 85.0 ||
            humidity < 0.0 || humidity > 100.0 ||
            pressure < 30000.0 || pressure > 110000.0) {
            Serial.println("[SENSOR] ERROR: Still invalid after recovery");
            sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_ERROR, "BME280 persistent I2C failure");
            return false;
        }
        sendEventMessage(EVENT_SENSOR_ERROR, SEVERITY_WARNING, "BME280 I2C recovered");
    }

    // Calculate altitude from pressure
    float altitude = 44330.0 * (1.0 - pow(pressure / PRESSURE_SEA_LEVEL, 1.0/5.255));

    // Calculate pressure trend if baseline is set
    int32_t pressureChange = 0;
    uint8_t pressureTrend = 1;  // 0=falling, 1=steady, 2=rising

    if (pressureBaseline > 0) {
        pressureChange = (int32_t)(pressure - pressureBaseline);

        // Determine trend (threshold: ±50 Pa = ±0.5 hPa)
        if (pressureChange < -50) {
            pressureTrend = 0;  // Falling
        } else if (pressureChange > 50) {
            pressureTrend = 2;  // Rising
        } else {
            pressureTrend = 1;  // Steady
        }
    }

    // Populate payload (convert to fixed-point integers)
    readings->timestamp = millis() / 1000;  // Uptime in seconds
    readings->temperature = (int16_t)(temperature * 100.0);  // °C * 100
    readings->humidity = (uint16_t)(humidity * 100.0);       // % * 100
    readings->pressure = (uint32_t)pressure;                 // Pa
    readings->altitude = (int16_t)altitude;                  // meters
    readings->pressureChange = pressureChange;               // Pa
    readings->pressureTrend = pressureTrend;

    // Battery will be filled by power_manager
    readings->batteryVoltage = 0;
    readings->batteryPercent = 0;

    // Debug output
    Serial.println("[SENSOR] Reading:");
    Serial.printf("  Temperature: %.2f°C\n", temperature);
    Serial.printf("  Humidity: %.2f%%\n", humidity);
    Serial.printf("  Pressure: %.2f hPa\n", pressure / 100.0);
    Serial.printf("  Altitude: %.1f m\n", altitude);

    if (pressureBaseline > 0) {
        const char* trendStr = (pressureTrend == 0) ? "falling" :
                              (pressureTrend == 2) ? "rising" : "steady";
        Serial.printf("  Pressure trend: %s (%+.1f hPa from baseline)\n",
                     trendStr, pressureChange / 100.0);
    }

    return true;
}

// ===================================================================
// Pressure Baseline Management
// ===================================================================

float getPressureBaseline() {
    return pressureBaseline;
}

void setPressureBaseline(float baseline) {
    pressureBaseline = baseline;
    savePressureBaseline(baseline);

    if (baseline > 0) {
        Serial.printf("[BASELINE] Set to %.2f Pa (%.2f hPa)\n", baseline, baseline / 100.0);
    } else {
        Serial.println("[BASELINE] Tracking disabled");
    }
}

void calibrateBaseline() {
    // Use current pressure as baseline
    float currentPressure = bme280.readPressure();  // Returns Pa
    setPressureBaseline(currentPressure);
    Serial.println("[BASELINE] Calibrated to current pressure");
}

void clearBaseline() {
    setPressureBaseline(0.0);
    Serial.println("[BASELINE] Cleared");
}

// ===================================================================
// SPIFFS Persistence
// ===================================================================

static float loadPressureBaseline() {
    if (!LittleFS.exists(PRESSURE_BASELINE_FILE)) {
        Serial.println("[BASELINE] No saved baseline (using 0.0)");
        return 0.0;
    }

    File file = LittleFS.open(PRESSURE_BASELINE_FILE, "r");
    if (!file) {
        Serial.println("[BASELINE] ERROR: Failed to open baseline file");
        return 0.0;
    }

    float baseline = file.parseFloat();
    file.close();

    if (baseline > 0) {
        Serial.printf("[BASELINE] Loaded: %.2f Pa (%.2f hPa)\n", baseline, baseline / 100.0);
    } else {
        Serial.println("[BASELINE] Tracking disabled (0.0)");
    }

    return baseline;
}

static void savePressureBaseline(float baseline) {
    File file = LittleFS.open(PRESSURE_BASELINE_FILE, "w");
    if (!file) {
        Serial.println("[BASELINE] ERROR: Failed to save baseline");
        return;
    }

    file.println(baseline, 2);  // Save with 2 decimal places
    file.close();

    Serial.printf("[BASELINE] Saved: %.2f Pa (%.2f hPa)\n", baseline, baseline / 100.0);
}

// ===================================================================
// Sensor Health Tracking
// ===================================================================

uint16_t getSensorFailures() {
    return sensorFailures;
}

#endif // SENSOR_TYPE_BME280
