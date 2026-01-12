/**
 * Sensor Manager - BME280 Environmental Sensor
 * Handles BME280 initialization, reading, and pressure baseline tracking
 * Migrated from esp12f_ds18b20_temp_sensor/bme280-sensor
 */

#include "sensor_manager.h"
#include "device_config.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <FS.h>
#include <LittleFS.h>

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

    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    // Apply calibration offsets and convert to payload format
    float temperature = temp_event.temperature + TEMP_OFFSET_C;
    float humidity = humidity_event.relative_humidity + HUMIDITY_OFFSET_RH;
    float pressure = pressure_event.pressure * 100.0 + PRESSURE_OFFSET_PA;  // Convert hPa to Pa

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
