#include "power_manager.h"
#include "device_config.h"
#include <Arduino.h>

/**
 * Initialize power management
 */
void initPowerManager() {
    // Configure ADC resolution
    analogReadResolution(12); // 12-bit resolution (0-4095)
    
    // Heltec V3: Enable battery ADC control pin
    #ifdef BATTERY_ADC_CTRL
    pinMode(BATTERY_ADC_CTRL, OUTPUT);
    digitalWrite(BATTERY_ADC_CTRL, HIGH);  // Enable ADC reading
    #endif
    
    // Configure battery pin with attenuation for full range
    pinMode(BATTERY_PIN, INPUT);
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);  // 0-3.3V range
    
    Serial.printf("[POWER] Initialized battery monitoring on GPIO%d\n", BATTERY_PIN);
}

/**
 * Read battery voltage and percentage
 */
void readBatteryStatus(uint16_t* voltage_mv, uint8_t* percentage) {
    if (voltage_mv == nullptr || percentage == nullptr) return;

    // Take multiple samples to reduce noise
    uint32_t adcSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        adcSum += analogRead(BATTERY_PIN);
        delay(2);
    }
    float adcAver = adcSum / (float)BATTERY_ADC_SAMPLES;

    // Convert raw ADC to voltage (at the pin)
    // ESP32-S3 ADC is ~3.3V max (actually attenuated)
    // Using default attenuation (11dB) allows reading up to ~3.1V
    // But we need to check if attenuation was set. 
    // Arduino ESP32 default is 11dB (0-3.3V range approx) since v2.x
    
    // Formula: ADC * (Vref / Resolution) * Multiplier * Calibration
    float voltage = adcAver * (BATTERY_ADC_REFERENCE / BATTERY_ADC_RESOLUTION) * 
                    BATTERY_VOLTAGE_MULTIPLIER * BATTERY_CALIBRATION;

    // Convert to millivolts
    *voltage_mv = (uint16_t)(voltage * 1000);

    // Calculate percentage (LiPo 3.0V - 4.2V)
    if (voltage >= BATTERY_MAX_VOLTAGE) {
        *percentage = 100;
    } else if (voltage <= BATTERY_MIN_VOLTAGE) {
        *percentage = 0;
    } else {
        // Linear interpolation
        *percentage = (uint8_t)((voltage - BATTERY_MIN_VOLTAGE) / 
                               (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0);
    }

    // Determine charging status (simple voltage threshold)
    // > 4.3V usually implies charging via USB
    bool isCharging = (voltage > 4.3);
    
    // Debug output
    // Serial.printf("[POWER] ADC:%.0f V:%.3f P:%d%%\n", adcAver, voltage, *percentage);
}

/**
 * Read battery voltage (volts)
 */
float readBatteryVoltage() {
    // Ensure ADC control is enabled (Heltec V3)
    #ifdef BATTERY_ADC_CTRL
    digitalWrite(BATTERY_ADC_CTRL, HIGH);
    delay(10);  // Allow settling time
    #endif
    
    // Take multiple samples to reduce noise
    uint32_t adcSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        adcSum += analogRead(BATTERY_PIN);
        delay(2);
    }
    float adcAver = adcSum / (float)BATTERY_ADC_SAMPLES;

    // Formula: ADC * (Vref / Resolution) * Multiplier * Calibration
    float voltage = adcAver * (BATTERY_ADC_REFERENCE / BATTERY_ADC_RESOLUTION) * 
                    BATTERY_VOLTAGE_MULTIPLIER * BATTERY_CALIBRATION;

    Serial.printf("[POWER] ADC raw: %.0f, Voltage: %.2fV\n", adcAver, voltage);
    
    return voltage;
}

/**
 * Calculate battery percentage from voltage
 */
uint8_t calculateBatteryPercent(float voltage) {
    if (voltage >= BATTERY_MAX_VOLTAGE) {
        return 100;
    } else if (voltage <= BATTERY_MIN_VOLTAGE) {
        return 0;
    } else {
        return (uint8_t)((voltage - BATTERY_MIN_VOLTAGE) / 
                        (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0);
    }
}

/**
 * Configure deep sleep
 */
void enterDeepSleep(uint32_t seconds) {
    Serial.printf("[POWER] Entering deep sleep for %lu seconds...\n", seconds);
    Serial.flush();  // Ensure message is sent before sleep
    
    // Turn off peripherals
    #ifdef OLED_ENABLED
        // We can't easily turn off OLED object, but we can cut power via Vext
    #endif
    
    // Turn off Vext to cut power to sensors and LoRa
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, HIGH); // HIGH = OFF
    
    delay(100);  // Allow power to settle
    
    // Configure wake timer
    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
    
    // Go to sleep
    esp_deep_sleep_start();
}
