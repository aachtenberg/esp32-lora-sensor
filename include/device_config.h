#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

// ====================================================================
// ESP32 LoRa Sensor - Hardware Configuration
// ====================================================================

// SX1262 LoRa Module Pins (SPI Interface)
// IMPORTANT: Verify these against your specific ESP32 LoRa module datasheet
#define LORA_MISO      19   // SPI MISO
#define LORA_MOSI      27   // SPI MOSI
#define LORA_SCK       5    // SPI Clock
#define LORA_NSS       18   // Chip Select (SS)
#define LORA_DIO1      26   // IRQ Pin (DIO1)
#define LORA_BUSY      23   // Busy Pin
#define LORA_RST       14   // Reset Pin

// BME280 Sensor (I2C - shared with OLED)
#define BME280_SDA     21   // I2C SDA
#define BME280_SCL     22   // I2C SCL
#define BME280_ADDR    0x76 // I2C Address (or 0x77 depending on SDO pin)

// OLED Display (I2C - shared bus with BME280)
#define OLED_SDA       21   // Same as BME280
#define OLED_SCL       22   // Same as BME280
#define OLED_ADDR      0x3C // Standard I2C address for SSD1306

// Battery Monitoring
#define BATTERY_PIN    35   // ADC1_CH7 (input only, no pull-up)
#define BATTERY_ADC_SAMPLES 10  // Number of samples to average

// Battery voltage divider calibration
// If using voltage divider R1=100K, R2=100K (2:1 ratio)
#define BATTERY_VOLTAGE_MULTIPLIER 2.0
#define BATTERY_ADC_REFERENCE 3.3  // ESP32 ADC reference voltage
#define BATTERY_ADC_RESOLUTION 4095.0  // 12-bit ADC

// Battery percentage thresholds (LiPo 3.7V nominal)
#define BATTERY_MIN_VOLTAGE 3.0   // Empty (shutdown to prevent damage)
#define BATTERY_MAX_VOLTAGE 4.2   // Full charge
#define BATTERY_CRITICAL_PERCENT 10  // Skip ACK wait if below this

// Sensor Timing
#define SENSOR_READ_INTERVAL_MS 2000  // Time for BME280 to stabilize
#define DEFAULT_DEEP_SLEEP_SECONDS 900  // 15 minutes default

// BME280 Calibration Offsets (from current project)
// Adjust these based on your sensor's calibration
#define TEMP_OFFSET_C 0.0
#define HUMIDITY_OFFSET_RH 0.0
#define PRESSURE_OFFSET_PA 0.0

// Configuration Button (optional)
#define CONFIG_BTN     0    // GPIO 0 (BOOT button on most ESP32 boards)

#endif // DEVICE_CONFIG_H
