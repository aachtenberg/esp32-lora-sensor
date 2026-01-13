#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

// ====================================================================
// ESP32 LoRa Sensor - Hardware Configuration
// ====================================================================

// SX1262 LoRa Module Pins (SPI Interface)
// Heltec WiFi LoRa 32 V3 (ESP32-S3) with built-in SX1262
#define LORA_MISO      11   // FSPIQ (built-in LoRa MISO)
#define LORA_MOSI      10   // FSPID (built-in LoRa MOSI)
#define LORA_SCK       9    // FSPICLK (built-in LoRa SCK)
#define LORA_NSS       8    // Chip Select / NSS (built-in LoRa CS)
#define LORA_DIO1      14   // IRQ Pin (DIO1)
#define LORA_BUSY      13   // Busy Pin
#define LORA_RST       12   // Reset Pin

// Heltec-specific: Vext power control for LoRa and OLED
// Vext must be LOW to enable power to peripherals
#define VEXT_CTRL      36   // Vext control pin (LOW = ON, HIGH = OFF)

// BME280 Sensor (I2C - External breakout board)
// Using software I2C on free GPIOs (avoid conflict with built-in OLED)
#define BME280_SDA     33   // I2C SDA (GPIO33 - Header J2, pin 12)
#define BME280_SCL     26   // I2C SCL (GPIO26 - Header J2, pin 15)
#define BME280_ADDR    0x76 // I2C Address (0x76 if SDO low, 0x77 if SDO high)

// BME280 Sampling Configuration
// See Adafruit_BME280.h for mode/sampling values
#define BME280_SAMPLING_MODE    0x03  // Normal mode
#define BME280_TEMP_SAMPLING    0x02  // x2 oversampling
#define BME280_PRESSURE_SAMPLING 0x05 // x16 oversampling
#define BME280_HUMIDITY_SAMPLING 0x01 // x1 oversampling
#define BME280_FILTER           0x03  // Filter coefficient 4
#define BME280_STANDBY_MS       1000  // Standby time in ms

// OLED Display (I2C - Built-in on board)
#define OLED_SDA       17   // Built-in OLED SDA
#define OLED_SCL       18   // Built-in OLED SCL
#define OLED_RST       21   // Built-in OLED Reset
#define OLED_ADDR      0x3C // Standard I2C address for SSD1306

// Battery Monitoring
// Note: GPIO35 may be used by LoRa BUSY, using GPIO36 instead
#define BATTERY_PIN    36   // ADC1_CH0 (VP - input only, no pull-up)
#define BATTERY_ADC_SAMPLES 10  // Number of samples to average

// Battery voltage divider calibration
// If using voltage divider R1=10K, R2=10K (2:1 ratio)
// Voltage divider: Battery+ -> 10K -> ADC_PIN -> 10K -> GND
#define BATTERY_VOLTAGE_MULTIPLIER 2.0
#define BATTERY_CALIBRATION 1.134     // Calibration factor (from current project)
#define BATTERY_ADC_REFERENCE 3.3     // ESP32 ADC reference voltage
#define BATTERY_ADC_RESOLUTION 4095.0 // 12-bit ADC

// Battery percentage thresholds (LiPo 3.7V nominal)
#define BATTERY_MIN_VOLTAGE 3.0   // Empty (shutdown to prevent damage)
#define BATTERY_MAX_VOLTAGE 4.2   // Full charge
#define BATTERY_CRITICAL_PERCENT 10  // Skip ACK wait if below this

// Sensor Timing
#define SENSOR_READ_INTERVAL_MS 2000  // Time for BME280 to stabilize
#define DEFAULT_DEEP_SLEEP_SECONDS 90  // 90 seconds default (configurable via MQTT)

// BME280 Calibration Offsets (from current project)
// Adjust these based on your sensor's calibration
#define TEMP_OFFSET_C 0.0
#define HUMIDITY_OFFSET_RH 0.0
#define PRESSURE_OFFSET_PA 0.0

// Pressure baseline configuration (for barometer-style weather tracking)
// Set to local station pressure for weather monitoring:
//   - Sea level: 101325 Pa (1013.25 hPa)
//   - High altitude: ~98000 Pa (980 hPa) for 200m elevation
//   - Set to 0.0 to disable baseline tracking
// Adjust via LoRa commands from gateway
#define PRESSURE_SEA_LEVEL 101325.0  // Pa - for altitude calculation
#define PRESSURE_BASELINE_FILE "/pressure_baseline.txt"

// Configuration Button (optional)
#define CONFIG_BTN     0    // GPIO 0 (BOOT button on most ESP32 boards)

#endif // DEVICE_CONFIG_H
