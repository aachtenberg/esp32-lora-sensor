# Testing Guide

This document covers all test programs available for debugging and verifying hardware before running the full sensor firmware.

## Test Files Overview

| Test File | Purpose | Environment |
|-----------|---------|-------------|
| `test_bme280.cpp` | Verify BME280 I2C wiring and readings | N/A (commented out) |
| `test_config_manager.cpp` | Test LittleFS config persistence | N/A (commented out) |
| `test_gps_pins.cpp` | Diagnose GPS module wiring | N/A (add custom env) |
| `test_sensor_manager.cpp` | Test sensor abstraction layer | N/A (commented out) |
| `test_lora_comm.cpp` | Test LoRa transmission/reception | N/A (commented out) |

> **Note:** Test environments are currently commented out in `platformio.ini`. Uncomment the relevant section before running tests.

## Quick Test Guide

### 1. GPS Pin Diagnostic

**Purpose:** Verify GPS module wiring and communication

**Create temporary environment in platformio.ini:**
```ini
[env:test-gps]
platform = espressif32 @ ^6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600

; Use GPS test file
build_src_filter = -<*> +<../test_gps_pins.cpp>
```

**Run test:**
```bash
pio run -e test-gps -t upload
pio device monitor
```

**Expected output:**
- Tests standard and swapped pin configurations
- Tests multiple baud rates (9600, 115200, 4800)
- Shows raw bytes received from GPS
- Indicates if TX/RX pins are mislabeled

**See full documentation:** [docs/TEST_GPS.md](TEST_GPS.md)

### 2. BME280 Sensor Test

**Purpose:** Verify BME280 I2C wiring before running full firmware

**Uncomment in platformio.ini:**
```ini
[env:test-bme280]
platform = espressif32 @ ^6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200

build_flags = -D CPU_FREQ_MHZ=80

lib_deps =
    adafruit/Adafruit BME280 Library @ 2.2.4
    adafruit/Adafruit Unified Sensor @ 1.1.14

upload_speed = 921600
build_src_filter = +<*> -<main.cpp> +<../test_bme280.cpp>
```

**Run test:**
```bash
pio run -e test-bme280 -t upload
pio device monitor
```

**Expected output:**
- I2C bus scan showing device at 0x76 or 0x77
- Continuous sensor readings every 2 seconds
- Temperature, humidity, pressure, and altitude values

**See full documentation:** [docs/TEST_BME280.md](TEST_BME280.md)

### 3. Config Manager Test

**Purpose:** Test configuration file persistence in LittleFS

**Uncomment in platformio.ini:**
```ini
[env:test-config-manager]
platform = espressif32 @ ^6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200

build_flags = -D CPU_FREQ_MHZ=80
lib_deps =
    adafruit/Adafruit BME280 Library @ 2.2.4
    adafruit/Adafruit Unified Sensor @ 1.1.14

upload_speed = 921600
board_build.filesystem = littlefs
build_src_filter = +<*> -<main.cpp> -<sensor_manager.cpp> +<../test_config_manager.cpp>
```

**Run test:**
```bash
pio run -e test-config-manager -t uploadfs  # Upload filesystem first
pio run -e test-config-manager -t upload
pio device monitor
```

**Expected output:**
- Configuration loading from LittleFS
- Interactive config menu
- Save/load verification

**See full documentation:** [docs/TEST_CONFIG_MANAGER.md](TEST_CONFIG_MANAGER.md)

## Troubleshooting Common Issues

### GPS Not Working
1. Run `test_gps_pins.cpp` to diagnose wiring
2. Check if TX/RX pins are swapped (common with clones)
3. Verify 3.3V power and GND connections
4. Ensure GPS has clear view of sky

### BME280 Not Found
1. Run I2C scan in `test_bme280.cpp`
2. Try alternate address (0x77 instead of 0x76)
3. Verify SDA/SCL connections on GPIO33/GPIO26
4. Check for I2C pull-up resistors (most breakouts have them)

### Configuration Not Persisting
1. Upload filesystem with `pio run -t uploadfs`
2. Check LittleFS is mounted successfully in logs
3. Verify files exist in `data/` directory
4. Use `test_config_manager.cpp` to debug file operations

## Creating Custom Tests

To add a new test:

1. **Create test file** in repository root: `test_myfeature.cpp`

2. **Add environment** in `platformio.ini`:
```ini
[env:test-myfeature]
platform = espressif32 @ ^6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200

build_flags = -D CPU_FREQ_MHZ=80
lib_deps = 
    ; Add required libraries

upload_speed = 921600
build_src_filter = +<*> -<main.cpp> +<../test_myfeature.cpp>
```

3. **Build and upload**:
```bash
pio run -e test-myfeature -t upload
pio device monitor
```

## Production Testing Checklist

Before deploying a sensor device:

- [ ] GPS wiring tested (if using GPS)
- [ ] Sensor readings are accurate
- [ ] LoRa transmission successful
- [ ] Configuration persists across reboots
- [ ] Battery monitoring shows correct voltage
- [ ] OLED display shows information clearly
- [ ] Device name is set correctly
- [ ] Sleep intervals configured appropriately
- [ ] Gateway receives and processes packets
