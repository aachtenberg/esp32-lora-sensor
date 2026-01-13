# Config Manager Test

Tests the configuration loading and saving functionality using LittleFS.

## How to Run

1. **Update platformio.ini** to use the test file:
   ```ini
   [env:esp32-s3-devkitc-1]
   platform = espressif32
   board = esp32-s3-devkitc-1
   framework = arduino
   monitor_speed = 115200
   
   ; Build config_manager test
   build_src_filter = 
       +<config_manager.cpp>
       -<main.cpp>
       -<sensor_manager.cpp>
   
   build_flags = 
       -D TEST_CONFIG_MANAGER
       -I lib/LoRaProtocol
   
   lib_deps = 
       adafruit/Adafruit BME280 Library@^2.2.2
       adafruit/Adafruit Unified Sensor@^1.1.9
   ```

2. **Upload the filesystem** (if not already done):
   ```bash
   pio run --target uploadfs
   ```

3. **Build and upload**:
   ```bash
   pio run --target upload
   ```

4. **Monitor the output**:
   ```bash
   pio device monitor
   ```

## What Gets Tested

✅ **Test 1:** Initialize config manager and mount LittleFS  
✅ **Test 2:** Read configuration from files (device_name.txt, deep_sleep_seconds.txt, pressure_baseline.txt)  
✅ **Test 3:** Update settings in memory and save to files  
✅ **Test 4:** Verify updated values persist  
✅ **Test 5:** Reload configuration from files  
✅ **Test 6:** Restore original values

## Interactive Configuration Menu

After tests complete, you can type `config` or `menu` in the serial monitor to access the interactive configuration menu:

```
========== Configuration Menu ==========
1. Set Device Name
2. Set Deep Sleep Interval (seconds)
3. Set Pressure Baseline (hPa)
4. Show Current Configuration
5. Save All Settings
6. Reload Configuration
Type 'exit' to close menu
========================================
```

## Expected Output

```
╔════════════════════════════════════════╗
║   CONFIG MANAGER TEST                 ║
╚════════════════════════════════════════╝

[TEST 1] Initialize Config Manager

=== Config Manager Initialization ===
✅ LittleFS mounted
✅ Loaded device name: BME280-LoRa-001
✅ Loaded deep sleep: 90 seconds
✅ Loaded pressure baseline: 0.00 hPa
Device Name: BME280-LoRa-001
Deep Sleep: 90 seconds
Pressure Baseline: 0.00 hPa
=====================================

✅ Config manager initialized

[TEST 2] Read Current Configuration
Device Name: BME280-LoRa-001
Deep Sleep: 90 seconds (1.5 minutes)
Pressure Baseline: 0.00 hPa

[TEST 3] Update Settings
✅ Device name updated: TestDevice-123
✅ Deep sleep updated: 600 seconds
✅ Pressure baseline updated: 1013.25 hPa

[TEST 4] Verify Updated Values
Device Name: TestDevice-123
Deep Sleep: 600 seconds
Pressure Baseline: 1013.25 hPa

[TEST 5] Reload Configuration from Files
✅ Loaded device name: TestDevice-123
✅ Loaded deep sleep: 600 seconds
✅ Loaded pressure baseline: 1013.25 hPa
✅ Configuration reloaded successfully
Device Name: TestDevice-123
Deep Sleep: 600 seconds
Pressure Baseline: 1013.25 hPa

[TEST 6] Restore Original Values
✅ Device name updated: BME280-LoRa-001
✅ Deep sleep updated: 900 seconds
✅ Pressure baseline updated: 0.00 hPa

✅ All tests completed!

Type 'config' or 'menu' to open interactive configuration menu
```

## Configuration Files

Located in `/data/` directory (uploaded to LittleFS):

- **device_name.txt** - Device identifier (e.g., "BME280-LoRa-001")
- **deep_sleep_seconds.txt** - Sleep interval in seconds (e.g., "900" = 15 minutes)
- **pressure_baseline.txt** - Reference pressure in hPa (e.g., "0.0" = not calibrated)

## Features

- ✅ Loads configuration from LittleFS on startup
- ✅ Saves individual settings immediately when changed
- ✅ Provides getter/setter functions for all config values
- ✅ Interactive serial menu for runtime configuration
- ✅ Automatic file creation if missing
- ✅ Validates file operations with error reporting
