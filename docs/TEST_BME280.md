# BME280 Test Instructions

Quick test to verify your BME280 wiring before running the full sensor code.

## What This Tests

1. I2C communication on GPIO33 (SDA) and GPIO26 (SCL)
2. BME280 detection at address 0x76 or 0x77
3. Sensor readings (temperature, humidity, pressure, altitude)
4. Data validity checks

## Upload and Run Test

```bash
cd ~/repos/esp32-lora-sensor

# Build and upload test sketch
pio run -e test-bme280 -t upload

# Monitor serial output
pio device monitor -e test-bme280
```

## Expected Output

```
================================
BME280 I2C Test
================================
SDA: GPIO33 (Header J2, pin 12)
SCL: GPIO26 (Header J2, pin 15)
Expected Address: 0x76
================================

Scanning I2C bus...
  Found device at address 0x76
  Found 1 device(s)

Initializing BME280...
  SUCCESS! BME280 found at address 0x76

BME280 configured successfully!
Reading sensor every 2 seconds...

--- BME280 Readings ---
Temperature: 22.45 °C
Humidity:    45.23 %
Pressure:    1013.25 hPa
Altitude:    120.50 m
```

## Troubleshooting

### "No I2C devices found"
**Problem**: I2C scan doesn't find any devices

**Solutions**:
1. Check wiring:
   - VCC → 3V (Header J2, pin 3)
   - GND → GND (Header J2, pin 1)
   - SDA → GPIO33 (Header J2, pin 12)
   - SCL → GPIO26 (Header J2, pin 15)

2. Verify power:
   - BME280 should have 3.3V on VCC pin
   - Check with multimeter if available

3. Check connections:
   - Ensure wires are firmly connected
   - Look for loose header pins
   - Try reseating the connections

### "Could not find BME280 sensor"
**Problem**: I2C device found but BME280 initialization fails

**Solutions**:
1. Try alternate address:
   - Some BME280 modules use 0x77 instead of 0x76
   - Test sketch will automatically try 0x77
   - If found at 0x77, update `device_config.h`:
     ```cpp
     #define BME280_ADDR    0x77  // Change from 0x76
     ```

2. Check sensor type:
   - Verify you have a BME280 (not BMP280 or BME680)
   - BME280 measures temp, humidity, AND pressure
   - BMP280 only measures temp and pressure

### "Temperature/Humidity/Pressure out of range"
**Problem**: Sensor reads but values are unrealistic

**Solutions**:
1. Check sensor quality:
   - Cheap BME280 clones may give bad readings
   - Try known-good sensor if available

2. Verify I2C pullups:
   - Most BME280 breakouts have built-in 10K pullups
   - Check if your module has pullup resistors

3. Check I2C speed:
   - Test uses 100kHz (standard speed)
   - Should work with most BME280 modules

### Still not working?

1. **Test with built-in OLED I2C**:
   Temporarily change pins in `device_config.h`:
   ```cpp
   #define BME280_SDA     4    // Use OLED's SDA
   #define BME280_SCL     15   // Use OLED's SCL
   ```
   If this works, problem is with GPIO33/26 I2C.

2. **Verify ESP32 is working**:
   Upload a simple blink sketch to confirm board is functional.

3. **Try different BME280 module**:
   Could be defective sensor.

## Next Steps

Once test succeeds:
1. Note which I2C address works (0x76 or 0x77)
2. Update `device_config.h` if needed
3. Return to main firmware development

## Quick Commands Reference

```bash
# Build test only (no upload)
pio run -e test-bme280

# Upload and monitor in one command
pio run -e test-bme280 -t upload && pio device monitor -e test-bme280

# Clean and rebuild
pio run -e test-bme280 -t clean
pio run -e test-bme280
```
