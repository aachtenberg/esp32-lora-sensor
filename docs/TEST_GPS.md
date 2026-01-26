# GPS Testing Guide

Quick guide for diagnosing and testing the NEO-6M GPS module.

## Hardware Requirements

- NEO-6M GPS module (or compatible)
- Clear view of sky (window or outdoors)
- ESP32 LoRa V3 board

## Wiring

```
NEO-6M GPS → ESP32 LoRa V3
───────────────────────────
VCC (3.3V) → 3V
GND        → GND
TX         → GPIO3 (ESP32 RX)
RX         → GPIO1 (ESP32 TX)
```

## Quick Test

### Step 1: Upload GPS Pin Diagnostic

Create a temporary test environment in `platformio.ini`:

```ini
[env:test-gps]
platform = espressif32 @ ^6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600
build_src_filter = -<*> +<../test_gps_pins.cpp>
```

Upload and monitor:

```bash
pio run -e test-gps -t upload
pio device monitor
```

### Step 2: Interpret Results

**✅ Success:**
```
=== Testing 9600 baud ===
✅ Received 245 bytes at 9600 baud
First 80 chars:
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

**❌ No data received:**
```
❌ No data at 9600 baud
❌ No data at 115200 baud
❌ No data at 4800 baud
```

**Possible causes:**
- GPS module not powered
- Wiring incorrect (check TX/RX connections)
- GPS module defective
- TX/RX pins swapped

**✅ Swapped pins worked:**
```
=== Testing SWAPPED pins at 9600 baud ===
✅ SWAPPED PINS WORKED! Received 234 bytes at 9600 baud
Your GPS module has TX/RX pins mislabeled!
```

**Solution:** Rewire GPS with TX/RX reversed

## Common Issues

### Issue 1: No Data Received

**Symptoms:**
- All tests show 0 bytes received
- No output at any baud rate

**Solutions:**
1. Check power: GPS module should have 3.3V on VCC
2. Verify GND connection
3. Look for LED indicator:
   - Solid: Module powered, no satellite fix
   - Blinking: Receiving satellite data
   - Off: Not powered or defective

### Issue 2: Garbled Data

**Symptoms:**
- Bytes received but not readable NMEA sentences
- Strange characters in output

**Solutions:**
1. Wrong baud rate - NEO-6M default is 9600
2. Weak power supply causing data corruption
3. Electrical noise on data lines

### Issue 3: TX/RX Pins Swapped

**Symptoms:**
- Standard configuration: No data
- Swapped configuration: Data received

**Solution:**
- Some GPS modules have mislabeled TX/RX pins
- Physically swap the connections:
  - GPS TX → GPIO1 (instead of GPIO3)
  - GPS RX → GPIO3 (instead of GPIO1)
- Update `device_config.h` if needed

### Issue 4: No Satellite Fix

**Symptoms:**
- GPS transmitting NMEA sentences
- All location fields show 0 or invalid
- Satellite count is 0

**Solutions:**
1. **Location:** GPS needs clear view of sky
   - Move near window
   - Go outdoors
   - Avoid indoor locations
2. **Time:** Initial fix can take 1-30 minutes
3. **Cold start:** GPS has no almanac data yet
4. **Weak signals:** Try repositioning antenna

## Full Firmware GPS Testing

After wiring is verified, test with full firmware:

### Build with GPS Support

```bash
# BME280 + GPS
pio run -e esp32-lora-sensor -D GPS_ENABLED -t upload

# Or use the dedicated GPS environment
pio run -e esp32-lora-sensor-bme280-gps-usb1 -t upload
```

### Monitor GPS Status

Open serial monitor and look for:

```
Initializing GPS module...
GPS initialization complete
---
Updating GPS location...
GPS: 52.123456, 4.567890 (alt: 12.3 m, sats: 8, HDOP: 1.2)
```

**No fix:**
```
Updating GPS location...
GPS: No fix yet
```

### Check OLED Display

The OLED should show GPS status:
- Satellite count
- Fix status (yes/no)
- Coordinates when available
- Altitude and HDOP

## GPS Data Format

GPS fields in `ReadingsPayload`:

```cpp
int32_t  gpsLatitude;     // Latitude * 1000000 (52.123456° = 52123456)
int32_t  gpsLongitude;    // Longitude * 1000000 (4.567890° = 4567890)
int16_t  gpsAltitude;     // Altitude in meters
uint8_t  gpsSatellites;   // Number of satellites (0-12+)
uint16_t gpsHdop;         // HDOP * 10 (1.5 = 15)
```

**When GPS disabled or no fix:**
- All fields set to 0
- Gateway still receives valid packet
- Protocol remains consistent

## Verify on Gateway

Check gateway logs for GPS data:

```
[MQTT] Publishing to telegraf/lora/sensor...
Device: 0x000044DD789E9EF0
  Temperature: 23.45°C
  GPS: 52.123456, 4.567890 (12m, 8 sats, HDOP=1.2)
```

## Troubleshooting Checklist

- [ ] GPS module powered (LED visible)
- [ ] Wiring matches documented pinout
- [ ] Test shows data reception at 9600 baud
- [ ] GPS has clear view of sky
- [ ] Waited at least 5 minutes for initial fix
- [ ] Satellite count > 0 in serial output
- [ ] GPS coordinates appear on OLED display
- [ ] Gateway receives GPS data in packets

## Advanced: GPS Configuration

NEO-6M modules can be configured with u-center software (Windows) to:
- Change baud rate
- Adjust update rate
- Enable/disable NMEA sentences
- Save configuration to flash

**Note:** Default settings work fine for most applications.
