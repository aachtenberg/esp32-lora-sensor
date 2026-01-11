# ESP32 LoRa Sensor

Battery-powered environmental sensor using BME280 and LoRa communication.

## Hardware

- **ESP32**: ESP32 LX7 Dual-core
- **LoRa**: SX1262 LoRa module (863-928 MHz)
- **Sensor**: BME280 (temperature, humidity, pressure)
- **Display**: 0.96" OLED SSD1306 (I2C)
- **Power**: 3.7V LiPo battery

## Features

- BME280 environmental sensing (temperature, humidity, pressure, altitude)
- LoRa peer-to-peer communication with gateway
- Deep sleep mode for extended battery life (3-6 months)
- Battery voltage and percentage monitoring
- Pressure baseline tracking with trend detection
- OLED display for status and readings
- Serial configuration menu
- SPIFFS-based configuration persistence

## Pin Configuration

See [include/device_config.h](include/device_config.h) for complete pin assignments.

**Critical pins** (verify against your module datasheet):
```
LoRa SX1262 (SPI):
  MISO = 19, MOSI = 27, SCK = 5
  NSS = 18, DIO1 = 26, BUSY = 23, RST = 14

BME280 + OLED (I2C shared):
  SDA = 21, SCL = 22
  BME280 addr = 0x76, OLED addr = 0x3C

Battery: GPIO 35 (ADC)
```

## Quick Start

### 1. Install PlatformIO

```bash
pip install platformio
```

### 2. Clone and Build

```bash
cd ~/repos/esp32-lora-sensor
pio run
```

### 3. Upload Filesystem (First Time Only)

```bash
pio run -t uploadfs
```

This uploads the configuration files in `data/` to the ESP32.

### 4. Upload Firmware

```bash
pio run -t upload
```

### 5. Monitor Serial Output

```bash
pio device monitor
```

## Configuration

### Serial Configuration Menu

Press 'C' within 5 seconds of boot to enter config mode:

```
1. Set device name
2. Set deep sleep interval
3. Calibrate pressure baseline
4. View current config
5. Save and exit
```

### SPIFFS Files

Configuration is stored in `/data/`:
- `device_name.txt` - Device name (default: BME280-LoRa-001)
- `deep_sleep_seconds.txt` - Sleep interval (default: 900 = 15 min)
- `pressure_baseline.txt` - Pressure baseline in hPa (0.0 = disabled)

## Power Optimization

Expected battery life on 3000mAh LiPo:

| Interval | Battery Life |
|----------|--------------|
| 15 min   | ~5 months    |
| 30 min   | ~10 months   |
| 60 min   | ~20 months*  |

*Limited by battery self-discharge (~1 year max)

**Current consumption**:
- Deep sleep: <50 µA
- Wake + sensor: ~50 mA for 2 seconds
- LoRa TX: ~120 mA for 300 ms

## LoRa Configuration

See [include/lora_config.h](include/lora_config.h) for radio settings.

**Default settings** (SF9/125kHz):
- Frequency: 915 MHz (US) / 868 MHz (EU)
- Spreading Factor: 9 (balance range/power)
- TX Power: 14 dBm
- Expected range: 1-2 km urban, 5-10 km rural

## Protocol

Uses binary packet protocol defined in [lib/LoRaProtocol/lora_protocol.h](lib/LoRaProtocol/lora_protocol.h).

**Packet structure**:
- Header: 16 bytes (magic, version, type, device ID, sequence, checksum)
- Payload: Up to 240 bytes (sensor readings, status, events)

**Message types**:
- `MSG_READINGS` (0x01) - BME280 sensor data (37 bytes)
- `MSG_STATUS` (0x02) - Device health/diagnostics
- `MSG_COMMAND` (0x10) - Incoming commands from gateway
- `MSG_ACK` (0x20) - Acknowledgments

## Development

### Project Structure

```
esp32-lora-sensor/
├── platformio.ini       # Build configuration
├── src/
│   ├── main.cpp         # Main application
│   ├── sensor_manager.* # BME280 sensor operations
│   ├── lora_comm.*      # LoRa TX/RX
│   ├── power_manager.*  # Deep sleep, battery
│   ├── config_manager.* # SPIFFS configuration
│   └── display_manager.*# OLED display
├── include/
│   ├── device_config.h  # Hardware pins
│   ├── lora_config.h    # Radio settings
│   └── version.h        # Firmware version
├── lib/LoRaProtocol/    # Shared protocol library
└── data/                # SPIFFS filesystem
```

### Building

```bash
# Build
pio run

# Upload firmware
pio run -t upload

# Upload filesystem
pio run -t uploadfs

# Clean build
pio run -t clean
```

## Troubleshooting

### Sensor not reading
- Check I2C wiring (SDA=21, SCL=22)
- Verify BME280 address (0x76 or 0x77)
- Check I2C pullup resistors (usually built-in)

### LoRa not transmitting
- Verify SPI pins match your module
- Check antenna connection
- Verify frequency matches your region (915 MHz US, 868 MHz EU)

### Battery draining too fast
- Measure deep sleep current with ammeter (should be <50 µA)
- Check for incorrect pullups on GPIO pins
- Disable OLED during sleep
- Increase sleep interval

### Display not working
- Check I2C address (run I2C scanner: 0x3C expected)
- Verify shared I2C bus with BME280
- Some modules have hardware I2C pin assignments

## Related Projects

- [esp32-lora-gateway](../esp32-lora-gateway/) - Gateway for receiving sensor data

## License

MIT License - see LICENSE file
