# ESP32 LoRa Sensor

Battery-powered environmental sensor using LoRa communication. Supports **BME280** (temperature, humidity, pressure), **DHT22** (temperature, humidity), or **DS18B20** (temperature only) sensors.

## Hardware

- **Board**: Heltec WiFi LoRa 32 V3 (ESP32-S3) with built-in SX1262
- **LoRa**: SX1262 LoRa module (863-928 MHz)
- **Sensor**: BME280, DHT22, or DS18B20 (compile-time selection)
- **Display**: 0.96" OLED SSD1306 (I2C) - built-in
- **Power**: 3.7V LiPo battery

## Supported Sensors

| Sensor | Build Environment | Readings |
|--------|-------------------|----------|
| **BME280** | `esp32-lora-sensor` (default) | Temperature, humidity, pressure, altitude |
| **DHT22/AM2302** | `esp32-lora-sensor-dht22` | Temperature, humidity |
| **DS18B20** | `esp32-lora-sensor-ds18b20` | Temperature only |

## Features

- **Multi-sensor support**: BME280 environmental, DHT22 temp/humidity, or DS18B20 temperature sensor
- LoRa peer-to-peer communication with gateway
- **Device name propagation**: Auto-sends name to gateway in status messages
- **Location field**: Prepared for future GPS integration
- Low-power operation with configurable sleep intervals
- **Configurable sensor interval**: Reading frequency (5-3600s) via LoRa command
- **I2C error recovery**: Automatic detection and recovery from BME280 communication failures
- **Status messages**: Sent every 5 wake cycles (~7.5 min at 90s interval)
- **Event messages**: Startup, errors, config changes
- Battery voltage and percentage monitoring
- Pressure baseline tracking with trend detection (0=falling, 1=steady, 2=rising)
- **Command reception**: Opens RX window after each transmission
- OLED display for status and readings
- Serial configuration menu
- LittleFS-based configuration persistence
- **Unified sequence numbering**: Prevents duplicates across all message types

## Pin Configuration

See [include/device_config.h](include/device_config.h) for complete pin assignments.

**Heltec WiFi LoRa 32 V3 (ESP32-S3):**
```
LoRa SX1262 (SPI - built-in):
  MISO = 11, MOSI = 10, SCK = 9
  NSS = 8, DIO1 = 14, BUSY = 13, RST = 12

OLED Display (I2C - built-in):
  SDA = 17, SCL = 18, RST = 21
  OLED addr = 0x3C

BME280 Sensor (I2C - external, separate bus):
  SDA = 33, SCL = 26
  BME280 addr = 0x76

DS18B20 Sensor (1-Wire - external):
  Data = GPIO 4 (with 4.7K pull-up to 3.3V)

Vext Control: GPIO 36 (LOW = peripherals ON)
Battery ADC: GPIO 36 (shared with Vext)
```

> **Note:** The BME280 uses a separate I2C bus from the built-in OLED to avoid conflicts.
> GPIO 36 serves dual purpose on the Heltec V3 - both Vext power control and battery ADC input.

## Quick Start

### 1. Install PlatformIO

```bash
pip install platformio
```

### 2. Clone and Build

**BME280 Sensor (default):**
```bash
cd ~/repos/esp32-lora-sensor
pio run -e esp32-lora-sensor
```

**DS18B20 Sensor:**
```bash
cd ~/repos/esp32-lora-sensor
pio run -e esp32-lora-sensor-ds18b20
```

### 3. Upload Filesystem (First Time Only)

```bash
pio run -t uploadfs
```

This uploads the configuration files in `data/` to the ESP32.

### 4. Upload Firmware

**BME280 Sensor:**
```bash
pio run -e esp32-lora-sensor -t upload
```

**DS18B20 Sensor:**
```bash
pio run -e esp32-lora-sensor-ds18b20 -t upload
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

### LittleFS Files

Configuration is stored in `/data/` (LittleFS filesystem):
- `device_name.txt` - Device name sent to gateway (default: "BME280-LoRa-001")
- `deep_sleep_seconds.txt` - Sleep interval between wake cycles (default: 90 seconds)
- `sensor_interval.txt` - Sensor reading interval during wake cycle (default: 30 seconds)
- `pressure_baseline.txt` - Pressure baseline in hPa (0.0 = disabled)

**Device name propagation:**
- Name is loaded from `device_name.txt` on boot
- Sent to gateway in status messages (every 5 wake cycles)
- Gateway automatically updates its registry and MQTT messages
- Full 64-bit device ID used (e.g., `AABBCCDDEEFF0011`)

## Power Consumption

**Typical current draw:**
- Active (sensor read + LoRa TX): ~50-120 mA for ~3 seconds
- Idle between readings: ~20-50 mA (display off)

> **Note:** True deep sleep mode is planned for future versions. Current implementation uses timed delays between readings.

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
- `MSG_READINGS` (0x01) - BME280 sensor data (22 bytes)
- `MSG_STATUS` (0x02) - Device health/diagnostics (88 bytes with name + location)
- `MSG_EVENT` (0x03) - System events (startup, errors, config changes)
- `MSG_COMMAND` (0x10) - Incoming commands from gateway
- `MSG_ACK` (0x20) - Acknowledgments

**Communication flow**:
1. Sensor reads BME280
2. Transmits readings via LoRa
3. Opens RX window to listen for commands
4. Processes any received commands
5. Waits for configured interval before next reading

**Status messages** (every 5 wake cycles):
- Device name from `/data/device_name.txt`
- Location field (empty, reserved for GPS)
- Uptime, wake count, battery, heap
- Sensor and TX failure counts
- Current deep sleep interval

**Event messages**:
- Sent on startup (triggers gateway deduplication buffer reset)
- Config changes
- Sensor errors
- LoRa transmission failures

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
└── data/                # LittleFS filesystem
```

### Building

```bash
# Build BME280 firmware (default)
pio run -e esp32-lora-sensor

# Build DS18B20 firmware
pio run -e esp32-lora-sensor-ds18b20

# Upload firmware (specify environment)
pio run -e esp32-lora-sensor -t upload
pio run -e esp32-lora-sensor-ds18b20 -t upload

# Upload filesystem
pio run -t uploadfs

# Clean build
pio run -t clean
```

## Troubleshooting

### BME280 not reading
- Check BME280 I2C wiring (SDA=33, SCL=26 on Heltec V3)
- Verify BME280 address (0x76 if SDO low, 0x77 if SDO high)
- Ensure Vext is enabled (GPIO 36 LOW)
- The firmware includes automatic I2C error recovery - if readings fail, it will power cycle the sensor and reinitialize

### DS18B20 not reading
- Check DS18B20 wiring: Data pin to GPIO 4 with 4.7K pull-up resistor to 3.3V
- Ensure Vext is enabled (GPIO 36 LOW)
- The firmware includes automatic 1-Wire bus recovery on read failures

### LoRa not transmitting
- Verify antenna is connected (built-in on Heltec V3)
- Check frequency matches your region (915 MHz US, 868 MHz EU)
- Ensure Vext is enabled for LoRa power

### Display not working
- Built-in OLED uses SDA=17, SCL=18, RST=21
- Check I2C address (0x3C expected)
- Ensure Vext is enabled

## Related Projects

- [esp32-lora-gateway](../esp32-lora-gateway/) - Gateway for receiving sensor data

## Shared Components

Both the sensor and gateway share:
- **Protocol library**: `lib/LoRaProtocol/lora_protocol.h` - Binary packet format definitions
- **LoRa configuration**: Must match exactly (frequency, spreading factor, bandwidth, sync word)
- **Hardware**: Heltec WiFi LoRa 32 V3 (ESP32-S3 with built-in SX1262)

## License

MIT License - see LICENSE file
