# Quick Start Guide

Get your ESP32 LoRa sensor running in 10 minutes.

## What You Need

- Heltec WiFi LoRa 32 V3 board
- One sensor: BME280, DHT22, or DS18B20
- USB-C cable
- PlatformIO installed

## Step 1: Wire Your Sensor (2 min)

Choose your sensor:

### BME280 (Temperature + Humidity + Pressure)
```
BME280 → ESP32
VCC → 3V
GND → GND
SDA → GPIO33
SCL → GPIO26
```

### DHT22 (Temperature + Humidity)
```
DHT22 → ESP32
VCC → 3V
GND → GND
DATA → GPIO2
```

### DS18B20 (Temperature Only)
```
DS18B20 → ESP32
VCC → 3V
GND → GND
DATA → GPIO4 (with 4.7K pull-up to 3.3V)
```

**Detailed wiring:** [docs/WIRING_GUIDE.md](WIRING_GUIDE.md)

## Step 2: Build & Upload (3 min)

```bash
cd esp32-lora-sensor

# Choose your sensor:
pio run -e esp32-lora-sensor -t upload              # BME280
pio run -e esp32-lora-sensor-ds18b20 -t upload     # DS18B20
pio run -e esp32-lora-sensor-dht22 -t upload       # DHT22

# Upload config files (first time only)
pio run -t uploadfs
```

## Step 3: Verify (2 min)

Open serial monitor:
```bash
pio device monitor
```

You should see:
```
====================================
ESP32 LoRa Sensor - Startup
====================================
Firmware: v2.0.0

Initializing sensor...
Initializing LoRa radio...
Setup complete!

--- Wake Cycle Start ---
Reading sensor data...
  Temperature: 23.45°C
  Humidity: 55.20%
  Pressure: 1013.25 hPa

Transmitting packet...
✅ Transmission successful!
```

## Step 4: Configure (Optional)

Press **'C'** within 5 seconds of boot to enter config mode:

```
1. Set device name       → Give your sensor a friendly name
2. Set sleep interval    → How often to transmit (default: 90s)
3. Calibrate baseline    → Set pressure reference for weather
4. View config           → See current settings
5. Save and exit         → Apply changes
```

## Common Issues

**"Sensor initialization failed"**
→ Check wiring, verify sensor power

**"LoRa initialization failed"**  
→ Antenna should be connected (built-in on Heltec V3)

**"No readings displayed"**
→ Run test program: [docs/TESTING.md](TESTING.md)

## Next Steps

- [ ] Set up gateway to receive data
- [ ] Configure MQTT broker for data logging
- [ ] Add GPS module (optional): [docs/TEST_GPS.md](TEST_GPS.md)
- [ ] Deploy multiple sensors
- [ ] Monitor via web dashboard

## Full Documentation

- **[README](../README.md)** - Complete project documentation
- **[Wiring Guide](WIRING_GUIDE.md)** - Detailed hardware connections
- **[Testing Guide](TESTING.md)** - Diagnostic tools
- **[GPS Setup](TEST_GPS.md)** - Add location tracking

## Gateway Setup

This sensor transmits data via LoRa. You need a gateway to receive it:

```bash
cd esp32-lora-gateway
pio run -t upload
```

See [esp32-lora-gateway README](../../esp32-lora-gateway/README.md) for gateway setup.
