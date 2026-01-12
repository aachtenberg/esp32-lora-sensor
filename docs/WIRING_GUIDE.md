# ESP32 LoRa Sensor - Wiring Guide

## Hardware
- **ESP32 LoRa V3 Module** (SX1262, 0.96" OLED)
- **BME280 Breakout Board** (I2C)
- **3.7V LiPo Battery** (with voltage divider for monitoring)

## Pin Configuration Summary

Based on the ESP32 LoRa V3 pinout diagram:

### LoRa SX1262 (Built-in - No wiring needed)
| Function | GPIO | Header | Notes |
|----------|------|--------|-------|
| MISO     | 19   | J3     | SPI data in |
| MOSI     | 27   | J3     | SPI data out |
| SCK      | 18   | J2     | SPI clock |
| NSS/CS   | 5    | J3     | Chip select |
| DIO1     | 34   | J2     | Interrupt |
| BUSY     | 35   | J2     | Busy signal |
| RST      | 14   | J3     | Reset |

### OLED Display (Built-in - No wiring needed)
| Function | GPIO | Header | Notes |
|----------|------|--------|-------|
| SDA      | 4    | -      | I2C data |
| SCL      | 15   | -      | I2C clock |
| RST      | 21   | J2     | Reset |

### BME280 Sensor (External - Wiring Required)

**Connect BME280 breakout board to Header J2:**

```
BME280 Breakout → ESP32 LoRa V3 (Header J2)
────────────────────────────────────────────
VCC (3.3V)      → 3V (pin 3)
GND             → GND (pin 1)
SDA             → GPIO33 (pin 12)
SCL             → GPIO26 (pin 15)
```

**Pin assignments in code:**
- BME280_SDA: GPIO 33 (Header J2, pin 12)
- BME280_SCL: GPIO 26 (Header J2, pin 15)
- BME280_ADDR: 0x76 (default, or 0x77 if SDO pulled high)

**Note:** We use GPIO 33/26 for BME280 to avoid conflicts with the built-in OLED display which uses GPIO 4/15.

### Battery Monitoring (Optional)

**Battery voltage divider circuit:**

```
LiPo Battery+ ──[10kΩ]──┬──[10kΩ]── GND
                         │
                    GPIO36 (VP)
```

**Connection:**
- Battery voltage divider output → GPIO 36 (VP pin on J3)
- Voltage divider ratio: 2:1 (using 10K+10K resistors)
- Measures up to 6.6V (suitable for 4.2V LiPo)

**Pin assignment in code:**
- BATTERY_PIN: GPIO 36 (ADC1_CH0, VP)

## Detailed Wiring Diagram

### BME280 to ESP32 LoRa V3

Looking at **Header J2** from the pinout diagram:

| Pin | Label | Function | Connect to BME280 |
|-----|-------|----------|-------------------|
| 1   | GND   | Ground   | GND               |
| 3   | 3V    | 3.3V     | VCC               |
| 12  | GPIO33| I2C SDA  | SDA               |
| 15  | GPIO26| I2C SCL  | SCL               |

### Physical Connection Steps

1. **Power connections:**
   - Connect BME280 VCC to ESP32 3V (Header J2, pin 3)
   - Connect BME280 GND to ESP32 GND (Header J2, pin 1)

2. **I2C connections:**
   - Connect BME280 SDA to ESP32 GPIO33 (Header J2, pin 12)
   - Connect BME280 SCL to ESP32 GPIO26 (Header J2, pin 15)

3. **Battery (optional):**
   - Create voltage divider: Battery+ → 10kΩ → GPIO36 → 10kΩ → GND
   - Connect divider midpoint to GPIO36 (VP pin on Header J3)

## I2C Address Verification

The BME280 has two possible I2C addresses:
- **0x76** (default, SDO pin connected to GND or left floating)
- **0x77** (if SDO pin connected to VCC)

If you experience "sensor not found" errors, try changing the address in `device_config.h`:

```cpp
#define BME280_ADDR    0x77  // Change from 0x76 to 0x77
```

## Troubleshooting

### BME280 not detected

1. **Check I2C address:**
   - Most BME280 breakouts default to 0x76
   - Some use 0x77 - check your module documentation

2. **Check wiring:**
   - Verify SDA/SCL are not swapped
   - Ensure good connections (no loose wires)
   - Check 3.3V power is present

3. **Check I2C pullups:**
   - BME280 breakouts usually have built-in 10K pullup resistors
   - If using bare chip, add 4.7K-10K pullups on SDA/SCL to 3.3V

4. **Test with I2C scanner:**
   - Upload an I2C scanner sketch to verify address
   - Should detect BME280 at 0x76 or 0x77

### OLED conflicts

If OLED doesn't work:
- Built-in OLED uses GPIO 4 (SDA) and GPIO 15 (SCL)
- BME280 uses separate GPIO 33 (SDA) and GPIO 25 (SCL)
- These should not conflict

### Battery reading incorrect

1. **Check voltage divider:**
   - Should be two 10K resistors (2:1 ratio)
   - Midpoint connected to GPIO36

2. **Verify calibration factor:**
   - Measure actual battery voltage with multimeter
   - Adjust `BATTERY_CALIBRATION` in device_config.h
   - Current value: 1.134

3. **Check ADC pin:**
   - GPIO36 is ADC1_CH0 (VP - Voltage Positive input)
   - Input-only pin, no internal pullup available

## GPIO Usage Summary

| GPIO | Function | Available? |
|------|----------|------------|
| 4    | OLED SDA | Used (built-in) |
| 5    | LoRa NSS | Used (built-in) |
| 14   | LoRa RST | Used (built-in) |
| 15   | OLED SCL | Used (built-in) |
| 18   | LoRa SCK | Used (built-in) |
| 19   | LoRa MISO| Used (built-in) |
| 21   | OLED RST | Used (built-in) |
| 26   | BME280 SCL| **Used (external)** |
| 27   | LoRa MOSI| Used (built-in) |
| 33   | BME280 SDA| **Used (external)** |
| 34   | LoRa DIO1| Used (built-in) |
| 35   | LoRa BUSY| Used (built-in) |
| 36   | Battery  | **Used (external)** |

**Available GPIOs for expansion:** 0, 2, 12, 13, 26, 32, 39

## References

- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Adafruit BME280 Guide](https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout)
