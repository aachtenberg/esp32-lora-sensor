# ESP32 LoRa Sensor - Wiring Guide

## Hardware
- **Heltec WiFi LoRa 32 V3** (ESP32-S3, SX1262, 0.96" OLED)
- **Sensor**: BME280 (I2C), DHT22 (1-Wire), or DS18B20 (1-Wire) - compile-time selection
- **3.7V LiPo Battery** (with voltage divider for monitoring)

## Pin Configuration Summary

Based on the **Heltec WiFi LoRa 32 V3 (ESP32-S3)** pinout:

### LoRa SX1262 (Built-in - No wiring needed)
| Function | GPIO | Notes |
|----------|------|-------|
| MISO     | 11   | SPI data in |
| MOSI     | 10   | SPI data out |
| SCK      | 9    | SPI clock |
| NSS/CS   | 8    | Chip select |
| DIO1     | 14   | Interrupt |
| BUSY     | 13   | Busy signal |
| RST      | 12   | Reset |

### OLED Display (Built-in - No wiring needed)
| Function | GPIO | Notes |
|----------|------|-------|
| SDA      | 17   | I2C data |
| SCL      | 18   | I2C clock |
| RST      | 21   | Reset |
| Address  | 0x3C | I2C address |

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

**Note:** We use GPIO 33/26 for BME280 to avoid conflicts with the built-in OLED display which uses GPIO 17/18.

### DS18B20 Sensor (External - Wiring Required)

**Connect DS18B20 temperature sensor:**

```
DS18B20 → ESP32 LoRa V3
──────────────────────────
VCC (3.3V)  → 3V
GND         → GND
DATA        → GPIO4 (with 4.7K pull-up to 3.3V)
```

**Wiring diagram with pull-up resistor:**
```
        3.3V
          │
         [4.7K]
          │
GPIO4 ────┼──── DS18B20 DATA
          │
         GND ── DS18B20 GND
          │
        3.3V ── DS18B20 VCC
```

**Pin assignment in code:**
- DS18B20_PIN: GPIO 4

**Note:** The 4.7K pull-up resistor is required for reliable 1-Wire communication. Some DS18B20 breakout boards include this resistor.

### DHT22 Sensor (External - Wiring Required)

**Connect DHT22/AM2302 temperature and humidity sensor:**

```
DHT22/AM2302 → ESP32 LoRa V3
─────────────────────────────
VCC (3.3V)  → 3V
GND         → GND
DATA        → GPIO2
```

**Pin assignment in code:**
- DHT22_PIN: GPIO 2

**⚠️ Important Notes:**
- GPIO2 is a general-purpose GPIO, suitable for 1-Wire communication
- DHT22 sensors have a built-in pull-up resistor, so no external resistor is needed
- If using a bare DHT22 chip (not a breakout module), add a 10K pull-up resistor between DATA and VCC

**Important timing notes:**
- DHT22 requires 2 seconds initial stabilization after power-on
- Minimum 2-second interval between readings
- Serial debugging works normally (GPIO2 doesn't conflict with serial)
- Power cycling requires 500ms off + 1 second stabilization

### GPS Module (Optional - Requires GPS_ENABLED flag)

**Connect NEO-6M GPS module:**

```
NEO-6M GPS → ESP32 LoRa V3
──────────────────────────
VCC (3.3V) → 3V
GND        → GND
TX         → GPIO3 (ESP32 RX)
RX         → GPIO1 (ESP32 TX)
```

**Pin assignments in code:**
- GPS_RX_PIN: GPIO 3 (ESP32 receives from GPS TX)
- GPS_TX_PIN: GPIO 1 (ESP32 transmits to GPS RX)

**⚠️ Important Notes:**
- GPS must be enabled with `-D GPS_ENABLED` build flag
- Uses UART1 for communication at 9600 baud
- GPS needs clear view of sky to acquire satellite fix
- Initial fix can take 30 seconds to several minutes
- When GPS is disabled, GPS fields in readings are set to zero

**Testing GPS:**
- Use `test_gps_pins.cpp` to diagnose wiring issues
- LED on GPS module should flash when receiving satellite data
- Serial output shows satellite count and HDOP values

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

### DS18B20 not detected

1. **Check wiring:**
   - Verify data pin is connected to GPIO 4
   - Ensure 4.7K pull-up resistor is connected between DATA and 3.3V
   - Check power connections (3.3V and GND)

2. **Check pull-up resistor:**
   - Must be 4.7K ohm (not 10K)
   - Some breakout boards include this resistor built-in

3. **Test with multiple sensors:**
   - DS18B20 supports multiple sensors on one bus
   - Each sensor has unique address

4. **Verify sensor is genuine:**
   - Counterfeit DS18B20 sensors are common
   - May not work reliably or give incorrect readings

### DHT22 not detected

1. **Check wiring:**
   - Verify data pin is connected to GPIO 2
   - Check power connections (3.3V and GND)
   - Note: Most DHT22 modules have built-in pull-up resistor

2. **Serial debugging:**
   - GPIO2 doesn't conflict with serial, so debugging works normally
   - This is why GPIO2 is preferred over GPIO1 (which is TX pin)

3. **Timing requirements:**
   - DHT22 needs 2 seconds to stabilize after power-on
   - Minimum 2-second interval between readings

4. **Verify sensor type:**
   - Ensure it's DHT22 (AM2302), not DHT11
   - DHT11 and DHT22 use different protocols

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
- Built-in OLED uses GPIO 17 (SDA) and GPIO 18 (SCL)
- BME280 uses separate GPIO 33 (SDA) and GPIO 26 (SCL)
- These should not conflict (separate I2C buses)

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

## GPIO Usage Summary (Heltec WiFi LoRa 32 V3)

| GPIO | Function | Available? |
|------|----------|------------|
| 8    | LoRa NSS | Used (built-in) |
| 9    | LoRa SCK | Used (built-in) |
| 10   | LoRa MOSI| Used (built-in) |
| 11   | LoRa MISO| Used (built-in) |
| 12   | LoRa RST | Used (built-in) |
| 13   | LoRa BUSY| Used (built-in) |
| 14   | LoRa DIO1| Used (built-in) |
| 17   | OLED SDA | Used (built-in) |
| 18   | OLED SCL | Used (built-in) |
| 21   | OLED RST | Used (built-in) |
| 1    | GPS TX (UART1)| **Used (external, GPS build)** |
| 3    | GPS RX (UART1)| **Used (external, GPS build)** |
| 2    | DHT22 Data | **Used (external, DHT22 build)** |
| 4    | DS18B20 Data | **Used (external, DS18B20 build)** |
| 26   | BME280 SCL| **Used (external, BME280 build)** |
| 33   | BME280 SDA| **Used (external, BME280 build)** |
| 36   | Battery ADC | **Used (optional)** |

**Note:** GPIO usage depends on which sensor build is active (BME280, DHT22, DS18B20, or GPS). Multiple sensors can coexist if GPIO pins don't conflict (e.g., GPS + DHT22 on pins 1,3,2 respectively).

**Available GPIOs for expansion:** 0, 32, 39 (and GPIO 1/2/3/4/26/33 if not using that sensor build)

## References

- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Adafruit BME280 Guide](https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout)
- [DS18B20 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)
- [Dallas Temperature Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
- [DHT22 Datasheet](https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf)
- [Adafruit DHT Library](https://github.com/adafruit/DHT-sensor-library)
