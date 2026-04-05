#include <Arduino.h>
#include <DHT.h>
#include "device_config.h"

#ifndef DHT22_DIAG_PIN
#define DHT22_DIAG_PIN DHT22_PIN
#endif
static DHT dht(DHT22_DIAG_PIN, DHT22);

static void testPinDrive() {
    Serial.print("Drive test on GPIO");
    Serial.println(DHT22_DIAG_PIN);

    pinMode(DHT22_DIAG_PIN, OUTPUT);
    digitalWrite(DHT22_DIAG_PIN, LOW);
    delay(5);
    Serial.print("  While driving LOW, readback: ");
    Serial.println(digitalRead(DHT22_DIAG_PIN) ? "HIGH" : "LOW");

    pinMode(DHT22_DIAG_PIN, INPUT_PULLUP);
    delay(5);
    Serial.print("  Released with pull-up, readback: ");
    Serial.println(digitalRead(DHT22_DIAG_PIN) ? "HIGH" : "LOW");

    pinMode(DHT22_DIAG_PIN, OUTPUT);
    digitalWrite(DHT22_DIAG_PIN, HIGH);
    delay(5);
    Serial.print("  While driving HIGH, readback: ");
    Serial.println(digitalRead(DHT22_DIAG_PIN) ? "HIGH" : "LOW");

    pinMode(DHT22_DIAG_PIN, INPUT_PULLUP);
    delay(5);
    Serial.print("  Released high, readback: ");
    Serial.println(digitalRead(DHT22_DIAG_PIN) ? "HIGH" : "LOW");
}

static void printPinState() {
    pinMode(DHT22_DIAG_PIN, INPUT_PULLUP);
    delay(5);

    Serial.print("Idle DATA state on GPIO");
    Serial.print(DHT22_DIAG_PIN);
    Serial.print(": ");
    Serial.println(digitalRead(DHT22_DIAG_PIN) ? "HIGH" : "LOW");
}

static bool readDht(float* temperature, float* humidity) {
    if (temperature == nullptr || humidity == nullptr) {
        return false;
    }

    *humidity = dht.readHumidity();
    *temperature = dht.readTemperature();
    return !isnan(*temperature) && !isnan(*humidity);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n================================");
    Serial.println("DHT22 Diagnostic Test");
    Serial.println("================================");
    Serial.print("Data pin: GPIO");
    Serial.println(DHT22_DIAG_PIN);
    Serial.println("Expected wiring:");
    Serial.println("  VCC  -> 3.3V");
    Serial.println("  GND  -> GND");
    Serial.print("  DATA -> GPIO");
    Serial.println(DHT22_DIAG_PIN);
    Serial.println("  Pull-up: 10k to 3.3V if sensor is bare");
    Serial.println("================================\n");

    pinMode(DHT22_DIAG_PIN, INPUT_PULLUP);
    printPinState();
    testPinDrive();

    dht.begin(); 
    delay(2500);

    float temperature = NAN;
    float humidity = NAN;
    if (readDht(&temperature, &humidity)) {
        Serial.printf("Initial read OK: %.2f C, %.2f %%\n", temperature, humidity);
    } else {
        Serial.println("Initial read failed (NaN)");
    }

    Serial.println();
}

void loop() {
    float temperature = NAN;
    float humidity = NAN;

    printPinState();
    testPinDrive();

    if (readDht(&temperature, &humidity)) {
        Serial.printf("DHT22 read OK: %.2f C, %.2f %%\n", temperature, humidity);
    } else {
        Serial.println("DHT22 read failed (NaN)");
    }

    Serial.println();
    delay(3000);
}