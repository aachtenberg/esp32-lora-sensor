/**
 * LED Test Sketch for Heltec WiFi LoRa 32 V3 (ESP32-S3)
 * 
 * Tests the built-in white LED on GPIO35
 * Also enables Vext (GPIO36) which powers the LED and OLED
 */
#include <Arduino.h>

// Heltec V3 pin definitions
#define LED_PIN        35   // Built-in white LED
#define VEXT_CTRL      36   // Vext power control (LOW = ON)

// Also test the RGB LED on newer boards (if present)
#define RGB_LED_PIN    38   // RGB LED on some S3 variants

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n====================================");
    Serial.println("Heltec LED Test");
    Serial.println("====================================");
    
    // Enable Vext power (required for LED and OLED)
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);  // LOW = power ON
    Serial.println("Vext enabled (GPIO36 = LOW)");
    delay(100);
    
    // Configure LED pin
    pinMode(LED_PIN, OUTPUT);
    Serial.printf("LED configured on GPIO%d\n", LED_PIN);
    
    // Also try RGB LED if available
    pinMode(RGB_LED_PIN, OUTPUT);
    Serial.printf("RGB LED configured on GPIO%d\n", RGB_LED_PIN);
    
    Serial.println("\nStarting LED blink test...");
    Serial.println("You should see the white LED blinking.");
    Serial.println("Press BOOT button (GPIO0) to toggle faster blink.\n");
    
    // Boot button for interaction
    pinMode(0, INPUT_PULLUP);
}

void loop() {
    static bool fastBlink = false;
    static unsigned long lastButtonCheck = 0;
    
    // Check boot button for mode toggle
    if (millis() - lastButtonCheck > 200) {
        if (digitalRead(0) == LOW) {
            fastBlink = !fastBlink;
            Serial.printf("Mode: %s blink\n", fastBlink ? "FAST" : "SLOW");
            delay(300);  // Debounce
        }
        lastButtonCheck = millis();
    }
    
    int delayTime = fastBlink ? 100 : 500;
    
    // LED ON
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(RGB_LED_PIN, HIGH);
    Serial.println("LED ON");
    delay(delayTime);
    
    // LED OFF
    digitalWrite(LED_PIN, LOW);
    digitalWrite(RGB_LED_PIN, LOW);
    Serial.println("LED OFF");
    delay(delayTime);
}
