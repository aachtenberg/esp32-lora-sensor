/**
 * OLED Display Test for Heltec WiFi LoRa 32 V3 (ESP32-S3)
 * 
 * Tests the built-in SSD1306 OLED display
 * OLED I2C: SDA=4, SCL=15, RST=21
 * Vext (GPIO36) must be LOW to power the display
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// Heltec V3 OLED pins
#define OLED_SDA       4
#define OLED_SCL       15
#define OLED_RST       21
#define VEXT_CTRL      36   // LOW = power ON

// SSD1306 128x64 display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n====================================");
    Serial.println("Heltec OLED Display Test");
    Serial.println("====================================");
    
    // CRITICAL: Enable Vext power first!
    Serial.println("Enabling Vext power (GPIO36 = LOW)...");
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);  // Let power stabilize
    
    // Reset the OLED manually
    Serial.println("Resetting OLED (GPIO21)...");
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(50);
    digitalWrite(OLED_RST, HIGH);
    delay(50);
    
    // Initialize display
    Serial.println("Initializing U8g2 display...");
    if (!display.begin()) {
        Serial.println("ERROR: Display initialization failed!");
        Serial.println("Check wiring and I2C address (0x3C)");
    } else {
        Serial.println("Display initialized successfully!");
    }
    
    // Set font and draw startup message
    display.setFont(u8g2_font_6x10_tf);
    display.clearBuffer();
    display.drawStr(10, 20, "OLED Test");
    display.drawStr(10, 35, "Heltec V3");
    display.drawStr(10, 50, "Display OK!");
    display.sendBuffer();
    
    Serial.println("Startup message displayed");
    Serial.println("\nDisplay should show 'OLED Test'");
}

void loop() {
    static int counter = 0;
    char buf[32];
    
    display.clearBuffer();
    
    // Title
    display.setFont(u8g2_font_7x14B_tf);
    display.drawStr(5, 14, "OLED Test");
    
    // Counter
    display.setFont(u8g2_font_6x10_tf);
    snprintf(buf, sizeof(buf), "Counter: %d", counter);
    display.drawStr(5, 30, buf);
    
    // Uptime
    snprintf(buf, sizeof(buf), "Uptime: %lus", millis() / 1000);
    display.drawStr(5, 42, buf);
    
    // Draw a moving bar
    int barX = (counter * 3) % 128;
    display.drawBox(barX, 50, 20, 10);
    
    display.sendBuffer();
    
    Serial.printf("Counter: %d\n", counter);
    counter++;
    
    delay(500);
}
