/**
 * GPS Pin Diagnostic Test
 * 
 * This test helps diagnose GPS wiring issues by:
 * 1. Monitoring GPIO3 (GPS TX -> ESP32 RX) for incoming data
 * 2. Sending test data on GPIO1 (ESP32 TX -> GPS RX)
 * 3. Testing with TX/RX swapped in case pins are mislabeled
 * 
 * Upload this, open serial monitor at 115200 baud, and watch output.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

#define GPS_RX_PIN 3   // ESP32 receives from GPS TX
#define GPS_TX_PIN 1   // ESP32 transmits to GPS RX

HardwareSerial gpsSerial(1);

void testBaudRate(uint32_t baud, int duration_ms) {
    Serial.printf("\n=== Testing %d baud for %d ms ===\n", baud, duration_ms);
    
    gpsSerial.end();
    delay(100);
    gpsSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);
    
    unsigned long start = millis();
    int bytesReceived = 0;
    char buffer[256];
    int bufferPos = 0;
    
    while (millis() - start < duration_ms) {
        if (gpsSerial.available()) {
            char c = gpsSerial.read();
            bytesReceived++;
            
            // Print first 80 chars for inspection
            if (bufferPos < 80) {
                buffer[bufferPos++] = c;
            }
            
            // Print byte value for debugging
            if (bytesReceived <= 20) {
                Serial.printf("Byte %d: 0x%02X (%c)\n", bytesReceived, c, 
                             (c >= 32 && c <= 126) ? c : '.');
            }
        }
        delay(1);
    }
    
    if (bytesReceived > 0) {
        buffer[bufferPos] = 0;
        Serial.printf("✅ Received %d bytes at %d baud\n", bytesReceived, baud);
        Serial.println("First 80 chars:");
        Serial.println(buffer);
    } else {
        Serial.printf("❌ No data at %d baud\n", baud);
    }
}

void testSwappedPins(uint32_t baud, int duration_ms) {
    Serial.printf("\n=== Testing SWAPPED pins at %d baud ===\n", baud);
    Serial.println("(GPS TX -> GPIO1, GPS RX -> GPIO3)");
    
    gpsSerial.end();
    delay(100);
    // Swap RX and TX pins
    gpsSerial.begin(baud, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
    delay(100);
    
    unsigned long start = millis();
    int bytesReceived = 0;
    
    while (millis() - start < duration_ms) {
        if (gpsSerial.available()) {
            char c = gpsSerial.read();
            bytesReceived++;
            if (bytesReceived <= 20) {
                Serial.printf("Byte %d: 0x%02X (%c)\n", bytesReceived, c,
                             (c >= 32 && c <= 126) ? c : '.');
            }
        }
        delay(1);
    }
    
    if (bytesReceived > 0) {
        Serial.printf("✅ SWAPPED PINS WORKED! Received %d bytes at %d baud\n", 
                     bytesReceived, baud);
        Serial.println("Your GPS module has TX/RX pins mislabeled!");
    } else {
        Serial.printf("❌ No data with swapped pins at %d baud\n", baud);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("==========================================");
    Serial.println("GPS PIN DIAGNOSTIC TEST");
    Serial.println("==========================================");
    Serial.println("Hardware: NEO-6M GPS Module");
    Serial.println("Expected wiring:");
    Serial.println("  GPS TX -> ESP32 GPIO3 (RX)");
    Serial.println("  GPS RX -> ESP32 GPIO1 (TX)");
    Serial.println("  GPS VCC -> 3.3V");
    Serial.println("  GPS GND -> GND");
    Serial.println("==========================================\n");
    
    delay(1000);
    
    // Test standard configuration at common baud rates
    Serial.println("\n### TEST 1: Standard Pin Configuration ###");
    testBaudRate(9600, 3000);
    testBaudRate(115200, 3000);
    testBaudRate(4800, 3000);
    
    // Test with swapped pins (common issue with clones)
    Serial.println("\n### TEST 2: Swapped Pin Configuration ###");
    Serial.println("(Testing if GPS TX/RX labels are wrong)");
    testSwappedPins(9600, 3000);
    testSwappedPins(115200, 3000);
    
    Serial.println("\n==========================================");
    Serial.println("DIAGNOSTIC COMPLETE");
    Serial.println("==========================================");
    Serial.println("\nResults Summary:");
    Serial.println("- If no data received: GPS may be defective or not powered");
    Serial.println("- If swapped pins worked: Rewire GPS with TX/RX reversed");
    Serial.println("- If data received but garbled: Wrong baud rate");
    Serial.println("- Check LED: Should be solid or flashing when transmitting");
    Serial.println("\n");
}

void loop() {
    // Keep running continuous test
    delay(5000);
    Serial.println("\n--- Running continuous monitor ---");
    testBaudRate(9600, 2000);
}
