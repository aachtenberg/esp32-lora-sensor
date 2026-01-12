#include "lora_comm.h"
#include "lora_config.h"
#include "device_config.h"
#include "lora_protocol.h"
#include "config_manager.h"
#include <RadioLib.h>
#include <SPI.h>

// SX1262 module instance - pointer to be initialized in initLoRa()
static SX1262* radio = nullptr;

// LoRa communication state
static int16_t lastRSSI = 0;
static int8_t lastSNR = 0;
static uint16_t sequenceNumber = 0;
static uint64_t deviceId = 0;
static bool loraInitialized = false;
static bool restartRequested = false;  // Flag for device restart command

/**
 * Initialize LoRa radio (SX1262)
 * Configures SPI, initializes RadioLib, and sets radio parameters
 */
bool initLoRa() {
    Serial.println("\n=== LoRa Initialization ===");
    
    // Get unique device ID from ESP32 chip ID
    deviceId = ESP.getEfuseMac();
    Serial.printf("Device ID: 0x%016llX\n", deviceId);
    
    // Enable Vext power (Heltec boards require this)
    Serial.print("Enabling Vext power... ");
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);  // LOW = power ON for Heltec boards
    delay(100);  // Wait for power stabilization
    Serial.println("✅");
    
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    
    // Create SX1262 instance
    Serial.print("Creating SX1262 instance... ");
    radio = new SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
    Serial.println("✅");
    
    // Initialize SX1262
    Serial.print("Initializing SX1262... ");
    int state = radio->begin(LORA_FREQUENCY,
                             LORA_BANDWIDTH,
                             LORA_SPREADING,
                             LORA_CODING_RATE,
                             LORA_SYNC_WORD,
                             LORA_TX_POWER,
                             LORA_PREAMBLE_LEN);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("❌ Failed! (code: %d)\n", state);
        delete radio;
        radio = nullptr;
        return false;
    }
    Serial.println("✅");
    
    // Configure CRC
    state = radio->setCRC(LORA_CRC_ENABLED);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("⚠️  CRC config failed (code: %d)\n", state);
    }
    
    // Force explicit header mode (includes length/coding info in packet)
    state = radio->explicitHeader();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("⚠️  Explicit header config failed (code: %d)\n", state);
    }
    
    // Print radio configuration
    Serial.println("\nRadio Configuration:");
    Serial.printf("  Frequency: %.1f MHz\n", LORA_FREQUENCY);
    Serial.printf("  Bandwidth: %.1f kHz\n", LORA_BANDWIDTH);
    Serial.printf("  Spreading Factor: %d\n", LORA_SPREADING);
    Serial.printf("  Coding Rate: 4/%d\n", LORA_CODING_RATE);
    Serial.printf("  TX Power: %d dBm\n", LORA_TX_POWER);
    Serial.printf("  Sync Word: 0x%02X\n", LORA_SYNC_WORD);
    Serial.println("===========================\n");
    
    loraInitialized = true;
    return true;
}

/**
 * Transmit packet with retry logic
 * Builds LoRa packet header and transmits data
 */
bool transmitPacket(const uint8_t* data, size_t len) {
    if (!loraInitialized) {
        Serial.println("❌ LoRa not initialized!");
        return false;
    }
    
    if (len > 255) {
        Serial.printf("❌ Packet too large: %d bytes (max 255)\n", len);
        return false;
    }

    Serial.printf("\n[LoRa TX] Sending %d bytes\n", len);
    
    // Debug: Print raw bytes
    Serial.print("  TX Data: ");
    for (size_t i = 0; i < len && i < 20; i++) {
        Serial.printf("%02X ", data[i]);
    }
    if (len > 20) Serial.print("...");
    Serial.println();
    
    // Retry transmission with exponential backoff
    for (int attempt = 0; attempt <= LORA_TX_RETRIES; attempt++) {
        if (attempt > 0) {
            uint16_t delayMs = LORA_TX_RETRY_DELAY_MS * (1 << (attempt - 1)); // Exponential backoff
            Serial.printf("  Retry %d/%d (waiting %dms)...\n", attempt, LORA_TX_RETRIES, delayMs);
            delay(delayMs);
        } else {
            Serial.print("  Transmitting... ");
        }
        
        // Ensure radio is in standby and ready before transmitting
        // This helps prevent Error -705 (SPI Response Timeout)
        radio->standby();
        delay(10);

        // Transmit packet
        int state = radio->transmit((uint8_t*)data, len);
        
        if (state == RADIOLIB_ERR_NONE) {
            // Get transmission statistics
            lastRSSI = radio->getRSSI();
            lastSNR = radio->getSNR();
            
            Serial.println("✅ Success!");
            Serial.printf("  RSSI: %d dBm\n", lastRSSI);
            Serial.printf("  SNR: %d dB\n", lastSNR);
            
            // Estimate time-on-air (rough calculation)
            float timeOnAir = radio->getTimeOnAir(len) / 1000.0;
            Serial.printf("  Time-on-air: %.2f ms\n", timeOnAir);
            
            return true;
        } else {
            Serial.printf("❌ Failed (code: %d)\n", state);
            
            // Print error description
            if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
                Serial.println("  Error: Packet too long");
            } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
                Serial.println("  Error: TX timeout");
            } else if (state == RADIOLIB_ERR_CHIP_NOT_FOUND) {
                Serial.println("  Error: SX1262 chip not responding");
                loraInitialized = false;
                return false; // Don't retry if chip is dead
            }
        }
    }
    
    Serial.printf("❌ All %d transmission attempts failed!\n", LORA_TX_RETRIES + 1);
    return false;
}

/**
 * Wait for ACK from gateway
 * Listens for ACK packet for specified timeout
 */
bool waitForAck(uint16_t timeout_ms) {
    if (!loraInitialized) {
        return false;
    }
    
    Serial.printf("\n[LoRa RX] Waiting for ACK (timeout: %dms)...\n", timeout_ms);
    
    // Put radio in RX mode with timeout
    int state = radio->startReceive(timeout_ms);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("❌ Failed to enter RX mode (code: %d)\n", state);
        return false;
    }
    
    // Wait for packet or timeout
    uint32_t startTime = millis();
    while (millis() - startTime < timeout_ms) {
        // Check if packet received
        if (radio->available()) {
            uint8_t rxBuffer[sizeof(LoRaPacketHeader) + LORA_MAX_PAYLOAD_SIZE];
            size_t rxLen = 0;
            
            state = radio->readData(rxBuffer, sizeof(rxBuffer));
            
            if (state == RADIOLIB_ERR_NONE) {
                rxLen = radio->getPacketLength();
                
                // Get reception statistics
                lastRSSI = radio->getRSSI();
                lastSNR = radio->getSNR();
                
                Serial.printf("✅ Packet received: %d bytes\n", rxLen);
                Serial.printf("  RSSI: %d dBm, SNR: %d dB\n", lastRSSI, lastSNR);
                
                // Validate header
                if (rxLen >= sizeof(LoRaPacketHeader)) {
                    LoRaPacketHeader* header = (LoRaPacketHeader*)rxBuffer;
                    
                    if (validateHeader(header)) {
                        Serial.printf("  Header valid: Type=0x%02X, Seq=%d\n",
                                    header->msgType, header->sequenceNum);
                        
                        // Check if it's an ACK
                        if (header->msgType == MSG_ACK) {
                            if (rxLen >= sizeof(LoRaPacketHeader) + sizeof(AckPayload)) {
                                AckPayload* ack = (AckPayload*)(rxBuffer + sizeof(LoRaPacketHeader));
                                Serial.printf("  ACK for sequence %d: %s\n",
                                            ack->ackSequenceNum,
                                            ack->success ? "SUCCESS" : "FAILED");
                                return ack->success;
                            }
                        } else {
                            Serial.printf("  ⚠️  Not an ACK (type: 0x%02X)\n", header->msgType);
                        }
                    } else {
                        Serial.println("  ❌ Header validation failed");
                    }
                } else {
                    Serial.println("  ❌ Packet too short for header");
                }
            } else {
                Serial.printf("❌ Read failed (code: %d)\n", state);
            }
        }
        
        delay(10); // Small delay to prevent busy-waiting
    }
    
    Serial.println("⏱️  Timeout - no ACK received");
    return false;
}

/**
 * Check for incoming commands from gateway
 * Listens for command packets for specified timeout
 */
bool checkForCommands(uint16_t timeout_ms) {
    if (!loraInitialized) {
        return false;
    }
    
    Serial.printf("\n[LoRa RX] Checking for commands (timeout: %dms)...\n", timeout_ms);
    
    // Put radio in RX mode with timeout
    int state = radio->startReceive(timeout_ms);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("❌ Failed to enter RX mode (code: %d)\n", state);
        return false;
    }
    
    // Wait for packet or timeout
    uint32_t startTime = millis();
    while (millis() - startTime < timeout_ms) {
        if (radio->available()) {
            uint8_t rxBuffer[sizeof(LoRaPacketHeader) + LORA_MAX_PAYLOAD_SIZE];
            
            state = radio->readData(rxBuffer, sizeof(rxBuffer));
            
            if (state == RADIOLIB_ERR_NONE) {
                size_t rxLen = radio->getPacketLength();
                
                // Get reception statistics
                lastRSSI = radio->getRSSI();
                lastSNR = radio->getSNR();
                
                Serial.printf("✅ Packet received: %d bytes\n", rxLen);
                Serial.printf("  RSSI: %d dBm, SNR: %d dB\n", lastRSSI, lastSNR);
                
                // Validate header
                if (rxLen >= sizeof(LoRaPacketHeader)) {
                    LoRaPacketHeader* header = (LoRaPacketHeader*)rxBuffer;
                    
                    if (validateHeader(header)) {
                        Serial.printf("  Header valid: Type=0x%02X\n", header->msgType);
                        
                        // Check if it's a command
                        if (header->msgType == MSG_COMMAND) {
                            if (rxLen >= sizeof(LoRaPacketHeader) + sizeof(CommandPayload)) {
                                CommandPayload* cmd = (CommandPayload*)(rxBuffer + sizeof(LoRaPacketHeader));
                                Serial.printf("  📡 Command received: Type=0x%02X\n", cmd->cmdType);
                                
                                // Process command and return success/failure
                                bool processed = processCommand(cmd);
                                return processed;
                            }
                        } else {
                            Serial.printf("  ⚠️  Not a command (type: 0x%02X)\n", header->msgType);
                        }
                    } else {
                        Serial.println("  ❌ Header validation failed");
                    }
                }
            } else {
                Serial.printf("❌ Read failed (code: %d)\n", state);
            }
        }
        
        delay(10);
    }
    
    Serial.println("⏱️  No commands received");
    return false;
}

/**
 * Process incoming command from gateway
 * Parses command type and parameters, applies settings
 */
bool processCommand(CommandPayload* cmd) {
    if (!cmd) {
        Serial.println("❌ Invalid command payload");
        return false;
    }
    
    Serial.printf("\n[COMMAND] Processing command type 0x%02X with %d param bytes\n", 
                  cmd->cmdType, cmd->paramLen);
    
    bool success = false;
    char paramStr[64] = "";
    
    // Extract parameter string (null-terminated)
    if (cmd->paramLen > 0 && cmd->paramLen < sizeof(paramStr)) {
        memcpy(paramStr, cmd->params, cmd->paramLen);
        paramStr[cmd->paramLen] = '\0';
        Serial.printf("  Parameters: %s\n", paramStr);
    }
    
    switch (cmd->cmdType) {
        case CMD_SET_SLEEP: {
            // Set deep sleep interval (seconds)
            int newSeconds = atoi(paramStr);
            if (newSeconds >= 0 && newSeconds <= 3600) {  // 0-1 hour
                Serial.printf("  ✓ Deep sleep interval set to %d seconds\n", newSeconds);
                setDeepSleepSeconds(newSeconds);
                success = true;
            } else {
                Serial.printf("  ❌ Invalid sleep value: %d (valid range: 0-3600)\n", newSeconds);
            }
            break;
        }
        
        case CMD_SET_INTERVAL: {
            // Set sensor read interval (seconds)
            int newSeconds = atoi(paramStr);
            if (newSeconds >= 5 && newSeconds <= 3600) {  // 5s - 1 hour
                Serial.printf("  ✓ Sensor interval set to %d seconds\n", newSeconds);
                setSensorIntervalSeconds(newSeconds);
                success = true;
            } else {
                Serial.printf("  ❌ Invalid interval value: %d (valid range: 5-3600)\n", newSeconds);
            }
            break;
        }
        
        case CMD_RESTART: {
            Serial.println("  ✓ Restart command received");
            Serial.println("  Device will restart after command window closes");
            restartRequested = true;
            success = true;
            break;
        }
        
        case CMD_STATUS: {
            Serial.println("  ✓ Status request received");
            // TODO: Send status packet immediately
            success = true;
            break;
        }
        
        case CMD_CALIBRATE: {
            Serial.println("  ✓ Pressure calibration command received");
            // TODO: Call sensor manager to calibrate
            success = true;
            break;
        }
        
        case CMD_CLEAR_BASELINE: {
            Serial.println("  ✓ Clear pressure baseline command received");
            // TODO: Call sensor manager to clear baseline
            success = true;
            break;
        }
        
        case CMD_TIME_SYNC: {
            Serial.println("  ✓ Time sync command received");
            // Extract timestamp from params if provided
            if (cmd->paramLen >= 4) {
                uint32_t timestamp = *(uint32_t*)cmd->params;
                Serial.printf("  Syncing to timestamp: %lu\n", timestamp);
                // TODO: Set system time
            }
            success = true;
            break;
        }
        
        default:
            Serial.printf("  ⚠️  Unknown command type: 0x%02X\n", cmd->cmdType);
    }
    
    if (success) {
        Serial.println("  ✅ Command processed successfully");
    }
    
    return success;
}

/**
 * Get RSSI of last transmission/reception
 */
int16_t getLastRSSI() {
    return lastRSSI;
}

/**
 * Get SNR of last transmission/reception
 */
int8_t getLastSNR() {
    return lastSNR;
}

/**
 * Check if restart was requested via command
 */
bool isRestartRequested() {
    return restartRequested;
}

/**
 * Clear restart flag
 */
void clearRestartFlag() {
    restartRequested = false;
}
