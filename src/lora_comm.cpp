#include "lora_comm.h"
#include "lora_config.h"
#include "device_config.h"
#include "lora_protocol.h"
#include "config_manager.h"
#include "sensor_manager.h"
#include <RadioLib.h>
#include <SPI.h>

// SX1262 module instance - pointer to be initialized in initLoRa()
static SX1262* radio = nullptr;

// LoRa communication state
static int16_t lastRSSI = 0;
static int8_t lastSNR = 0;
static uint64_t deviceId = 0;
static bool loraInitialized = false;
static bool restartRequested = false;  // Flag for device restart command
static uint16_t txFailures = 0;  // Count transmission failures
static uint32_t lastSuccessTx = 0;  // Timestamp of last successful transmission
static uint16_t lastTimeOnAir = 0;  // Last transmission time-on-air (ms)
static int8_t txPowerDbm = 0;  // Current TX power

// External global sequence number from main.cpp
extern uint16_t g_sequenceNumber;

/**
 * Initialize LoRa radio (SX1262)
 * Configures SPI, initializes RadioLib, and sets radio parameters
 */
bool initLoRa() {
    Serial.printf("[%lu] === LoRa Initialization ===\n", millis());
    
    // Get unique device ID from ESP32 chip ID
    deviceId = ESP.getEfuseMac();
    Serial.printf("[%lu] Device ID: 0x%016llX\n", millis(), deviceId);
    
    // Enable Vext power (Heltec boards require this)
    Serial.printf("[%lu] Enabling Vext power... ", millis());
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);  // LOW = power ON for Heltec boards
    delay(100);  // Wait for power stabilization
    Serial.println("✅");
    
    // Initialize SPI for LoRa module
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    
    // Create SX1262 instance
    Serial.printf("[%lu] Creating SX1262 instance... ", millis());
    radio = new SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY));
    Serial.println("✅");
    
    // Initialize SX1262
    Serial.printf("[%lu] Initializing SX1262... ", millis());
    txPowerDbm = LORA_TX_POWER;  // Store TX power
    int state = radio->begin(LORA_FREQUENCY,
                             LORA_BANDWIDTH,
                             LORA_SPREADING,
                             LORA_CODING_RATE,
                             LORA_SYNC_WORD,
                             LORA_TX_POWER,
                             LORA_PREAMBLE_LEN);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[%lu] ❌ Failed! (code: %d)\n", millis(), state);
        delete radio;
        radio = nullptr;
        return false;
    }
    Serial.println("✅");
    
    // Configure CRC
    state = radio->setCRC(LORA_CRC_ENABLED);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[%lu] ⚠️  CRC config failed (code: %d)\n", millis(), state);
    }
    
    // Force explicit header mode (includes length/coding info in packet)
    state = radio->explicitHeader();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[%lu] ⚠️  Explicit header config failed (code: %d)\n", millis(), state);
    }
    
    // Print radio configuration
    Serial.printf("[%lu] Radio Configuration:\n", millis());
    Serial.printf("[%lu]   Frequency: %.1f MHz\n", millis(), LORA_FREQUENCY);
    Serial.printf("[%lu]   Bandwidth: %.1f kHz\n", millis(), LORA_BANDWIDTH);
    Serial.printf("[%lu]   Spreading Factor: %d\n", millis(), LORA_SPREADING);
    Serial.printf("[%lu]   Coding Rate: 4/%d\n", millis(), LORA_CODING_RATE);
    Serial.printf("[%lu]   TX Power: %d dBm\n", millis(), LORA_TX_POWER);
    Serial.printf("[%lu]   Sync Word: 0x%02X\n", millis(), LORA_SYNC_WORD);
    Serial.printf("[%lu] ===========================\n", millis());
    
    loraInitialized = true;
    return true;
}

/**
 * Transmit packet with retry logic
 * Builds LoRa packet header and transmits data
 */
bool transmitPacket(const uint8_t* data, size_t len) {
    if (!loraInitialized) {
        Serial.printf("[%lu] ❌ LoRa not initialized!\n", millis());
        return false;
    }
    
    if (len > 255) {
        Serial.printf("[%lu] ❌ Packet too large: %d bytes (max 255)\n", millis(), len);
        return false;
    }

    Serial.printf("[%lu] [LoRa TX] Sending %d bytes\n", millis(), len);
    
    // Debug: Print raw bytes
    Serial.printf("[%lu]   TX Data: ", millis());
    for (size_t i = 0; i < len && i < 20; i++) {
        Serial.printf("%02X ", data[i]);
    }
    if (len > 20) Serial.print("...");
    Serial.println();
    
    // Retry transmission with exponential backoff
    for (int attempt = 0; attempt <= LORA_TX_RETRIES; attempt++) {
        if (attempt > 0) {
            uint16_t delayMs = LORA_TX_RETRY_DELAY_MS * (1 << (attempt - 1)); // Exponential backoff
            Serial.printf("[%lu]   Retry %d/%d (waiting %dms)...\n", millis(), attempt, LORA_TX_RETRIES, delayMs);
            delay(delayMs);
        } else {
            Serial.printf("[%lu]   Transmitting... ", millis());
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
            lastSuccessTx = millis() / 1000;
            lastTimeOnAir = (uint16_t)(radio->getTimeOnAir(len) / 1000.0); // Convert to ms
            
            Serial.println("✅ Success!");
            Serial.printf("[%lu]   RSSI: %d dBm, SNR: %d dB, ToA: %u ms\n", millis(), lastRSSI, lastSNR, lastTimeOnAir);
            
            return true;
        } else {
            txFailures++;
            Serial.printf("[%lu] ❌ Failed (code: %d)\n", millis(), state);
            
            // Print error description
            if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
                Serial.printf("[%lu]   Error: Packet too long\n", millis());
            } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
                Serial.printf("[%lu]   Error: TX timeout\n", millis());
            } else if (state == RADIOLIB_ERR_CHIP_NOT_FOUND) {
                Serial.printf("[%lu]   Error: SX1262 chip not responding\n", millis());
                loraInitialized = false;
                return false; // Don't retry if chip is dead
            }
        }
    }
    
    Serial.printf("[%lu] ❌ All %d transmission attempts failed!\n", millis(), LORA_TX_RETRIES + 1);
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
    
    Serial.printf("[%lu] [LoRa RX] Waiting for ACK (timeout: %dms)\n", millis(), timeout_ms);

    // Put radio in continuous RX mode (no timeout parameter for better reliability)
    int state = radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[%lu] ❌ Failed to enter RX mode (code: %d)\n", millis(), state);
        return false;
    }

    // Wait for packet or timeout by polling DIO1 pin directly
    uint32_t startTime = millis();
    while (millis() - startTime < timeout_ms) {
        // Check DIO1 pin for IRQ (packet received)
        if (digitalRead(LORA_DIO1) == HIGH) {
            uint8_t rxBuffer[sizeof(LoRaPacketHeader) + LORA_MAX_PAYLOAD_SIZE];
            size_t rxLen = 0;
            
            state = radio->readData(rxBuffer, sizeof(rxBuffer));
            
            if (state == RADIOLIB_ERR_NONE) {
                rxLen = radio->getPacketLength();
                
                // Get reception statistics
                lastRSSI = radio->getRSSI();
                lastSNR = radio->getSNR();
                
                Serial.printf("[%lu] ✅ Packet received: %d bytes\n", millis(), rxLen);
                Serial.printf("[%lu]   RSSI: %d dBm, SNR: %d dB\n", millis(), lastRSSI, lastSNR);
                
                // Validate header
                if (rxLen >= sizeof(LoRaPacketHeader)) {
                    LoRaPacketHeader* header = (LoRaPacketHeader*)rxBuffer;
                    
                    if (validateHeader(header)) {
                        Serial.printf("[%lu]   Header valid: Type=0x%02X, Seq=%d\n",
                                    millis(), header->msgType, header->sequenceNum);
                        
                        // Check if it's an ACK
                        if (header->msgType == MSG_ACK) {
                            if (rxLen >= sizeof(LoRaPacketHeader) + sizeof(AckPayload)) {
                                AckPayload* ack = (AckPayload*)(rxBuffer + sizeof(LoRaPacketHeader));
                                Serial.printf("[%lu]   ACK for sequence %d: %s\n",
                                            millis(), ack->ackSequenceNum,
                                            ack->success ? "SUCCESS" : "FAILED");
                                return ack->success;
                            }
                        } else {
                            Serial.printf("[%lu]   ⚠️  Not an ACK (type: 0x%02X)\n", millis(), header->msgType);
                        }
                    } else {
                        Serial.printf("[%lu]   ❌ Header validation failed\n", millis());
                    }
                } else {
                    Serial.printf("[%lu]   ❌ Packet too short for header\n", millis());
                }
            } else {
                Serial.printf("[%lu] ❌ Read failed (code: %d)\n", millis(), state);
            }
        }
        
        delay(10); // Small delay to prevent busy-waiting
    }

    // Timeout - no ACK received, put radio back in standby
    Serial.printf("[%lu] Timeout - no ACK received\n", millis());
    radio->standby();
    return false;
}

/**
 * Check for incoming commands from gateway
 * Listens for command packets for specified timeout
 */
bool checkForCommands(uint16_t timeout_ms) {
    uint32_t funcEntryMs = millis();
    Serial.printf("[%lu] checkForCommands() entered\n", funcEntryMs);
    
    if (!loraInitialized) {
        return false;
    }
    
    uint32_t rxStartMs = millis();
    Serial.printf("[%lu] Starting RX (timeout: %dms)\n", rxStartMs, timeout_ms);

    // Put radio in continuous RX mode (no timeout parameter for better reliability)
    int state = radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[%lu] ❌ Failed to enter RX mode (code: %d)\n", millis(), state);
        return false;
    }
    
    Serial.printf("[%lu] Radio in RX mode\n", millis());

    // Wait for packet or timeout by polling DIO1 pin directly
    uint32_t startTime = millis();
    while (millis() - startTime < timeout_ms) {
        // Check DIO1 pin for IRQ (packet received)
        if (digitalRead(LORA_DIO1) == HIGH) {
            Serial.printf("[%lu] 📡 DIO1 triggered - packet detected\n", millis());
            uint8_t rxBuffer[sizeof(LoRaPacketHeader) + LORA_MAX_PAYLOAD_SIZE];
            
            state = radio->readData(rxBuffer, sizeof(rxBuffer));
            
            if (state == RADIOLIB_ERR_NONE) {
                size_t rxLen = radio->getPacketLength();
                
                // Get reception statistics
                lastRSSI = radio->getRSSI();
                lastSNR = radio->getSNR();
                
                Serial.printf("[%lu] ✅ Packet received: %d bytes\n", millis(), rxLen);
                Serial.printf("[%lu]   RSSI: %d dBm, SNR: %d dB\n", millis(), lastRSSI, lastSNR);
                
                // Validate header
                if (rxLen >= sizeof(LoRaPacketHeader)) {
                    LoRaPacketHeader* header = (LoRaPacketHeader*)rxBuffer;
                    
                    if (validateHeader(header)) {
                        Serial.printf("[%lu]   Header valid: Type=0x%02X\n", millis(), header->msgType);
                        
                        // Check if it's a command
                        if (header->msgType == MSG_COMMAND) {
                            // Validate we received the full payload (use header's payloadLen, not full struct size)
                            if (rxLen >= sizeof(LoRaPacketHeader) + header->payloadLen) {
                                CommandPayload* cmd = (CommandPayload*)(rxBuffer + sizeof(LoRaPacketHeader));
                                Serial.printf("[%lu]   📡 Command received: Type=0x%02X, ParamLen=%d\n",
                                             millis(), cmd->cmdType, cmd->paramLen);

                                // Process command and return success/failure
                                bool processed = processCommand(cmd);
                                return processed;
                            } else {
                                Serial.printf("[%lu]   ❌ Incomplete command packet (got %d bytes, expected %d)\n",
                                             millis(), rxLen, sizeof(LoRaPacketHeader) + header->payloadLen);
                            }
                        } else {
                            Serial.printf("[%lu]   ⚠️  Not a command (type: 0x%02X)\n", millis(), header->msgType);
                        }
                    } else {
                        Serial.printf("[%lu]   ❌ Header validation failed\n", millis());
                    }
                }
            } else {
                Serial.printf("[%lu] ❌ Read failed (code: %d)\n", millis(), state);
            }
        }
        
        delay(10);
    }

    // Timeout - no commands received, put radio back in standby
    Serial.printf("[%lu] No commands received (timeout)\n", millis());
    radio->standby();
    return false;
}

/**
 * Process incoming command from gateway
 * Parses command type and parameters, applies settings
 */
bool processCommand(CommandPayload* cmd) {
    if (!cmd) {
        Serial.printf("[%lu] ❌ Invalid command payload\n", millis());
        return false;
    }
    
    Serial.printf("[%lu] [COMMAND] Processing command type 0x%02X with %d param bytes\n", 
                  millis(), cmd->cmdType, cmd->paramLen);
    
    bool success = false;
    
    switch (cmd->cmdType) {
        case CMD_SET_SLEEP: {
            // Set deep sleep interval (seconds) - expects uint16_t
            if (cmd->paramLen == sizeof(uint16_t)) {
                uint16_t newSeconds = *(uint16_t*)cmd->params;
                Serial.printf("  Parameters: %u seconds\n", newSeconds);
                if (newSeconds >= 0 && newSeconds <= 3600) {  // 0-1 hour
                    Serial.printf("  ✓ Deep sleep interval set to %u seconds\n", newSeconds);
                    setDeepSleepSeconds(newSeconds);
                    
                    char eventMsg[64];
                    snprintf(eventMsg, sizeof(eventMsg), "Sleep interval changed to %us", newSeconds);
                    sendEventMessage(EVENT_CONFIG_CHANGE, SEVERITY_INFO, eventMsg);
                    
                    success = true;
                } else {
                    Serial.printf("  ❌ Invalid sleep value: %u (valid range: 0-3600)\n", newSeconds);
                }
            } else {
                Serial.printf("  ❌ Invalid parameter length: %d (expected 2)\n", cmd->paramLen);
            }
            break;
        }
        
        case CMD_SET_INTERVAL: {
            // Set sensor read interval (seconds) - expects uint16_t
            if (cmd->paramLen == sizeof(uint16_t)) {
                uint16_t newSeconds = *(uint16_t*)cmd->params;
                Serial.printf("  Parameters: %u seconds\n", newSeconds);
                if (newSeconds >= 5 && newSeconds <= 3600) {  // 5s - 1 hour
                    Serial.printf("  ✓ Sensor interval set to %u seconds\n", newSeconds);
                    setSensorIntervalSeconds(newSeconds);
                    
                    char eventMsg[64];
                    snprintf(eventMsg, sizeof(eventMsg), "Sensor interval changed to %us", newSeconds);
                    sendEventMessage(EVENT_CONFIG_CHANGE, SEVERITY_INFO, eventMsg);
                    
                    success = true;
                } else {
                    Serial.printf("  ❌ Invalid interval value: %u (valid range: 5-3600)\n", newSeconds);
                }
            } else {
                Serial.printf("  ❌ Invalid parameter length: %d (expected 2)\n", cmd->paramLen);
            }
            break;
        }
        
        case CMD_RESTART: {
            Serial.println("  ✓ Restart command received");
            Serial.println("  Device will restart after command window closes");
            
            sendEventMessage(EVENT_SHUTDOWN, SEVERITY_INFO, "Restart command received");
            
            restartRequested = true;
            success = true;
            break;
        }
        
        case CMD_STATUS: {
            Serial.println("  ✓ Status request received");
            // Send status packet immediately
            sendStatusMessage();
            success = true;
            break;
        }
        
        case CMD_CALIBRATE: {
            Serial.println("  ✓ Pressure calibration command received");
            calibrateBaseline();
            success = true;
            break;
        }

        case CMD_SET_BASELINE: {
            // Set specific baseline value (hPa) - expects float (4 bytes)
            if (cmd->paramLen == sizeof(float)) {
                float baselineHpa = *(float*)cmd->params;
                Serial.printf("  Parameters: %.2f hPa\n", baselineHpa);
                if (baselineHpa > 900 && baselineHpa < 1100) {  // Sanity check
                    float baselinePa = baselineHpa * 100.0;  // Convert hPa to Pa
                    Serial.printf("  ✓ Setting baseline to %.2f hPa\n", baselineHpa);
                    setPressureBaseline(baselinePa);
                    success = true;
                } else {
                    Serial.printf("  ❌ Invalid baseline: %.2f (valid range: 900-1100 hPa)\n", baselineHpa);
                }
            } else {
                Serial.printf("  ❌ Invalid parameter length: %d (expected 4)\n", cmd->paramLen);
            }
            break;
        }

        case CMD_CLEAR_BASELINE: {
            Serial.println("  ✓ Clear pressure baseline command received");
            clearBaseline();
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
 * Get transmission failure count
 */
uint16_t getTxFailures() {
    return txFailures;
}

/**
 * Send status message to gateway
 */
bool sendStatusMessage() {
    if (!loraInitialized) {
        Serial.println("❌ LoRa not initialized!");
        return false;
    }
    
    extern uint32_t g_wakeCount;
    extern uint64_t g_deviceId;
    
    Serial.println("\n📊 Sending STATUS message...");
    
    // Build status payload
    StatusPayload status;
    status.uptime = millis() / 1000;
    status.wakeCount = g_wakeCount;
    status.sensorHealthy = 1;  // TODO: Track sensor health
    status.loraRssi = lastRSSI;
    status.loraSNR = lastSNR;
    status.freeHeap = ESP.getFreeHeap() / 1024;
    status.sensorFailures = getSensorFailures();
    status.txFailures = txFailures;
    status.lastSuccessTx = lastSuccessTx;
    status.deepSleepSec = getDeepSleepSeconds();
    status.sensorIntervalSec = getSensorIntervalSeconds();
    status.timeOnAir = lastTimeOnAir;
    status.txPower = txPowerDbm;
    
    // Include device name from config
    String deviceName = getDeviceName();
    strncpy(status.deviceName, deviceName.c_str(), sizeof(status.deviceName) - 1);
    status.deviceName[sizeof(status.deviceName) - 1] = '\0';  // Ensure null termination
    
    // Include location (empty for now, future: GPS coordinates)
    status.location[0] = '\0';  // Empty for now, will be populated by GPS in future
    
    // Build packet
    uint8_t packet[sizeof(LoRaPacketHeader) + sizeof(StatusPayload)];
    LoRaPacketHeader* header = (LoRaPacketHeader*)packet;
    
    initHeader(header, MSG_STATUS, g_deviceId, g_sequenceNumber++, sizeof(StatusPayload));
    memcpy(packet + sizeof(LoRaPacketHeader), &status, sizeof(StatusPayload));
    
    // Transmit
    bool success = transmitPacket(packet, sizeof(packet));
    
    if (success) {
        Serial.println("✅ Status message sent");
    } else {
        Serial.println("❌ Status message failed");
    }
    
    return success;
}

/**
 * Send event message to gateway
 */
bool sendEventMessage(uint8_t eventType, uint8_t severity, const char* message) {
    if (!loraInitialized) {
        Serial.println("❌ LoRa not initialized!");
        return false;
    }
    
    extern uint64_t g_deviceId;
    
    Serial.printf("\n📢 Sending EVENT: %s\n", message);
    
    // Build event payload
    EventPayload event;
    event.eventType = eventType;
    event.severity = severity;
    event.messageLen = strlen(message);
    
    // Truncate message if too long
    if (event.messageLen > sizeof(event.message) - 1) {
        event.messageLen = sizeof(event.message) - 1;
    }
    
    memcpy(event.message, message, event.messageLen);
    event.message[event.messageLen] = '\0';
    
    // Build packet
    uint8_t payloadLen = 3 + event.messageLen;
    uint8_t packet[sizeof(LoRaPacketHeader) + payloadLen];
    LoRaPacketHeader* header = (LoRaPacketHeader*)packet;
    
    initHeader(header, MSG_EVENT, g_deviceId, g_sequenceNumber++, payloadLen);
    memcpy(packet + sizeof(LoRaPacketHeader), &event, payloadLen);
    
    // Transmit
    bool success = transmitPacket(packet, sizeof(LoRaPacketHeader) + payloadLen);
    
    if (success) {
        Serial.println("✅ Event message sent");
    } else {
        Serial.println("❌ Event message failed");
    }
    
    return success;
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
