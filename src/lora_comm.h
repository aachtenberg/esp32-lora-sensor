#ifndef LORA_COMM_H
#define LORA_COMM_H

#include <Arduino.h>

// Initialize LoRa radio
// Returns true on success, false on failure
bool initLoRa();

// Transmit packet with retry logic
// Returns true if successfully transmitted, false if all retries failed
bool transmitPacket(const uint8_t* data, size_t len);

// Wait for ACK from gateway
// Returns true if ACK received, false on timeout
bool waitForAck(uint16_t timeout_ms);

// Check for incoming commands from gateway
// Returns true if command received and processed
bool checkForCommands(uint16_t timeout_ms);

// Get RSSI of last transmission
int16_t getLastRSSI();

// Get SNR of last transmission
int8_t getLastSNR();

#endif // LORA_COMM_H
