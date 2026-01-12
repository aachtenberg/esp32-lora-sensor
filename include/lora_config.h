#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

// ====================================================================
// LoRa Radio Configuration
// ====================================================================

// Frequency Configuration
// US: 902-928 MHz (typically 915.0)
// EU: 863-870 MHz (typically 868.0)
// Choose based on your region and module specs
#define LORA_FREQUENCY      915.0   // MHz

// Radio Parameters (balanced for range and power)
#define LORA_BANDWIDTH      125.0   // kHz (125, 250, 500)
#define LORA_SPREADING      9       // SF7-SF12 (higher = longer range, slower)
#define LORA_CODING_RATE    7       // 4/5=5, 4/6=6, 4/7=7, 4/8=8
#define LORA_TX_POWER       14      // dBm (start conservative, max ~20)
#define LORA_PREAMBLE_LEN   8       // Standard preamble length
#define LORA_SYNC_WORD      0x12    // Private network sync word

// Expected Performance @ SF9/125kHz:
// - Range: 1-2 km urban, 5-10 km rural
// - Air time: ~300ms per 53-byte packet
// - Battery: ~120mA during TX

// Transmission Settings
#define LORA_TX_RETRIES     3       // Number of retry attempts
#define LORA_TX_RETRY_DELAY_MS 100  // Initial retry delay (exponential backoff)
#define LORA_ACK_TIMEOUT_MS 2000    // Wait time for ACK from gateway
#define LORA_COMMAND_TIMEOUT_MS 10000 // Listen time for commands from gateway (10 seconds)

// CRC and Error Detection
#define LORA_CRC_ENABLED    true    // Hardware CRC check

#endif // LORA_CONFIG_H
