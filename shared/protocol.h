#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// --- LoRa Configuration ---
#define LORA_FREQUENCY      925.0   // MHz
#define LORA_BANDWIDTH      125.0   // kHz
#define LORA_SPREADING      10
#define LORA_CODING_RATE    7
#define LORA_SYNC_WORD      0x34
#define LORA_TX_POWER       0    // dBm
#define LORA_PREAMBLE_LEN   12

// --- SX1262 Pin Definitions (Pico-LoRa-SX1262 on SPI1) ---
#define LORA_SCK   10  
#define LORA_MOSI  11
#define LORA_MISO  12
#define LORA_CS     3
#define LORA_RST   15
#define LORA_DIO1  20
#define LORA_BUSY   2

// --- Message Types ---
#define MSG_TELEMETRY        0x01
#define MSG_CMD_SET_DEVICE   0x02
#define MSG_CMD_SET_FAN      0x03
#define MSG_CMD_ACK          0x04
#define MSG_CMD_REQ_STATUS   0x05
#define MSG_TELEMETRY_EXT    0x06
#define MSG_CMD_SET_RTC      0x07

// --- Packet Header ---
struct PacketHeader {
    uint8_t msgType;
    uint8_t seqNum;
} __attribute__((packed));

// --- Telemetry Payload (sensor -> base) ---
struct TelemetryPayload {
    uint16_t voltage_mV;
    int16_t  current_mA;
    int16_t  temp_C_x10;    // temperature * 10
    uint8_t  fan_duty_pct;
    uint8_t  device_states;  // bitmask: bit0=dev0, bit1=dev1, bit2=dev2
} __attribute__((packed));

// --- Extended Telemetry Payload (sensor -> base, RP2040-LoRa with logger hat) ---
struct TelemetryExtPayload {
    uint16_t voltage_mV;
    int16_t  current_mA;
    int16_t  temp_C_x10;       // DS18B20
    uint8_t  fan_duty_pct;
    uint8_t  device_states;    // bitmask: bit0=dev0, bit1=dev1, bit2=dev2
    // --- logger hat fields ---
    uint16_t humidity_x10;     // SHT40: RH% * 10
    int16_t  sht_temp_C_x10;   // SHT40: temp * 10
    uint16_t lux;              // BH1750
    uint16_t vbat_mV;          // VBAT/2 * 2, in millivolts
    uint32_t rtc_epoch;        // PCF8563: unix timestamp
} __attribute__((packed));

// --- Command: Set Device (base -> sensor) ---
struct CmdSetDevicePayload {
    uint8_t device_id;   // 0-2
    uint8_t state;       // 0=off, 1=on, 2=reset_cycle
} __attribute__((packed));

// --- Command: Set Fan (base -> sensor) ---
struct CmdSetFanPayload {
    uint8_t mode;   // 0=auto, 1=manual
    uint8_t duty;   // 0-100 (only used if mode=manual)
} __attribute__((packed));

// --- Command: ACK (sensor -> base) ---
struct CmdAckPayload {
    uint8_t ackedMsgType;
    uint8_t ackedSeqNum;
    uint8_t status;  // 0=ok, 1=error
} __attribute__((packed));

// --- Command: Set RTC (base -> sensor) ---
struct CmdSetRtcPayload {
    uint32_t epoch;  // unix timestamp
} __attribute__((packed));

// --- CRC8 (Dallas/Maxim) ---
inline uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

// Maximum packet size (header + largest payload + crc)
#define MAX_PACKET_SIZE 32

#endif // PROTOCOL_H
