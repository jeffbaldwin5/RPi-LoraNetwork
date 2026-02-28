#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "protocol.h"

// --- LoRa (SPI1) ---
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI1);

// --- State ---
static uint8_t seqNum = 0;
static uint32_t lastTelemetryTime = 0;
static volatile bool rxFlag = false;

// --- Serial input buffer ---
static char serialBuf[256];
static size_t serialBufPos = 0;

// --- Forward declarations ---
void initLoRa();
void handleLoRaRx();
void handleSerialInput();
void processSerialCommand(const char* json);
void sendDeviceCommand(uint8_t deviceId, uint8_t state);
void sendFanCommand(uint8_t mode, uint8_t duty);
void sendStatusRequest();

// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("{\"status\":\"Base station starting...\"}"));

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    initLoRa();

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("{\"status\":\"Base station ready\"}"));
}

void loop() {
    handleLoRaRx();
    handleSerialInput();
}

// ============================================================
// LoRa
// ============================================================
void initLoRa() {
    SPI1.setRX(LORA_MISO);
    SPI1.setTX(LORA_MOSI);
    SPI1.setSCK(LORA_SCK);
    SPI1.begin();

    int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING,
                            LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("{\"error\":\"LoRa init failed, code %d\"}\n", state);
        while (true) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            delay(200);
        }
    }

    radio.setDio1Action([]() { rxFlag = true; });
    radio.startReceive();
    Serial.println(F("{\"status\":\"LoRa initialized\"}"));
}

void handleLoRaRx() {
    if (!rxFlag) return;
    rxFlag = false;

    size_t len = radio.getPacketLength();
    if (len == 0) return;

    uint8_t buf[MAX_PACKET_SIZE];
    int state = radio.readData(buf, len);
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE || len < 3) return;

    // Verify CRC
    uint8_t rxCrc = buf[len - 1];
    uint8_t calcCrc = crc8(buf, len - 1);
    if (rxCrc != calcCrc) {
        Serial.println(F("{\"error\":\"CRC mismatch\"}"));
        return;
    }

    PacketHeader hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    switch (hdr.msgType) {
        case MSG_TELEMETRY: {
            if (len - 1 < sizeof(PacketHeader) + sizeof(TelemetryPayload)) break;
            TelemetryPayload tel;
            memcpy(&tel, buf + sizeof(PacketHeader), sizeof(tel));
            lastTelemetryTime = millis();

            digitalWrite(PIN_LED, HIGH);

            Serial.printf("{\"voltage_mV\":%u,\"current_mA\":%d,\"temp_C\":%.1f,"
                          "\"fan_pct\":%u,\"devices\":[%d,%d,%d],"
                          "\"rssi\":%.1f,\"snr\":%.1f}\n",
                          tel.voltage_mV,
                          tel.current_mA,
                          tel.temp_C_x10 / 10.0f,
                          tel.fan_duty_pct,
                          (tel.device_states >> 0) & 1,
                          (tel.device_states >> 1) & 1,
                          (tel.device_states >> 2) & 1,
                          rssi, snr);

            digitalWrite(PIN_LED, LOW);
            break;
        }
        case MSG_CMD_ACK: {
            if (len - 1 < sizeof(PacketHeader) + sizeof(CmdAckPayload)) break;
            CmdAckPayload ack;
            memcpy(&ack, buf + sizeof(PacketHeader), sizeof(ack));
            Serial.printf("{\"ack\":{\"msg_type\":%u,\"seq\":%u,\"status\":%u}}\n",
                          ack.ackedMsgType, ack.ackedSeqNum, ack.status);
            break;
        }
        default:
            break;
    }
}

// ============================================================
// Serial Command Parsing (simple JSON)
// ============================================================
void handleSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBufPos > 0) {
                serialBuf[serialBufPos] = '\0';
                processSerialCommand(serialBuf);
                serialBufPos = 0;
            }
        } else if (serialBufPos < sizeof(serialBuf) - 1) {
            serialBuf[serialBufPos++] = c;
        }
    }
}

// Minimal JSON key-value extractor (no external JSON lib needed)
static bool findJsonString(const char* json, const char* key, char* out, size_t outLen) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char* pos = strstr(json, search);
    if (!pos) return false;
    pos = strchr(pos + strlen(search), ':');
    if (!pos) return false;
    pos++;
    while (*pos == ' ') pos++;
    if (*pos == '"') {
        pos++;
        size_t i = 0;
        while (*pos && *pos != '"' && i < outLen - 1) {
            out[i++] = *pos++;
        }
        out[i] = '\0';
        return true;
    }
    return false;
}

static bool findJsonInt(const char* json, const char* key, int* out) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char* pos = strstr(json, search);
    if (!pos) return false;
    pos = strchr(pos + strlen(search), ':');
    if (!pos) return false;
    pos++;
    while (*pos == ' ') pos++;
    *out = atoi(pos);
    return true;
}

void processSerialCommand(const char* json) {
    char cmd[32];
    if (!findJsonString(json, "cmd", cmd, sizeof(cmd))) {
        Serial.println(F("{\"error\":\"missing cmd field\"}"));
        return;
    }

    if (strcmp(cmd, "device") == 0) {
        int id = 0;
        char state[16];
        if (!findJsonInt(json, "id", &id) || !findJsonString(json, "state", state, sizeof(state))) {
            Serial.println(F("{\"error\":\"device cmd requires id and state\"}"));
            return;
        }
        uint8_t stateVal = 0;
        if (strcmp(state, "on") == 0) stateVal = 1;
        else if (strcmp(state, "reset") == 0) stateVal = 2;
        sendDeviceCommand((uint8_t)id, stateVal);

    } else if (strcmp(cmd, "fan") == 0) {
        char mode[16];
        if (!findJsonString(json, "mode", mode, sizeof(mode))) {
            Serial.println(F("{\"error\":\"fan cmd requires mode\"}"));
            return;
        }
        if (strcmp(mode, "auto") == 0) {
            sendFanCommand(0, 0);
        } else if (strcmp(mode, "manual") == 0) {
            int duty = 0;
            findJsonInt(json, "duty", &duty);
            sendFanCommand(1, (uint8_t)duty);
        }

    } else if (strcmp(cmd, "status") == 0) {
        sendStatusRequest();

    } else {
        Serial.printf("{\"error\":\"unknown cmd: %s\"}\n", cmd);
    }
}

// ============================================================
// Command Transmission
// ============================================================
void sendDeviceCommand(uint8_t deviceId, uint8_t state) {
    uint8_t buf[sizeof(PacketHeader) + sizeof(CmdSetDevicePayload) + 1];
    PacketHeader hdr = { MSG_CMD_SET_DEVICE, seqNum++ };
    CmdSetDevicePayload cmd = { deviceId, state };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &cmd, sizeof(cmd));
    size_t payloadLen = sizeof(hdr) + sizeof(cmd);
    buf[payloadLen] = crc8(buf, payloadLen);

    radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    Serial.printf("{\"sent\":\"device\",\"id\":%u,\"state\":%u}\n", deviceId, state);
}

void sendFanCommand(uint8_t mode, uint8_t duty) {
    uint8_t buf[sizeof(PacketHeader) + sizeof(CmdSetFanPayload) + 1];
    PacketHeader hdr = { MSG_CMD_SET_FAN, seqNum++ };
    CmdSetFanPayload cmd = { mode, duty };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &cmd, sizeof(cmd));
    size_t payloadLen = sizeof(hdr) + sizeof(cmd);
    buf[payloadLen] = crc8(buf, payloadLen);

    radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    Serial.printf("{\"sent\":\"fan\",\"mode\":%u,\"duty\":%u}\n", mode, duty);
}

void sendStatusRequest() {
    uint8_t buf[sizeof(PacketHeader) + 1];
    PacketHeader hdr = { MSG_CMD_REQ_STATUS, seqNum++ };

    memcpy(buf, &hdr, sizeof(hdr));
    size_t payloadLen = sizeof(hdr);
    buf[payloadLen] = crc8(buf, payloadLen);

    radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    Serial.println(F("{\"sent\":\"status_request\"}"));
}
