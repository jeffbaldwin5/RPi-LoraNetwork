#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>

#include <RadioLib.h>
#include <hal/RPi/PiHal.h>

#include "../shared/protocol.h"

// --- Zebra HAT pin mapping (BCM) ---
#define RPI_SPI_CHANNEL  0
#define RPI_SPI_DEVICE   0
#define RPI_CS_GPIO      24
#define RPI_DIO1_GPIO    22
#define RPI_RST_GPIO     17
#define RPI_BUSY_GPIO    27
#define RPI_TX_POWER     0       // dBm (Zebra HAT, low power for close range)
#define RPI_PREAMBLE_LEN 17      // Zebra HAT preamble

// --- HAL & Radio ---
// PiHal(spiChannel, spiSpeed, spiDevice, gpioDevice)
PiHal* hal = new PiHal(RPI_SPI_CHANNEL, 2000000, RPI_SPI_DEVICE);
SX1262 radio = new Module(hal, RPI_CS_GPIO, RPI_DIO1_GPIO, RPI_RST_GPIO, RPI_BUSY_GPIO);

// --- State ---
static uint8_t seqNum = 0;
static volatile bool rxFlag = false;

// --- stdin input buffer ---
static char stdinBuf[256];
static size_t stdinBufPos = 0;

// --- Forward declarations ---
void initLoRa();
void handleLoRaRx();
void handleStdinInput();
void processCommand(const char* json);
void sendDeviceCommand(uint8_t deviceId, uint8_t state);
void sendFanCommand(uint8_t mode, uint8_t duty);
void sendStatusRequest();
void sendRtcCommand(uint32_t epoch);

// --- DIO1 ISR callback ---
static void dio1ISR(void) {
    rxFlag = true;
}

// --- Minimal JSON helpers (same as RP2350 base station) ---
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

// ============================================================
// LoRa
// ============================================================
void initLoRa() {
    int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING,
                            LORA_CODING_RATE, LORA_SYNC_WORD, RPI_TX_POWER);
    if (state != RADIOLIB_ERR_NONE) {
        fprintf(stderr, "{\"error\":\"LoRa init failed, code %d\"}\n", state);
        exit(1);
    }

    // Enable DIO3 TCXO on Zebra HAT
    state = radio.setTCXO(1.6);
    if (state != RADIOLIB_ERR_NONE) {
        fprintf(stderr, "{\"error\":\"TCXO setup failed, code %d\"}\n", state);
        exit(1);
    }

    // Set preamble length for Zebra HAT
    state = radio.setPreambleLength(RPI_PREAMBLE_LEN);
    if (state != RADIOLIB_ERR_NONE) {
        fprintf(stderr, "{\"error\":\"setPreambleLength failed, code %d\"}\n", state);
        exit(1);
    }

    radio.setDio1Action(dio1ISR);
    radio.startReceive();
    printf("{\"status\":\"LoRa initialized\"}\n");
    fflush(stdout);
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
        printf("{\"error\":\"CRC mismatch\"}\n");
        fflush(stdout);
        return;
    }

    PacketHeader hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    switch (hdr.msgType) {
        case MSG_TELEMETRY: {
            if (len - 1 < sizeof(PacketHeader) + sizeof(TelemetryPayload)) break;
            TelemetryPayload tel;
            memcpy(&tel, buf + sizeof(PacketHeader), sizeof(tel));

            printf("{\"voltage_V\":%.3f,\"current_mA\":%d,\"temp_F\":%.1f,"
                   "\"fan_pct\":%u,\"devices\":[%d,%d,%d],"
                   "\"rssi\":%.1f,\"snr\":%.1f}\n",
                   tel.voltage_mV / 1000.0f,
                   tel.current_mA,
                   tel.temp_C_x10 / 10.0f * 9.0f / 5.0f + 32.0f,
                   tel.fan_duty_pct,
                   (tel.device_states >> 0) & 1,
                   (tel.device_states >> 1) & 1,
                   (tel.device_states >> 2) & 1,
                   rssi, snr);
            fflush(stdout);
            break;
        }
        case MSG_TELEMETRY_EXT: {
            if (len - 1 < sizeof(PacketHeader) + sizeof(TelemetryExtPayload)) break;
            TelemetryExtPayload tel;
            memcpy(&tel, buf + sizeof(PacketHeader), sizeof(tel));

            printf("{\"voltage_V\":%.3f,\"current_mA\":%d,\"temp_F\":%.1f,"
                   "\"fan_pct\":%u,\"devices\":[%d,%d,%d],"
                   "\"humidity\":%.1f,\"sht_temp_F\":%.1f,"
                   "\"lux\":%u,\"vbat_mV\":%u,\"rtc_epoch\":%lu,"
                   "\"rssi\":%.1f,\"snr\":%.1f}\n",
                   tel.voltage_mV / 1000.0f,
                   tel.current_mA,
                   tel.temp_C_x10 / 10.0f * 9.0f / 5.0f + 32.0f,
                   tel.fan_duty_pct,
                   (tel.device_states >> 0) & 1,
                   (tel.device_states >> 1) & 1,
                   (tel.device_states >> 2) & 1,
                   tel.humidity_x10 / 10.0f,
                   tel.sht_temp_C_x10 / 10.0f * 9.0f / 5.0f + 32.0f,
                   tel.lux,
                   tel.vbat_mV,
                   (unsigned long)tel.rtc_epoch,
                   rssi, snr);
            fflush(stdout);
            break;
        }
        case MSG_CMD_ACK: {
            if (len - 1 < sizeof(PacketHeader) + sizeof(CmdAckPayload)) break;
            CmdAckPayload ack;
            memcpy(&ack, buf + sizeof(PacketHeader), sizeof(ack));
            printf("{\"ack\":{\"msg_type\":%u,\"seq\":%u,\"status\":%u}}\n",
                   ack.ackedMsgType, ack.ackedSeqNum, ack.status);
            fflush(stdout);
            break;
        }
        default:
            break;
    }
}

// ============================================================
// stdin Command Parsing
// ============================================================
void handleStdinInput() {
    // Non-blocking read from stdin
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return;

    if (c == '\n' || c == '\r') {
        if (stdinBufPos > 0) {
            stdinBuf[stdinBufPos] = '\0';
            processCommand(stdinBuf);
            stdinBufPos = 0;
        }
    } else if (stdinBufPos < sizeof(stdinBuf) - 1) {
        stdinBuf[stdinBufPos++] = c;
    }
}

void processCommand(const char* json) {
    char cmd[32];
    if (!findJsonString(json, "cmd", cmd, sizeof(cmd))) {
        printf("{\"error\":\"missing cmd field\"}\n");
        fflush(stdout);
        return;
    }

    if (strcmp(cmd, "device") == 0) {
        int id = 0;
        char state[16];
        if (!findJsonInt(json, "id", &id) || !findJsonString(json, "state", state, sizeof(state))) {
            printf("{\"error\":\"device cmd requires id and state\"}\n");
            fflush(stdout);
            return;
        }
        uint8_t stateVal = 0;
        if (strcmp(state, "on") == 0) stateVal = 1;
        else if (strcmp(state, "reset") == 0) stateVal = 2;
        sendDeviceCommand((uint8_t)id, stateVal);

    } else if (strcmp(cmd, "fan") == 0) {
        char mode[16];
        if (!findJsonString(json, "mode", mode, sizeof(mode))) {
            printf("{\"error\":\"fan cmd requires mode\"}\n");
            fflush(stdout);
            return;
        }
        if (strcmp(mode, "auto") == 0) {
            sendFanCommand(0, 0);
        } else if (strcmp(mode, "manual") == 0) {
            int duty = 0;
            findJsonInt(json, "duty", &duty);
            sendFanCommand(1, (uint8_t)duty);
        }

    } else if (strcmp(cmd, "rtc") == 0) {
        int epoch = 0;
        if (!findJsonInt(json, "epoch", &epoch)) {
            printf("{\"error\":\"rtc cmd requires epoch\"}\n");
            fflush(stdout);
            return;
        }
        sendRtcCommand((uint32_t)epoch);

    } else if (strcmp(cmd, "status") == 0) {
        sendStatusRequest();

    } else {
        printf("{\"error\":\"unknown cmd: %s\"}\n", cmd);
        fflush(stdout);
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

    printf("{\"sent\":\"device\",\"id\":%u,\"state\":%u}\n", deviceId, state);
    fflush(stdout);
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

    printf("{\"sent\":\"fan\",\"mode\":%u,\"duty\":%u}\n", mode, duty);
    fflush(stdout);
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

    printf("{\"sent\":\"status_request\"}\n");
    fflush(stdout);
}

void sendRtcCommand(uint32_t epoch) {
    uint8_t buf[sizeof(PacketHeader) + sizeof(CmdSetRtcPayload) + 1];
    PacketHeader hdr = { MSG_CMD_SET_RTC, seqNum++ };
    CmdSetRtcPayload cmd = { epoch };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &cmd, sizeof(cmd));
    size_t payloadLen = sizeof(hdr) + sizeof(cmd);
    buf[payloadLen] = crc8(buf, payloadLen);

    radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    printf("{\"sent\":\"rtc\",\"epoch\":%lu}\n", (unsigned long)epoch);
    fflush(stdout);
}

// ============================================================
// Main
// ============================================================
int main() {
    printf("{\"status\":\"Base station starting...\"}\n");
    fflush(stdout);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    initLoRa();

    printf("{\"status\":\"Base station ready\"}\n");
    fflush(stdout);

    while (true) {
        handleLoRaRx();
        handleStdinInput();
        usleep(1000);  // 1ms sleep to avoid busy-spinning
    }

    return 0;
}
