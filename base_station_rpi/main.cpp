#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

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
#define RPI_TX_POWER     18      // dBm (Zebra HAT 1W, matches pymc config)
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
    // Zebra HAT: pass TCXO voltage (1.6V) directly to begin() so modSetup()
    // enables the TCXO before calibration.  begin() signature:
    //   begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage)
    int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING,
                            LORA_CODING_RATE, LORA_SYNC_WORD, RPI_TX_POWER,
                            RPI_PREAMBLE_LEN, 1.6);
    if (state != RADIOLIB_ERR_NONE) {
        fprintf(stderr, "{\"error\":\"LoRa init failed, code %d\"}\n", state);
        exit(1);
    }

    // Override PA config: use datasheet defaults (paDutyCycle=4, hpMax=7)
    // instead of RadioLib's optimized table — required for Zebra HAT external PA
    state = radio.setOutputPower(RPI_TX_POWER, false);
    if (state != RADIOLIB_ERR_NONE) {
        fprintf(stderr, "{\"error\":\"setOutputPower failed, code %d\"}\n", state);
    }

    // Read back key registers to verify radio config
    uint8_t syncMSB = radio.mod->SPIreadRegister(0x0740);
    uint8_t syncLSB = radio.mod->SPIreadRegister(0x0741);
    // Frequency is stored as 32-bit value at 0x088B
    uint8_t freq3 = radio.mod->SPIreadRegister(0x088B);
    uint8_t freq2 = radio.mod->SPIreadRegister(0x088C);
    uint8_t freq1 = radio.mod->SPIreadRegister(0x088D);
    uint8_t freq0 = radio.mod->SPIreadRegister(0x088E);
    uint32_t freqReg = ((uint32_t)freq3 << 24) | ((uint32_t)freq2 << 16) |
                       ((uint32_t)freq1 << 8) | freq0;
    double actualFreq = (double)freqReg * 32.0 / (1 << 25);

    printf("{\"init_ok\":true,\"freq_MHz\":%.4f,\"freq_reg\":\"0x%08X\","
           "\"sync_reg\":\"0x%02X%02X\","
           "\"bw_kHz\":%.1f,\"sf\":%d,\"cr\":%d,"
           "\"sync\":\"0x%02X\",\"pwr_dBm\":%d,\"preamble\":%d,\"tcxo_V\":%.1f}\n",
           actualFreq, freqReg, syncMSB, syncLSB,
           LORA_BANDWIDTH, LORA_SPREADING, LORA_CODING_RATE,
           LORA_SYNC_WORD, RPI_TX_POWER, RPI_PREAMBLE_LEN, 1.6);
    fflush(stdout);

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
static void hexDump(const char* label, const uint8_t* data, size_t len) {
    printf("{\"hex_dump\":\"%s\",\"len\":%zu,\"bytes\":\"", label, len);
    for (size_t i = 0; i < len; i++) {
        printf("%02X", data[i]);
    }
    printf("\"}\n");
    fflush(stdout);
}

void sendDeviceCommand(uint8_t deviceId, uint8_t state) {
    uint8_t buf[sizeof(PacketHeader) + sizeof(CmdSetDevicePayload) + 1];
    PacketHeader hdr = { MSG_CMD_SET_DEVICE, seqNum++ };
    CmdSetDevicePayload cmd = { deviceId, state };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &cmd, sizeof(cmd));
    size_t payloadLen = sizeof(hdr) + sizeof(cmd);
    buf[payloadLen] = crc8(buf, payloadLen);

    hexDump("tx_device", buf, payloadLen + 1);
    int txState = radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    printf("{\"sent\":\"device\",\"id\":%u,\"state\":%u,\"tx_code\":%d}\n", deviceId, state, txState);
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

    hexDump("tx_fan", buf, payloadLen + 1);
    int txState = radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    printf("{\"sent\":\"fan\",\"mode\":%u,\"duty\":%u,\"tx_code\":%d}\n", mode, duty, txState);
    fflush(stdout);
}

void sendStatusRequest() {
    uint8_t buf[sizeof(PacketHeader) + 1];
    PacketHeader hdr = { MSG_CMD_REQ_STATUS, seqNum++ };

    memcpy(buf, &hdr, sizeof(hdr));
    size_t payloadLen = sizeof(hdr);
    buf[payloadLen] = crc8(buf, payloadLen);

    hexDump("tx_status", buf, payloadLen + 1);

    // --- Pre-TX register dump ---
    uint8_t syncMSB = radio.mod->SPIreadRegister(0x0740);
    uint8_t syncLSB = radio.mod->SPIreadRegister(0x0741);
    uint8_t freq3 = radio.mod->SPIreadRegister(0x088B);
    uint8_t freq2 = radio.mod->SPIreadRegister(0x088C);
    uint8_t freq1 = radio.mod->SPIreadRegister(0x088D);
    uint8_t freq0 = radio.mod->SPIreadRegister(0x088E);
    uint32_t freqReg = ((uint32_t)freq3 << 24) | ((uint32_t)freq2 << 16) |
                       ((uint32_t)freq1 << 8) | freq0;
    double actualFreq = (double)freqReg * 32.0 / (1 << 25);
    printf("{\"pre_tx_regs\":{\"sync\":\"0x%02X%02X\",\"freq_MHz\":%.4f}}\n",
           syncMSB, syncLSB, actualFreq);
    fflush(stdout);

    // --- Manual transmit with buffer readback ---
    // Step 1: standby
    radio.standby();

    // Step 2: write buffer via RadioLib internals (GODMODE)
    // Set packet params (length)
    radio.setPacketParams(RPI_PREAMBLE_LEN, RADIOLIB_SX126X_LORA_CRC_ON,
                          payloadLen + 1, RADIOLIB_SX126X_LORA_HEADER_EXPLICIT, false);

    // Set DIO IRQ for TX_DONE on DIO1
    radio.setDioIrqParams(RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT,
                          RADIOLIB_SX126X_IRQ_TX_DONE);

    // Set buffer base address
    radio.setBufferBaseAddress();

    // Write data to radio buffer
    radio.writeBuffer(buf, payloadLen + 1);

    // Step 3: Read back buffer to verify
    uint8_t readback[MAX_PACKET_SIZE] = {0};
    radio.readBuffer(readback, payloadLen + 1);
    hexDump("tx_readback", readback, payloadLen + 1);

    // Verify match
    bool match = (memcmp(buf, readback, payloadLen + 1) == 0);
    printf("{\"buffer_match\":%s}\n", match ? "true" : "false");
    fflush(stdout);

    // Step 4: clear IRQ and transmit
    radio.clearIrqStatus();

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    // Set TX mode
    radio.setTx(RADIOLIB_SX126X_TX_TIMEOUT_NONE);

    // Wait for TX_DONE (poll IRQ flags)
    uint32_t start = hal->millis();
    uint32_t irq = 0;
    while (!(irq & RADIOLIB_SX126X_IRQ_TX_DONE)) {
        irq = radio.getIrqFlags();
        if (hal->millis() - start > 10000) {
            printf("{\"error\":\"TX timeout after 10s\",\"irq\":\"0x%08X\"}\n", irq);
            fflush(stdout);
            break;
        }
        hal->delay(1);
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    long tx_ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;

    // Read IRQ status after TX
    uint32_t finalIrq = radio.getIrqFlags();
    radio.clearIrqStatus();

    rxFlag = false;
    radio.startReceive();

    printf("{\"sent\":\"status_request\",\"tx_done\":%s,\"irq\":\"0x%08X\",\"tx_time_ms\":%ld}\n",
           (finalIrq & RADIOLIB_SX126X_IRQ_TX_DONE) ? "true" : "false", finalIrq, tx_ms);
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

    hexDump("tx_rtc", buf, payloadLen + 1);
    int txState = radio.transmit(buf, payloadLen + 1);
    rxFlag = false;
    radio.startReceive();

    printf("{\"sent\":\"rtc\",\"epoch\":%lu,\"tx_code\":%d}\n", (unsigned long)epoch, txState);
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
