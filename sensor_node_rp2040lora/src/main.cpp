#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <INA226_WE.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT4x.h>
#include <BH1750.h>
#include <pcf8563.h>
#include "protocol.h"

// ============================================================
// Waveshare RP2040-LoRa SX1262 pin mapping
// ============================================================
#define WS_LORA_MOSI   15
#define WS_LORA_MISO   24
#define WS_LORA_SCK    14
#define WS_LORA_CS     13
#define WS_LORA_RST    23
#define WS_LORA_BUSY   18
#define WS_LORA_DIO1   16
#define WS_LORA_ANT_SW 17  // Antenna switch: LOW=TX, HIGH=RX/idle

// --- Peripheral Pin Definitions ---
// These use the remaining GPIOs on the RP2040-LoRa board.
// Adjust as needed for your wiring.
#define PIN_SDA        4
#define PIN_SCL        5
#define PIN_ONEWIRE    6
#define PIN_FAN_PWM    7
#define PIN_DEV0       8
#define PIN_DEV1       9
#define PIN_DEV2      11
#define PIN_SENSOR_EN 10
#define PIN_VBAT_ADC   1

#define NUM_DEVICES    3
static const uint8_t devicePins[NUM_DEVICES] = { PIN_DEV0, PIN_DEV1, PIN_DEV2 };

// --- Timing ---
#define SENSOR_INTERVAL_MS    2000
#define TELEMETRY_INTERVAL_MS 120000
#define RESET_DELAY_MS        3000

// --- LoRa (SPI1) ---
// RP2040-LoRa uses SPI1 for the onboard SX1262
SX1262 radio = new Module(WS_LORA_CS, WS_LORA_DIO1, WS_LORA_RST, WS_LORA_BUSY, SPI1);

// --- Sensors ---
INA226_WE ina226 = INA226_WE(0x40);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature tempSensor(&oneWire);
Adafruit_SHT4x sht40;
BH1750 bh1750;
PCF8563_Class rtc;

// --- Sensor presence flags ---
static bool hasINA226 = false;
static bool hasDS18B20 = false;
static bool hasSHT40 = false;
static bool hasBH1750 = false;
static bool hasPCF8563 = false;

// --- State ---
static volatile bool rxFlag = false;
static uint8_t seqNum = 0;
static uint16_t voltage_mV = 0;
static int16_t  current_mA = 0;
static int16_t  temp_C_x10 = 250;  // default 25.0C
static uint8_t  fan_duty_pct = 0;
static uint16_t humidity_x10 = 0;
static int16_t  sht_temp_C_x10 = 0;
static uint16_t lux = 0;
static uint16_t vbat_mV = 0;
static bool     fan_auto = true;
static uint8_t  manual_fan_duty = 0;
static bool     device_on[NUM_DEVICES] = { false, false, false };

// Reset cycle state per device
static bool     resetActive[NUM_DEVICES] = { false, false, false };
static uint32_t resetOffTime[NUM_DEVICES] = { 0, 0, 0 };

static uint32_t lastSensorRead = 0;
static uint32_t lastTelemetry = 0;
static bool     tempRequested = false;

// --- Forward declarations ---
void initLoRa();
void initSensors();
void initFan();
void initDeviceGPIO();
void readSensors();
void updateFan();
void sendTelemetryExt();
void handleLoRaRx();
void executeCommand(const uint8_t* packet, size_t len);
void setDevice(uint8_t id, bool on);
void startResetCycle(uint8_t id);
void processResetCycles();
void sendAck(uint8_t ackedType, uint8_t ackedSeq, uint8_t status);
uint8_t buildDeviceStateBitmask();
void setAntennaTx();
void setAntennaRx();

// ============================================================
// Antenna switch helpers
// ============================================================
void setAntennaTx() {
    digitalWrite(WS_LORA_ANT_SW, LOW);
}

void setAntennaRx() {
    digitalWrite(WS_LORA_ANT_SW, HIGH);
}

// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("RP2040-LoRa Sensor Node starting..."));

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialize antenna switch to RX mode
    pinMode(WS_LORA_ANT_SW, OUTPUT);
    setAntennaRx();

    initDeviceGPIO();
    initSensors();
    initFan();
    initLoRa();

    // Send initial telemetry immediately
    Serial.println(F("Sensor Online"));
    readSensors();
    updateFan();
    sendTelemetryExt();
    lastTelemetry = millis();

    digitalWrite(PIN_LED, LOW);
}

void loop() {
    uint32_t now = millis();

    // Sensor reads every 2s
    if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
        lastSensorRead = now;
        readSensors();
        updateFan();
    }

    // Telemetry every 2 minutes
    if (now - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
        lastTelemetry = now;
        sendTelemetryExt();
    }

    // Check for incoming LoRa commands
    handleLoRaRx();

    // Process any active reset cycles
    processResetCycles();
}

// ============================================================
// LoRa
// ============================================================
void initLoRa() {
    Serial.print(F("Initializing LoRa... "));
    SPI1.setRX(WS_LORA_MISO);
    SPI1.setTX(WS_LORA_MOSI);
    SPI1.setSCK(WS_LORA_SCK);
    SPI1.begin();

    int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING,
                            LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("FAILED, code "));
        Serial.println(state);
        while (true) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            delay(200);
        }
    }

    radio.setPreambleLength(LORA_PREAMBLE_LEN);

    // Configure for receive
    radio.setDio1Action([]() { rxFlag = true; });
    rxFlag = false;
    setAntennaRx();
    radio.startReceive();
    Serial.println(F("OK"));
}

void handleLoRaRx() {
    if (!rxFlag) return;
    rxFlag = false;

    size_t len = radio.getPacketLength();
    if (len == 0) return;

    uint8_t buf[MAX_PACKET_SIZE];
    int state = radio.readData(buf, len);
    setAntennaRx();
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE || len < 3) return;

    // Verify CRC
    uint8_t rxCrc = buf[len - 1];
    uint8_t calcCrc = crc8(buf, len - 1);
    if (rxCrc != calcCrc) {
        Serial.println(F("CRC error on received packet"));
        return;
    }

    executeCommand(buf, len - 1);
}

void executeCommand(const uint8_t* packet, size_t len) {
    PacketHeader hdr;
    memcpy(&hdr, packet, sizeof(hdr));

    switch (hdr.msgType) {
        case MSG_CMD_SET_DEVICE: {
            if (len < sizeof(PacketHeader) + sizeof(CmdSetDevicePayload)) break;
            CmdSetDevicePayload cmd;
            memcpy(&cmd, packet + sizeof(PacketHeader), sizeof(cmd));
            if (cmd.device_id >= NUM_DEVICES) {
                sendAck(hdr.msgType, hdr.seqNum, 1);
                break;
            }
            if (cmd.state == 2) {
                startResetCycle(cmd.device_id);
            } else {
                setDevice(cmd.device_id, cmd.state == 1);
            }
            sendAck(hdr.msgType, hdr.seqNum, 0);
            Serial.printf("CMD: device %d -> %d\n", cmd.device_id, cmd.state);
            break;
        }
        case MSG_CMD_SET_FAN: {
            if (len < sizeof(PacketHeader) + sizeof(CmdSetFanPayload)) break;
            CmdSetFanPayload cmd;
            memcpy(&cmd, packet + sizeof(PacketHeader), sizeof(cmd));
            if (cmd.mode == 0) {
                fan_auto = true;
                Serial.println(F("CMD: fan -> auto"));
            } else {
                fan_auto = false;
                manual_fan_duty = cmd.duty > 100 ? 100 : cmd.duty;
                Serial.printf("CMD: fan -> manual %d%%\n", manual_fan_duty);
            }
            sendAck(hdr.msgType, hdr.seqNum, 0);
            break;
        }
        case MSG_CMD_REQ_STATUS: {
            sendTelemetryExt();
            sendAck(hdr.msgType, hdr.seqNum, 0);
            Serial.println(F("CMD: status request"));
            break;
        }
        default:
            break;
    }
}

void sendAck(uint8_t ackedType, uint8_t ackedSeq, uint8_t status) {
    uint8_t buf[sizeof(PacketHeader) + sizeof(CmdAckPayload) + 1];
    PacketHeader hdr = { MSG_CMD_ACK, seqNum++ };
    CmdAckPayload ack = { ackedType, ackedSeq, status };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &ack, sizeof(ack));
    size_t payloadLen = sizeof(hdr) + sizeof(ack);
    buf[payloadLen] = crc8(buf, payloadLen);

    setAntennaTx();
    radio.transmit(buf, payloadLen + 1);
    setAntennaRx();
    rxFlag = false;
    radio.startReceive();
}

void sendTelemetryExt() {
    uint8_t buf[sizeof(PacketHeader) + sizeof(TelemetryExtPayload) + 1];
    PacketHeader hdr = { MSG_TELEMETRY_EXT, seqNum++ };

    uint32_t epoch = 0;
    if (hasPCF8563) {
        RTC_Date now = rtc.getDateTime();
        struct tm t;
        t.tm_year = now.year - 1900;
        t.tm_mon  = now.month - 1;
        t.tm_mday = now.day;
        t.tm_hour = now.hour;
        t.tm_min  = now.minute;
        t.tm_sec  = now.second;
        t.tm_isdst = 0;
        epoch = (uint32_t)mktime(&t);
    }

    TelemetryExtPayload tel = {
        voltage_mV,
        current_mA,
        temp_C_x10,
        fan_duty_pct,
        buildDeviceStateBitmask(),
        humidity_x10,
        sht_temp_C_x10,
        lux,
        vbat_mV,
        epoch
    };

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &tel, sizeof(tel));
    size_t payloadLen = sizeof(hdr) + sizeof(tel);
    buf[payloadLen] = crc8(buf, payloadLen);

    setAntennaTx();
    int state = radio.transmit(buf, payloadLen + 1);
    setAntennaRx();
    rxFlag = false;
    radio.startReceive();

    if (state == RADIOLIB_ERR_NONE) {
        digitalWrite(PIN_LED, HIGH);
        delay(50);
        digitalWrite(PIN_LED, LOW);
    }

    Serial.printf("TX telem: %.3fV %dmA %.1fF fan:%d%% dev:%d%d%d hum:%.1f%% sht:%.1fC lux:%u vbat:%umV\n",
                  voltage_mV / 1000.0f, current_mA, temp_C_x10 / 10.0f * 9.0f / 5.0f + 32.0f,
                  fan_duty_pct,
                  device_on[0], device_on[1], device_on[2],
                  humidity_x10 / 10.0f, sht_temp_C_x10 / 10.0f,
                  lux, vbat_mV);
}

// ============================================================
// Sensors
// ============================================================
void initSensors() {
    // Enable logger hat sensors via GP10
    pinMode(PIN_SENSOR_EN, OUTPUT);
    digitalWrite(PIN_SENSOR_EN, HIGH);
    delay(50);  // let sensors power up

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    // Probe INA226 on I2C before calling init
    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() == 0) {
        hasINA226 = true;
        ina226.init();
        ina226.setResistorRange(0.01, 8.0);  // 10mOhm shunt, 8A max
        ina226.setAverage(INA226_AVERAGE_64);
        ina226.setConversionTime(INA226_CONV_TIME_1100);
        Serial.println(F("INA226: OK"));
    } else {
        Serial.println(F("INA226: not found, skipping"));
    }

    tempSensor.begin();
    if (tempSensor.getDeviceCount() > 0) {
        hasDS18B20 = true;
        tempSensor.setResolution(12);
        tempSensor.setWaitForConversion(false);
        tempSensor.requestTemperatures();
        tempRequested = true;
        Serial.println(F("DS18B20: OK"));
    } else {
        Serial.println(F("DS18B20: not found, skipping"));
    }

    // --- Logger hat sensors ---
    // SHT40 (0x44)
    if (sht40.begin(&Wire)) {
        hasSHT40 = true;
        sht40.setPrecision(SHT4X_HIGH_PRECISION);
        Serial.println(F("SHT40: OK"));
    } else {
        Serial.println(F("SHT40: not found, skipping"));
    }

    // BH1750 (0x23)
    if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
        hasBH1750 = true;
        Serial.println(F("BH1750: OK"));
    } else {
        Serial.println(F("BH1750: not found, skipping"));
    }

    // PCF8563 (0x51)
    Wire.beginTransmission(0x51);
    if (Wire.endTransmission() == 0) {
        rtc.begin(Wire);
        hasPCF8563 = true;
        Serial.println(F("PCF8563: OK"));
    } else {
        Serial.println(F("PCF8563: not found, skipping"));
    }

    // VBAT ADC on GP1 (ADC channel 1, voltage divider VBAT/2)
    analogReadResolution(12);
    pinMode(PIN_VBAT_ADC, INPUT);
}

void readSensors() {
    if (hasINA226) {
        float busV = ina226.getBusVoltage_V();
        float shuntI = ina226.getCurrent_mA();
        voltage_mV = (uint16_t)(busV * 1000.0f);
        current_mA = (int16_t)shuntI;
    }

    if (hasDS18B20) {
        if (tempRequested && tempSensor.isConversionComplete()) {
            float t = tempSensor.getTempCByIndex(0);
            if (t != DEVICE_DISCONNECTED_C) {
                temp_C_x10 = (int16_t)(t * 10.0f);
            }
        }
        tempSensor.requestTemperatures();
        tempRequested = true;
    }

    // --- Logger hat sensors ---
    if (hasSHT40) {
        sensors_event_t humEv, tempEv;
        if (sht40.getEvent(&humEv, &tempEv)) {
            humidity_x10 = (uint16_t)(humEv.relative_humidity * 10.0f);
            sht_temp_C_x10 = (int16_t)(tempEv.temperature * 10.0f);
        }
    }

    if (hasBH1750) {
        float l = bh1750.readLightLevel();
        if (l >= 0) {
            lux = (uint16_t)l;
        }
    }

    // VBAT: GP1 through voltage divider (VBAT/2), 3.3V ref, 12-bit ADC
    uint16_t raw = analogRead(PIN_VBAT_ADC);
    vbat_mV = (uint16_t)((raw * 3300UL * 2) / 4095);
}

// ============================================================
// Fan PWM
// ============================================================
void initFan() {
    pinMode(PIN_FAN_PWM, OUTPUT);
    analogWriteFreq(25000);  // 25kHz for 4-pin fans
    analogWriteRange(255);
    analogWrite(PIN_FAN_PWM, 0);
    Serial.println(F("Fan PWM initialized."));
}

void updateFan() {
    uint8_t duty;
    if (fan_auto) {
        float tempC = temp_C_x10 / 10.0f;
        if (tempC < 30.0f) {
            duty = 0;
        } else if (tempC < 40.0f) {
            duty = 25 + (uint8_t)((tempC - 30.0f) / 10.0f * 50.0f);
        } else if (tempC < 50.0f) {
            duty = 75 + (uint8_t)((tempC - 40.0f) / 10.0f * 25.0f);
        } else {
            duty = 100;
        }
    } else {
        duty = manual_fan_duty;
    }

    fan_duty_pct = duty;
    analogWrite(PIN_FAN_PWM, (uint32_t)duty * 255 / 100);
}

// ============================================================
// Device GPIO Control
// ============================================================
void initDeviceGPIO() {
    for (uint8_t i = 0; i < NUM_DEVICES; i++) {
        pinMode(devicePins[i], OUTPUT);
        digitalWrite(devicePins[i], LOW);
    }
    Serial.println(F("Device GPIOs initialized."));
}

void setDevice(uint8_t id, bool on) {
    if (id >= NUM_DEVICES) return;
    device_on[id] = on;
    resetActive[id] = false;
    digitalWrite(devicePins[id], on ? HIGH : LOW);
}

void startResetCycle(uint8_t id) {
    if (id >= NUM_DEVICES) return;
    device_on[id] = false;
    digitalWrite(devicePins[id], LOW);
    resetActive[id] = true;
    resetOffTime[id] = millis();
    Serial.printf("Reset cycle started for device %d\n", id);
}

void processResetCycles() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_DEVICES; i++) {
        if (resetActive[i] && (now - resetOffTime[i] >= RESET_DELAY_MS)) {
            device_on[i] = true;
            digitalWrite(devicePins[i], HIGH);
            resetActive[i] = false;
            Serial.printf("Reset cycle complete for device %d\n", i);
        }
    }
}

uint8_t buildDeviceStateBitmask() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NUM_DEVICES; i++) {
        if (device_on[i]) mask |= (1 << i);
    }
    return mask;
}
