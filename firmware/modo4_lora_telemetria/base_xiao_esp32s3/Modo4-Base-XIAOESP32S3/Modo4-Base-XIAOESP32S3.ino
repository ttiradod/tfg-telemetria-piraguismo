/*
  TFG - Teresa Tirado Dotor. Revisión 2026-10-18.
  Serial.println("TFG Teresa Tirado - Base del modo 4");
  XIAO ESP32-S3 + Wio-SX1262: pasarela USB/LoRa, sin receptor GNSS.
  Mantener esta base emparejada con su interfaz Python del mismo modo.
  BASE_RX incluye RSSI/SNR y contador de recepción antes de la trama T.
  La recepción de un ACK no demuestra persistencia física en la SD de la piragua.
*/
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#define LORA_NSS 41
#define LORA_DIO1 39
#define LORA_RST 42
#define LORA_BUSY 40
#define LORA_SCK 7
#define LORA_MISO 8
#define LORA_MOSI 9
#define LORA_ANT_SW 38
constexpr uint32_t USB_BAUD = 115200;
constexpr size_t MAX_FRAME = 64;
constexpr float LORA_FREQUENCY_MHZ = 869.525;
constexpr float LORA_BANDWIDTH_KHZ = 125.0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;
constexpr uint8_t LORA_SYNC_WORD = 0x12;
constexpr int8_t LORA_OUTPUT_POWER_DBM = 14;
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
// Ventana posterior a telemetría para reducir colisiones; respaldo si no llega.
constexpr uint32_t CMD_GUARD_AFTER_TEL_MS = 40;
constexpr uint32_t CMD_SAFE_WINDOW_MS = 500;
constexpr uint32_t CMD_FALLBACK_MS = 2500;
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
volatile bool packetReceived = false;
String usbFrame;
String pendingCommand;
bool discardingUsbLine = false;
bool commandPending = false;
bool haveTelemetry = false;
uint32_t commandQueuedAtMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t loraTxOk = 0;
uint32_t loraTxErrors = 0;
uint32_t loraRxOk = 0;
uint32_t loraRxCrcErrors = 0;
void IRAM_ATTR onPacketReceived() { packetReceived = true; }
bool restartReceive() {
  packetReceived = false;
  const int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR_BASE;START_RECEIVE;%d\n", state);
    return false;
  }
  return true;
}
bool isCommandFrame(const String &frame) {
  return frame.startsWith("T;C;") && frame.length() <= MAX_FRAME;
}
void transmitCommand(const String &frame, const char *reason) {
  packetReceived = false;
  String mutableFrame = frame;
  const uint32_t airtimeUs = static_cast<uint32_t>(radio.getTimeOnAir(frame.length()));
  const int state = radio.transmit(mutableFrame);
  packetReceived = false;
  if (state == RADIOLIB_ERR_NONE) {
    loraTxOk++;
    Serial.printf("BASE_TX_OK;count=%lu;bytes=%u;airtime_us=%lu;reason=%s\n",
                  static_cast<unsigned long>(loraTxOk), static_cast<unsigned int>(frame.length()),
                  static_cast<unsigned long>(airtimeUs), reason);
  } else {
    loraTxErrors++;
    Serial.printf("ERROR_BASE;TX_LORA;code=%d;count=%lu\n", state,
                  static_cast<unsigned long>(loraTxErrors));
  }
  restartReceive();
}
void queueCommand(const String &frame) {
  if (!isCommandFrame(frame)) {
    Serial.println("ERROR_BASE;INVALID_COMMAND");
    return;
  }
  if (commandPending) {
    Serial.println("ERROR_BASE;COMMAND_ALREADY_PENDING");
    return;
  }
  pendingCommand = frame;
  commandPending = true;
  commandQueuedAtMs = millis();
  Serial.print("BASE_CMD_QUEUED;");
  Serial.println(frame);
}
void pollUsbInput() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (discardingUsbLine) {
        discardingUsbLine = false;
        usbFrame = "";
        Serial.println("ERROR_BASE;FRAME_TOO_LONG");
      } else if (usbFrame.length() > 0) {
        queueCommand(usbFrame);
        usbFrame = "";
      }
      continue;
    }
    if (discardingUsbLine) continue;
    if (usbFrame.length() >= MAX_FRAME) {
      discardingUsbLine = true;
      usbFrame = "";
      continue;
    }
    usbFrame += c;
  }
}
void pollLoRaInput() {
  if (!packetReceived) return;
  packetReceived = false;
  String frame;
  const int state = radio.readData(frame);
  if (state == RADIOLIB_ERR_NONE) {
    if (frame.length() == 0 || frame.length() > MAX_FRAME) {
      Serial.println("ERROR_BASE;INVALID_RX_LENGTH");
      restartReceive();
      return;
    }
    loraRxOk++;
    const float rssi = radio.getRSSI();
    const float snr = radio.getSNR();
    if (frame.startsWith("T;D;")) {
      haveTelemetry = true;
      lastTelemetryMs = millis();
    }
    Serial.printf("BASE_RX;%.1f;%.1f;%lu;%s\n", rssi, snr, static_cast<unsigned long>(loraRxOk),
                  frame.c_str());
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    loraRxCrcErrors++;
    Serial.printf("ERROR_BASE;LORA_CRC;count=%lu\n", static_cast<unsigned long>(loraRxCrcErrors));
  } else {
    Serial.printf("ERROR_BASE;READ_LORA;code=%d\n", state);
  }
  restartReceive();
}
void sendPendingCommandIfSafe() {
  if (!commandPending) return;
  const uint32_t now = millis();
  bool sendNow = false;
  const char *reason = "";
  if (haveTelemetry) {
    const uint32_t sinceTelemetry = now - lastTelemetryMs;
    if (sinceTelemetry >= CMD_GUARD_AFTER_TEL_MS && sinceTelemetry <= CMD_SAFE_WINDOW_MS) {
      sendNow = true;
      reason = "AFTER_TELEMETRY";
    }
  }
  if (!sendNow && now - commandQueuedAtMs >= CMD_FALLBACK_MS) {
    sendNow = true;
    reason = "FALLBACK";
  }
  if (!sendNow) return;
  const String frame = pendingCommand;
  pendingCommand = "";
  commandPending = false;
  transmitCommand(frame, reason);
}
void setup() {
  Serial.begin(USB_BAUD);
  delay(2000);
  Serial.println();
  Serial.println("============================================");
  Serial.println("TFG Teresa Tirado - Base del modo 4");
  Serial.println(" 869.525 MHz | BW125 | SF7 | CR4/5");
  Serial.println("============================================");
  usbFrame.reserve(MAX_FRAME + 1);
  pendingCommand.reserve(MAX_FRAME + 1);
  pinMode(LORA_ANT_SW, OUTPUT);
  digitalWrite(LORA_ANT_SW, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.print("Inicializando SX1262... ");
  int state =
      radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                  LORA_SYNC_WORD, LORA_OUTPUT_POWER_DBM, LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE || radio.explicitHeader() != RADIOLIB_ERR_NONE ||
      radio.setCRC(2) != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR_BASE;RADIO_INIT;%d\n", state);
    while (true) delay(1000);
  }
  Serial.println("OK");
  if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE) {
    Serial.println("ERROR_BASE;RF_SWITCH");
    while (true) delay(1000);
  }
  radio.setPacketReceivedAction(onPacketReceived);
  if (!restartReceive()) {
    while (true) delay(1000);
  }
  Serial.println("Base preparada. Esperando telemetria y comandos USB.");
}
void loop() {
  pollLoRaInput();
  pollUsbInput();
  sendPendingCommandIfSafe();
  delay(1);
}
