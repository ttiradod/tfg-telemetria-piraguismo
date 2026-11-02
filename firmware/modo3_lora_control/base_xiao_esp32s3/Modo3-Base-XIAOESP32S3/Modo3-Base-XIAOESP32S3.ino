/*
  TFG - Teresa Tirado Dotor. Revisión 2026-10-18.
  ESTACIÓN BASE DEL MODO 3: Control remoto y consulta de estado mediante LoRa.

  XIAO ESP32-S3 + Wio-SX1262: pasarela USB/LoRa, sin receptor GNSS.
  Mantener esta base emparejada con su interfaz Python del mismo modo.
  BASE_RX incluye RSSI/SNR antes de la trama T.
  La recepción de un ACK no demuestra persistencia física en la SD de la piragua.
*/
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
const int LORA_NSS = 41;
const int LORA_DIO1 = 39;
const int LORA_RST = 42;
const int LORA_BUSY = 40;
const int LORA_SCK = 7;
const int LORA_MISO = 8;
const int LORA_MOSI = 9;
const int LORA_ANT_SW = 38;
const uint32_t USB_BAUD = 115200;
const size_t MAX_FRAME = 64;
const float LORA_FREQUENCY_MHZ = 869.525;
const float LORA_BANDWIDTH_KHZ = 125.0;
const uint8_t LORA_SPREADING_FACTOR = 7;
const uint8_t LORA_CODING_RATE = 5;
const uint8_t LORA_SYNC_WORD = 0x12;
const int8_t LORA_OUTPUT_POWER_DBM = 14;
const uint16_t LORA_PREAMBLE_LENGTH = 8;
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
volatile bool loraPacketReceived = false;
String usbFrame;
bool discardingUsbLine = false;
void IRAM_ATTR onLoraPacketReceived() { loraPacketReceived = true; }
bool restartLoraReceive() {
  loraPacketReceived = false;
  return radio.startReceive() == RADIOLIB_ERR_NONE;
}
void transmitCommand(const String &frame) {
  if (!frame.startsWith("T;C;") || frame.length() > MAX_FRAME) {
    Serial.println("BASE_ERROR;ORDEN_INVALIDA");
    return;
  }
  loraPacketReceived = false;
  String packet = frame;
  const int state = radio.transmit(packet);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("BASE_TX;");
    Serial.println(frame);
  } else {
    Serial.printf("BASE_ERROR;TX;%d\n", state);
  }
  if (!restartLoraReceive()) {
    Serial.println("BASE_ERROR;RECEPCION");
  }
}
void pollUsb() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      if (discardingUsbLine) {
        discardingUsbLine = false;
        usbFrame = "";
        Serial.println("BASE_ERROR;TRAMA_LARGA");
      } else if (usbFrame.length() > 0) {
        transmitCommand(usbFrame);
        usbFrame = "";
      }
      continue;
    }
    if (discardingUsbLine) {
      continue;
    }
    if (usbFrame.length() >= MAX_FRAME) {
      discardingUsbLine = true;
      usbFrame = "";
      continue;
    }
    usbFrame += character;
  }
}
void pollLora() {
  if (!loraPacketReceived) {
    return;
  }
  loraPacketReceived = false;
  String frame;
  const int state = radio.readData(frame);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("BASE_RX;%.1f;%.1f;%s\n", radio.getRSSI(), radio.getSNR(), frame.c_str());
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("BASE_ERROR;CRC");
  } else {
    Serial.printf("BASE_ERROR;RX;%d\n", state);
  }
  restartLoraReceive();
}
void setup() {
  Serial.begin(USB_BAUD);
  delay(2000);
  Serial.println();
  Serial.println("============================================");
  Serial.println("TFG Teresa Tirado - Base del modo 3");
  Serial.println(" 869.525 MHz | BW125 | SF7 | CR4/5 | 14 dBm");
  Serial.println("============================================");
  usbFrame.reserve(MAX_FRAME + 1);
  pinMode(LORA_ANT_SW, OUTPUT);
  digitalWrite(LORA_ANT_SW, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  const int state =
      radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                  LORA_SYNC_WORD, LORA_OUTPUT_POWER_DBM, LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE || radio.explicitHeader() != RADIOLIB_ERR_NONE ||
      radio.setCRC(2) != RADIOLIB_ERR_NONE || radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE) {
    Serial.printf("BASE_ERROR;INICIO_RADIO;%d\n", state);
    while (true) {
      delay(1000);
    }
  }
  radio.setPacketReceivedAction(onLoraPacketReceived);
  if (!restartLoraReceive()) {
    Serial.println("BASE_ERROR;RECEPCION");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("BASE_LISTA");
}
void loop() {
  pollLora();
  pollUsb();
  delay(1);
}
