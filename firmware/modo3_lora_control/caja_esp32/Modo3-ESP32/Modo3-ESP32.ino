/*
  TFG - Teresa Tirado Dotor. Revisión 2026-10-18.
  MODO 3: Control remoto y consulta de estado mediante LoRa.

  Hardware: ESP32-WROOM-32, LSM6DS3, LG290P y microSD. SX1262 por HSPI.
  Núcleo 1: acquisitionTask, prioridad 3, periodo nominal de 20 ms.
  Núcleo 0: storageTask, prioridad 1. Cola de 256 muestras entre ambas.
  loop(): LED y radio LoRa.
  Los cuatro modos comparten CSV, pines y adquisición. Las marcas no detienen la SD.

  LED apagado: sin errores detectados. Parpadeo: error registrado hasta reiniciar.
  Flush cada 50 filas correctas. No hay cierre seguro mediante los botones de marca:
  cortar la alimentación puede perder datos pendientes y afectar al archivo.
  Esta revisión requiere validar las placas y sus CSV antes de cerrar la entrega.
*/
#include <Arduino.h>
#include <atomic>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>
#include <esp_timer.h>
#include <TinyGPSPlus.h>
#include "SparkFunLSM6DS3.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <RadioLib.h>
const int LORA_SCK = 14;
const int LORA_MISO = 32;
const int LORA_MOSI = 13;
const int LORA_NSS = 25;
const int LORA_RST = 26;
const int LORA_RF_SW = 27;
const int LORA_DIO1 = 34;
const int LORA_BUSY = 35;
const int IMU_SDA = 21;
const int IMU_SCL = 22;
const int SD_CS = 5;
const int SD_SCK = 18;
const int SD_MISO = 19;
const int SD_MOSI = 23;
const int GNSS_RX = 16;
const int GNSS_TX = 17;
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const uint32_t GNSS_BAUD = 460800;
const int SAMPLE_PERIOD_MS = 20;
const int QUEUE_LENGTH = 256;
const int FLUSH_EVERY_ROWS = 50;
const uint32_t GNSS_MAX_AGE_MS = 1000;
const float LORA_FREQUENCY_MHZ = 869.525;
const float LORA_BANDWIDTH_KHZ = 125.0;
const uint8_t LORA_SPREADING_FACTOR = 7;
const uint8_t LORA_CODING_RATE = 5;
const uint8_t LORA_SYNC_WORD = 0x12;
const int8_t LORA_OUTPUT_POWER_DBM = 14;
const uint16_t LORA_PREAMBLE_LENGTH = 8;
const uint32_t LORA_TURNAROUND_MS = 80;
const size_t PROTOCOL_MAX_FRAME = 64;
const size_t PROTOCOL_BUFFER_SIZE = PROTOCOL_MAX_FRAME + 2;
const uint32_t PROTOCOL_MAX_SEQUENCE = 9999;
LSM6DS3 imu(I2C_MODE, 0x6B);
HardwareSerial gnssSerial(2);
TinyGPSPlus gps;
SPIClass sdSpi(VSPI);
SPIClass loraSpi(HSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
File csvFile;
// Fila común de 20 columnas: t_us en microsegundos desde el arranque,
// velocidad en m/s, aceleración en g y velocidad angular en grados/s.
// gps_new identifica una velocidad nueva; gps_valid indica posición reciente.
// Modo 1: test_id=0 y eventos vacíos. Modos 2-4: marcas dentro del registro continuo.
struct SampleRecord {
  uint32_t sequence;
  uint64_t timeUs;
  char utcDate[11];
  char utcTime[13];
  uint32_t testId;
  char event[16];
  char eventSource[8];
  uint8_t gpsNew;
  uint8_t gpsValid;
  uint8_t satellites;
  float hdop;
  double latitude;
  double longitude;
  float speed;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
// Copia coherente para web/telemetría. Solo adquisición publica datos GNSS.
struct StatusSnapshot {
  uint32_t testId;
  bool active;
  uint8_t gpsValid;
  uint8_t satellites;
  float hdop;
  double latitude;
  double longitude;
  float speed;
};
StatusSnapshot statusSnapshot = {};
portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
// Solo acquisitionTask modifica estas dos variables.
uint32_t currentTestId = 0;
bool markActive = false;
std::atomic<uint32_t> samplesWritten{0};
struct ControlResult {
  char result;  // S/F/Q aceptada; 1/2/3/4/5 motivo de rechazo.
  bool active;
  uint32_t testId;
};
QueueHandle_t commandQueue = nullptr;
QueueHandle_t commandResultQueue = nullptr;
StatusSnapshot readStatus() {
  portENTER_CRITICAL(&statusMux);
  const StatusSnapshot snapshot = statusSnapshot;
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}
// Se llama solo desde loop(): hay una única orden en curso.
// Espera bloqueando esta tarea, mientras adquisición y SD siguen ejecutándose.
ControlResult requestControl(char order) {
  if (xQueueSend(commandQueue, &order, 0) != pdPASS) {
    const StatusSnapshot snapshot = readStatus();
    return {'3', snapshot.active, snapshot.testId};
  }
  ControlResult result;
  xQueueReceive(commandResultQueue, &result, portMAX_DELAY);
  return result;
}
void publishStatus(const SampleRecord &sample) {
  StatusSnapshot snapshot = {currentTestId, markActive,      sample.gpsValid,  sample.satellites,
                             sample.hdop,   sample.latitude, sample.longitude, sample.speed};
  portENTER_CRITICAL(&statusMux);
  statusSnapshot = snapshot;
  portEXIT_CRITICAL(&statusMux);
}
QueueHandle_t sampleQueue = nullptr;
uint32_t sampleSequence = 0;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
std::atomic<bool> systemError{false};
void signalError(const char *message) {
  if (!systemError.exchange(true)) {
    Serial.print("ERROR: ");
    Serial.println(message);
  }
}
void updateLed() {
  static bool ledState = false;
  static uint32_t previousMs = 0;
  if (systemError) {
    if (millis() - previousMs >= 150) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
      previousMs = millis();
    }
    return;
  }
  digitalWrite(LED_BUILTIN, LOW);
}
void sendGnssCommand(const char *command) {
  uint8_t checksum = 0;
  for (const char *p = command; *p; p++) {
    checksum ^= static_cast<uint8_t>(*p);
  }
  char message[120];
  snprintf(message, sizeof(message), "$%s*%02X\r\n", command, checksum);
  gnssSerial.print(message);
  delay(50);
}
// Se solicita 10 Hz y salida GGA/RMC; no se comprueba el ACK del receptor.
void configureGnss() {
  sendGnssCommand("PQTMCFGFIXRATE,W,100");
  sendGnssCommand("PQTMCFGMSGRATE,W,GGA,1");
  sendGnssCommand("PQTMCFGMSGRATE,W,RMC,1");
}
void readGnss() {
  while (gnssSerial.available() > 0) {
    const char c = static_cast<char>(gnssSerial.read());
    gps.encode(c);
  }
}
void copyGnssDateTime(SampleRecord &sample) {
  sample.utcDate[0] = '\0';
  sample.utcTime[0] = '\0';
  if (!gps.location.isValid() || gps.location.age() >= GNSS_MAX_AGE_MS) return;
  if (gps.date.isValid() && gps.date.age() < GNSS_MAX_AGE_MS) {
    const unsigned year = gps.date.year();
    const unsigned month = gps.date.month();
    const unsigned day = gps.date.day();
    if (year <= 9999 && month >= 1 && month <= 12 && day >= 1 && day <= 31) {
      snprintf(sample.utcDate, sizeof(sample.utcDate), "%04u-%02u-%02u", year, month, day);
    }
  }
  if (gps.time.isValid() && gps.time.age() < GNSS_MAX_AGE_MS) {
    const unsigned hour = gps.time.hour();
    const unsigned minute = gps.time.minute();
    const unsigned second = gps.time.second();
    const unsigned milliseconds = gps.time.centisecond() * 10;
    if (hour <= 23 && minute <= 59 && second <= 60 && milliseconds <= 990) {
      snprintf(sample.utcTime, sizeof(sample.utcTime), "%02u:%02u:%02u.%03u", hour, minute, second,
               milliseconds);
    }
  }
}
// Mantener la caja inmóvil al arrancar: media de 500 lecturas de giroscopio.
void calibrateGyroscope() {
  Serial.println("Caja quieta: calibrando giroscopio...");
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sumX += imu.readFloatGyroX();
    sumY += imu.readFloatGyroY();
    sumZ += imu.readFloatGyroZ();
    readGnss();
    delay(5);
  }
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  Serial.println("Calibracion terminada");
}
// Un archivo nuevo por arranque. No se sobrescriben sesiones anteriores.
bool createCsvFile() {
  char filename[24];
  for (int number = 0; number < 10000; number++) {
    snprintf(filename, sizeof(filename), "/DATA%04d.CSV", number);
    if (SD.exists(filename)) {
      continue;
    }
    csvFile = SD.open(filename, FILE_WRITE);
    if (!csvFile) {
      return false;
    }
    Serial.print("Archivo: ");
    Serial.println(filename);
    csvFile.println(
        "sample_seq,"
        "t_us,"
        "utc_date,"
        "utc_time,"
        "test_id,"
        "event,"
        "event_source,"
        "gps_new,"
        "gps_valid,"
        "satellites,"
        "hdop,"
        "lat_deg,"
        "lon_deg,"
        "speed_2d_mps,"
        "ax_g,"
        "ay_g,"
        "az_g,"
        "gx_dps,"
        "gy_dps,"
        "gz_dps");
    csvFile.flush();
    if (csvFile.getWriteError()) return false;
    return true;
  }
  return false;
}
// El instante local precede a las lecturas IMU. El GNSS es el último disponible;
// no existe disparo físico simultáneo entre sensores ni sincronización PPS.
SampleRecord createSample() {
  SampleRecord sample = {};
  sample.sequence = sampleSequence++;
  sample.timeUs = static_cast<uint64_t>(esp_timer_get_time());
  copyGnssDateTime(sample);
  // isUpdated se consulta ANTES de leer mps(), que consume ese indicador.
  const bool freshSpeed = gps.speed.isValid() && gps.speed.age() < GNSS_MAX_AGE_MS;
  sample.gpsNew = gps.speed.isUpdated() && freshSpeed ? 1 : 0;
  const bool freshPosition = gps.location.isValid() && gps.location.age() < GNSS_MAX_AGE_MS;
  sample.gpsValid = freshPosition ? 1 : 0;
  sample.satellites = gps.satellites.isValid() && gps.satellites.age() < GNSS_MAX_AGE_MS
                          ? gps.satellites.value()
                          : 0;
  sample.hdop = freshPosition && gps.hdop.isValid() && gps.hdop.age() < GNSS_MAX_AGE_MS
                    ? gps.hdop.hdop()
                    : NAN;
  sample.latitude = freshPosition ? gps.location.lat() : NAN;
  sample.longitude = freshPosition ? gps.location.lng() : NAN;
  sample.speed = freshSpeed ? gps.speed.mps() : NAN;
  sample.ax = imu.readFloatAccelX();
  sample.ay = imu.readFloatAccelY();
  sample.az = imu.readFloatAccelZ();
  sample.gx = imu.readFloatGyroX() - gyroOffsetX;
  sample.gy = imu.readFloatGyroY() - gyroOffsetY;
  sample.gz = imu.readFloatGyroZ() - gyroOffsetZ;
  return sample;
}
// El estado del test cambia únicamente si la muestra con su marca entra en cola.
// ACK confirma ese paso; no confirma un flush físico de la tarjeta.
void acquisitionTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    readGnss();
    SampleRecord sample = createSample();
    sample.testId = markActive ? currentTestId : 0;
    char order = '\0';
    const bool hasCommand = xQueueReceive(commandQueue, &order, 0) == pdPASS;
    char result = order;
    if (hasCommand) {
      if (order == 'S') {
        if (markActive)
          result = '1';
        else {
          sample.testId = currentTestId + 1;
          strcpy(sample.event, "MARK_START");
          strcpy(sample.eventSource, "LORA");
        }
      } else if (order == 'F') {
        if (!markActive)
          result = '2';
        else {
          sample.testId = currentTestId;
          strcpy(sample.event, "MARK_END");
          strcpy(sample.eventSource, "LORA");
        }
      } else if (order != 'Q')
        result = '5';
    }
    const bool queued = xQueueSend(sampleQueue, &sample, 0) == pdPASS;
    if (!queued) {
      signalError("La cola de muestras esta llena");
      if (result == 'S' || result == 'F') result = '4';
    } else if (result == 'S') {
      currentTestId = sample.testId;
      markActive = true;
    } else if (result == 'F') {
      markActive = false;
    }
    publishStatus(sample);
    if (hasCommand) {
      const ControlResult response = {result, markActive, currentTestId};
      xQueueSend(commandResultQueue, &response, portMAX_DELAY);
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}
// Una operación por fila. true significa bytes entregados a la librería SD;
// su persistencia se solicita por lotes mediante flush().
bool writeCsvRow(const SampleRecord &sample) {
  char row[384];
  const int length = snprintf(
      row, sizeof(row),
      "%lu,%llu,%s,%s,%lu,%s,%s,%u,%u,%u,%.2f,%.8f,%.8f,%.3f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
      static_cast<unsigned long>(sample.sequence), static_cast<unsigned long long>(sample.timeUs),
      sample.utcDate, sample.utcTime, static_cast<unsigned long>(sample.testId), sample.event,
      sample.eventSource, static_cast<unsigned int>(sample.gpsNew),
      static_cast<unsigned int>(sample.gpsValid), static_cast<unsigned int>(sample.satellites),
      sample.hdop, sample.latitude, sample.longitude, sample.speed, sample.ax, sample.ay, sample.az,
      sample.gx, sample.gy, sample.gz);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(row)) {
    signalError("La fila CSV es demasiado larga");
    return false;
  }
  const size_t written =
      csvFile.write(reinterpret_cast<const uint8_t *>(row), static_cast<size_t>(length));
  if (written != static_cast<size_t>(length) || csvFile.getWriteError()) {
    signalError("Error de escritura en microSD");
    return false;
  }
  return true;
}
// Única tarea que accede al archivo después de setup(). Núcleo 0.
void storageTask(void *parameter) {
  SampleRecord sample;
  int rowsSinceFlush = 0;
  while (true) {
    xQueueReceive(sampleQueue, &sample, portMAX_DELAY);
    if (!writeCsvRow(sample)) continue;
    samplesWritten.fetch_add(1);
    if (++rowsSinceFlush >= FLUSH_EVERY_ROWS) {
      csvFile.flush();
      rowsSinceFlush = 0;
      if (csvFile.getWriteError()) signalError("Error de flush en microSD");
    }
  }
}
volatile bool loraPacketReceived = false;
bool hasLastCommand = false;
uint32_t lastCommandSequence = 0;
char lastCommandFrame[PROTOCOL_BUFFER_SIZE] = "";
char lastCommandResponse[PROTOCOL_BUFFER_SIZE] = "";
void IRAM_ATTR onLoraPacketReceived() { loraPacketReceived = true; }
bool restartLoraReceive() {
  loraPacketReceived = false;
  return radio.startReceive() == RADIOLIB_ERR_NONE;
}
bool parseSequence(const char *text, uint32_t &sequence) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  const unsigned long value = strtoul(text, &end, 10);
  if (*end != '\0' || value == 0 || value > PROTOCOL_MAX_SEQUENCE) {
    return false;
  }
  sequence = static_cast<uint32_t>(value);
  return true;
}
void buildResponse(char type, uint32_t sequence, char result, char *response, size_t capacity) {
  const StatusSnapshot snapshot = readStatus();
  snprintf(response, capacity, "T;%c;%lu;%c;%u;%lu;%lu", type, static_cast<unsigned long>(sequence),
           result, snapshot.active ? 1U : 0U, static_cast<unsigned long>(snapshot.testId),
           static_cast<unsigned long>(samplesWritten.load()));
}
void sendLoraResponse(const char *response) {
  delay(LORA_TURNAROUND_MS);
  loraPacketReceived = false;
  String packet(response);
  const int state = radio.transmit(packet);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("LORA TX: ");
    Serial.println(response);
  } else {
    Serial.printf("Error LoRa TX: %d\n", state);
  }
  if (!restartLoraReceive()) {
    signalError("No se ha podido reactivar la recepción LoRa");
  }
}
// ACK/NACK se genera después de resolver la orden en acquisitionTask.
void executeCommand(uint32_t sequence, char order, char *response, size_t capacity) {
  const ControlResult result = requestControl(order);
  const bool accepted = result.result == 'S' || result.result == 'F' || result.result == 'Q';
  snprintf(response, capacity, "T;%c;%lu;%c;%u;%lu;%lu", accepted ? 'A' : 'N',
           static_cast<unsigned long>(sequence), result.result, result.active ? 1U : 0U,
           static_cast<unsigned long>(result.testId),
           static_cast<unsigned long>(samplesWritten.load()));
}
void processCommandFrame(char *frame) {
  if (frame == nullptr || frame[0] == '\0') {
    return;
  }
  const size_t frameLength = strlen(frame);
  if (frameLength > PROTOCOL_MAX_FRAME) {
    return;
  }
  char originalFrame[PROTOCOL_BUFFER_SIZE];
  strncpy(originalFrame, frame, sizeof(originalFrame) - 1);
  originalFrame[sizeof(originalFrame) - 1] = '\0';
  char *fields[5] = {};
  uint8_t fieldCount = 0;
  char *savePointer = nullptr;
  char *token = strtok_r(frame, ";", &savePointer);
  while (token != nullptr && fieldCount < 5) {
    fields[fieldCount++] = token;
    token = strtok_r(nullptr, ";", &savePointer);
  }
  if (fieldCount != 4 || strcmp(fields[0], "T") != 0 || strcmp(fields[1], "C") != 0 ||
      strlen(fields[3]) != 1) {
    Serial.println("Orden LoRa con formato incorrecto");
    return;
  }
  uint32_t sequence = 0;
  if (!parseSequence(fields[2], sequence)) {
    Serial.println("Secuencia LoRa incorrecta");
    return;
  }
  if (hasLastCommand && sequence == lastCommandSequence) {
    if (strcmp(originalFrame, lastCommandFrame) == 0) {
      Serial.println("Orden repetida: se devuelve la respuesta anterior");
      sendLoraResponse(lastCommandResponse);
    } else {
      char conflictResponse[PROTOCOL_BUFFER_SIZE];
      buildResponse('N', sequence, '5', conflictResponse, sizeof(conflictResponse));
      sendLoraResponse(conflictResponse);
    }
    return;
  }
  char response[PROTOCOL_BUFFER_SIZE];
  executeCommand(sequence, fields[3][0], response, sizeof(response));
  hasLastCommand = true;
  lastCommandSequence = sequence;
  strncpy(lastCommandFrame, originalFrame, sizeof(lastCommandFrame) - 1);
  lastCommandFrame[sizeof(lastCommandFrame) - 1] = '\0';
  strncpy(lastCommandResponse, response, sizeof(lastCommandResponse) - 1);
  lastCommandResponse[sizeof(lastCommandResponse) - 1] = '\0';
  sendLoraResponse(response);
}
void pollLora() {
  if (!loraPacketReceived) {
    return;
  }
  loraPacketReceived = false;
  String packet;
  const int state = radio.readData(packet);
  if (state == RADIOLIB_ERR_NONE) {
    if (packet.length() > 0 && packet.length() <= PROTOCOL_MAX_FRAME) {
      char frame[PROTOCOL_BUFFER_SIZE];
      packet.toCharArray(frame, sizeof(frame));
      Serial.print("LORA RX: ");
      Serial.println(frame);
      processCommandFrame(frame);
      restartLoraReceive();
      return;
    }
  } else if (state != RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.printf("Error LoRa RX: %d\n", state);
  }
  restartLoraReceive();
}
bool initializeLora() {
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  const int state =
      radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                  LORA_SYNC_WORD, LORA_OUTPUT_POWER_DBM, LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("No se ha podido iniciar el SX1262: %d\n", state);
    return false;
  }
  if (radio.explicitHeader() != RADIOLIB_ERR_NONE || radio.setCRC(2) != RADIOLIB_ERR_NONE ||
      radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE) {
    Serial.println("No se ha podido configurar el SX1262");
    return false;
  }
  radio.setRfSwitchPins(LORA_RF_SW, RADIOLIB_NC);
  radio.setPacketReceivedAction(onLoraPacketReceived);
  return restartLoraReceive();
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=======================================");
  Serial.println("TFG Teresa Tirado - Modo 3: control remoto LoRa");
  Serial.println("=======================================");
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  // Ajustes iguales a los valores previos de SparkFun 1.0.3.
  // La IMU actualiza internamente a 416 Hz; el firmware la lee a 50 Hz.
  imu.settings.accelRange = 16;
  imu.settings.gyroRange = 2000;
  imu.settings.accelSampleRate = 416;
  imu.settings.gyroSampleRate = 416;
  if (imu.begin() != 0) {
    signalError("No se ha podido iniciar la IMU");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("IMU iniciada");
  gnssSerial.setRxBufferSize(4096);
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  configureGnss();
  Serial.println("GNSS iniciado a 10 Hz");
  calibrateGyroscope();
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSpi, 4000000)) {
    signalError("No se ha podido iniciar la microSD");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  if (!createCsvFile()) {
    signalError("No se ha podido crear el archivo CSV");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("microSD iniciada");
  if (!initializeLora()) {
    signalError("No se ha podido iniciar LoRa");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("SX1262 iniciado");
  Serial.println("LoRa: 869.525 MHz | BW125 | SF7 | CR4/5 | 14 dBm");
  sampleQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SampleRecord));
  if (sampleQueue == nullptr) {
    signalError("No se ha podido crear la cola");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("Cola creada");
  // Una orden y una respuesta bastan: loop() tramita las órdenes en serie.
  commandQueue = xQueueCreate(1, sizeof(char));
  commandResultQueue = xQueueCreate(1, sizeof(ControlResult));
  if (!commandQueue || !commandResultQueue) {
    signalError("No se han podido crear las colas de control");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  const BaseType_t storageResult =
      xTaskCreatePinnedToCore(storageTask, "storage", 4096, nullptr, 1, nullptr, 0);
  const BaseType_t acquisitionResult =
      xTaskCreatePinnedToCore(acquisitionTask, "acquisition", 4096, nullptr, 3, nullptr, 1);
  if (storageResult != pdPASS || acquisitionResult != pdPASS) {
    signalError("No se han podido crear las tareas");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("Tarea almacenamiento -> Core 0");
  Serial.println("Tarea adquisicion -> Core 1");
  Serial.println("IMU -> 50 Hz");
  Serial.println("GNSS -> 10 Hz");
  Serial.println("Registro iniciado");
  Serial.println("Ordenes LoRa: S=inicio | F=final | Q=estado");
  Serial.println("LED apagado = OK | LED parpadeando = error");
}
void loop() {
  pollLora();
  updateLed();
  delay(10);
}
