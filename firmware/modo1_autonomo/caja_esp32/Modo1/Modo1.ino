/*
  TFG - Teresa Tirado Dotor. Revisión 2026-10-18.
  MODO 1: Registro autónomo.

  Hardware: ESP32-WROOM-32, LSM6DS3, LG290P y microSD.
  Núcleo 1: acquisitionTask, prioridad 3, periodo nominal de 20 ms.
  Núcleo 0: storageTask, prioridad 1. Cola de 256 muestras entre ambas.
  loop(): LED.
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
LSM6DS3 imu(I2C_MODE, 0x6B);
HardwareSerial gnssSerial(2);
TinyGPSPlus gps;
SPIClass sdSpi(VSPI);
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
void acquisitionTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    readGnss();
    SampleRecord sample = createSample();
    if (xQueueSend(sampleQueue, &sample, 0) != pdPASS) {
      signalError("La cola de muestras esta llena");
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
    if (++rowsSinceFlush >= FLUSH_EVERY_ROWS) {
      csvFile.flush();
      rowsSinceFlush = 0;
      if (csvFile.getWriteError()) signalError("Error de flush en microSD");
    }
  }
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=======================================");
  Serial.println("TFG Teresa Tirado - Modo 1: registro autonomo");
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
  sampleQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SampleRecord));
  if (sampleQueue == nullptr) {
    signalError("No se ha podido crear la cola");
    while (true) {
      updateLed();
      delay(20);
    }
  }
  Serial.println("Cola creada");
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
  Serial.println("LED apagado = OK | LED parpadeando = error");
}
void loop() {
  updateLed();
  delay(20);
}
