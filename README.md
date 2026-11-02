# Sistema de telemetría para piragüismo

Código fuente asociado al Trabajo Fin de Grado **«Diseño e implementación de un sistema de telemetría deportiva para adquisición y análisis de datos de rendimiento en piragüismo»**, desarrollado en el Grado en Ingeniería Telemática en Telecomunicación de la Universidad de Extremadura.

El proyecto implementa un prototipo embarcado en una piragua capaz de registrar datos GNSS e inerciales, almacenarlos en una tarjeta microSD y ofrecer distintas alternativas de control y telemetría mediante Wi-Fi y LoRa. También incluye herramientas en Python para el control remoto, la visualización de telemetría y el análisis posterior de los registros.

## Funcionalidades principales

- Adquisición de datos de una IMU LSM6DS3 a 50 Hz.
- Recepción de posición, velocidad y tiempo mediante un receptor GNSS Quectel LG290P a 10 Hz.
- Almacenamiento de los registros en archivos CSV sobre una tarjeta microSD.
- Control local mediante una interfaz web Wi-Fi.
- Control remoto y consulta del estado mediante LoRa.
- Envío periódico de telemetría mediante LoRa.
- Postprocesado de los registros para obtener trayectorias, parciales, velocidad, cadencia y variables de la IMU.
- Generación de gráficas y de un informe de resultados en PDF.

## Hardware principal

- ESP32-WROOM-32 como unidad principal de la caja instalada en la piragua.
- Receptor GNSS Quectel LG290P.
- IMU LSM6DS3.
- Lector de tarjetas microSD.
- Transceptor LoRa SX1262 conectado directamente al ESP32 de la caja.
- XIAO ESP32-S3 y Wio-SX1262 en la estación base.

## Modos de funcionamiento

El sistema dispone de cuatro firmwares independientes:

1. **Modo 1 - Registro autónomo:** adquisición de datos y almacenamiento en la microSD sin control externo.
2. **Modo 2 - Control Wi-Fi:** incorpora un punto de acceso y una interfaz web para consultar el estado y marcar el inicio y el final de los tests.
3. **Modo 3 - Control LoRa:** permite consultar el estado y enviar las marcas de inicio y final desde una estación base conectada al ordenador.
4. **Modo 4 - Telemetría LoRa:** añade al control remoto el envío periódico de posición, velocidad y estado del sistema.

Los modos 3 y 4 incluyen dos programas de firmware: uno para el ESP32-WROOM-32 de la caja y otro para el XIAO ESP32-S3 de la estación base.

## Organización del repositorio

```text
tfg-telemetria-piraguismo/
├── firmware/
│   ├── modo1_autonomo/
│   ├── modo2_wifi/
│   ├── modo3_lora_control/
│   └── modo4_lora_telemetria/
│
├── aplicaciones/
│   ├── control_modo3/
│   ├── telemetria_modo4/
│   └── postprocesado/
│
└── dependencias.txt
```

## Dependencias

### Arduino

Los firmwares se desarrollaron utilizando Arduino IDE y requieren las siguientes bibliotecas:

- TinyGPSPlus.
- SparkFun LSM6DS3.
- RadioLib, para los modos 3 y 4.
- Bibliotecas incluidas en el núcleo ESP32: Wire, SPI, SD, WiFi y WebServer.

### Python

Las dependencias de las aplicaciones pueden instalarse ejecutando desde la carpeta principal:

```bash
py -m pip install -r dependencias.txt
```

## Ejecución de las aplicaciones

### Control del modo 3

Con la estación base conectada por USB, ejecutar:

```text
aplicaciones/control_modo3/ejecutar.bat
```

La aplicación permite seleccionar el puerto serie, consultar el estado del sistema y enviar las marcas de inicio y final.

### Telemetría del modo 4

Con la estación base conectada por USB, ejecutar:

```text
aplicaciones/telemetria_modo4/ejecutar.bat
```

La interfaz muestra los datos recibidos mediante LoRa y permite enviar las órdenes de control.

### Postprocesado

Para analizar un registro, se debe copiar el archivo CSV generado por la caja en la carpeta `aplicaciones/postprocesado` y ejecutar:

```text
aplicaciones/postprocesado/INICIAR_POSTPROCESADO.bat
```

El programa guía al usuario durante la selección del test y genera las gráficas y el informe de resultados correspondientes.

## Autora

**Teresa Tirado Dotor**  
Grado en Ingeniería Telemática en Telecomunicación  
Universidad de Extremadura

## Nota

Este repositorio recoge el código desarrollado para un prototipo académico. El sistema se ha diseñado como herramienta de apoyo al entrenamiento deportivo y no como dispositivo de navegación o de seguridad.
