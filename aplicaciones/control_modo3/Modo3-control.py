"""TFG Teresa Tirado — control del modo 3 LoRa. Revisión 2026-10-18."""

import time

import serial
from serial.tools import list_ports


BAUD = 115200
TIMEOUT_SECONDS = 4.0
MAX_ATTEMPTS = 3
MAX_SEQUENCE = 9999


def select_port():
    """Muestra los puertos disponibles y devuelve el seleccionado."""

    ports = list(list_ports.comports())

    if not ports:
        print("No se ha encontrado ningún puerto serie.")
        return None

    print("\nPuertos serie disponibles:")

    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {port.device} - {port.description}")

    while True:
        try:
            option = int(input("\nNúmero del puerto del XIAO base: "))

            if 1 <= option <= len(ports):
                return ports[option - 1].device

        except ValueError:
            pass

        print("Opción no válida.")


def parse_response(line, expected_sequence):
    """Interpreta una respuesta ACK o NACK recibida desde la piragua."""

    if not line.startswith("BASE_RX;"):
        return None

    parts = line.split(";", 3)

    if len(parts) != 4:
        return None

    rssi = parts[1]
    snr = parts[2]
    frame = parts[3]
    fields = frame.split(";")

    if len(fields) != 7:
        return None

    if fields[0] != "T" or fields[1] not in ("A", "N"):
        return None

    try:
        sequence = int(fields[2])
        active = fields[4] == "1"
        test_id = int(fields[5])
        samples = int(fields[6])
    except ValueError:
        return None

    if (sequence != expected_sequence or not 1 <= sequence <= MAX_SEQUENCE
            or fields[4] not in ("0", "1") or test_id < 0 or samples < 0):
        return None

    return {
        "accepted": fields[1] == "A",
        "result": fields[3],
        "active": active,
        "test_id": test_id,
        "samples": samples,
        "rssi": rssi,
        "snr": snr,
        "frame": frame,
    }


def send_command(port, sequence, order):
    """Envía una orden y realiza hasta tres intentos con igual secuencia."""

    frame = f"T;C;{sequence};{order}"

    print("\n" + "=" * 60)
    print(f"TX: {frame}")

    for attempt in range(1, MAX_ATTEMPTS + 1):
        port.write((frame + "\n").encode("ascii"))
        port.flush()

        deadline = time.monotonic() + TIMEOUT_SECONDS

        while time.monotonic() < deadline:
            raw = port.readline()

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()

            if line:
                print(line)

            response = parse_response(line, sequence)

            if response is not None:
                return response

        if attempt < MAX_ATTEMPTS:
            print(f"Sin respuesta. Reintento {attempt + 1}/{MAX_ATTEMPTS}...")

    return None


def show_response(response):
    """Presenta de forma comprensible la respuesta recibida."""

    if response is None:
        print("Sin confirmación: consulta el estado antes de asumir si la orden se aplicó.")
        return

    if response["accepted"]:
        messages = {
            "S": "Inicio aceptado: marca en cola de almacenamiento",
            "F": "Final aceptado: marca en cola de almacenamiento",
            "Q": "Estado consultado",
        }

        print(f"\n{messages.get(response['result'], 'Orden aceptada')}.")

    else:
        reasons = {
            "1": "ya existe un test activo",
            "2": "no existe ningún test activo",
            "3": "la marca anterior todavía está pendiente",
            "4": "cola llena: marca rechazada; el test no ha cambiado",
            "5": "orden desconocida o conflicto de secuencia",
        }

        reason = reasons.get(response["result"], "motivo desconocido")
        print(f"\nOrden rechazada: {reason}.")

    state = "EN CURSO" if response["active"] else "SIN TEST ACTIVO"

    print(f"Estado: {state}")
    print(f"Test: {response['test_id']}")
    print(f"Filas entregadas a SD: {response['samples']}")
    print(f"RSSI: {response['rssi']} dBm")
    print(f"SNR: {response['snr']} dB")


def main():
    """Ejecuta el menú de control remoto."""

    print("TFG - Teresa Tirado")
    print("Modo 3 - Control remoto LoRa simple")
    print("\nIMPORTANTE: cierra el Monitor Serie del XIAO base.")

    port_name = select_port()

    if port_name is None:
        return

    print(f"\nAbriendo {port_name} a {BAUD} baud...")

    try:
        port = serial.Serial(
            port_name,
            BAUD,
            timeout=0.2,
            write_timeout=2.0,
        )
    except serial.SerialException as error:
        print(f"No se pudo abrir el puerto: {error}")
        return

    with port:
        time.sleep(3.0)
        port.reset_input_buffer()
        print("Puerto abierto. XIAO base preparado.")

        sequence = 0

        while True:
            print("\n------------ CONTROL REMOTO LORA ------------")
            print("1. Consultar estado")
            print("2. Marcar inicio")
            print("3. Marcar final")
            print("0. Salir")
            print("----------------------------------------------")

            option = input("\nOpción: ").strip()

            orders = {
                "1": "Q",
                "2": "S",
                "3": "F",
            }

            if option == "0":
                break

            if option not in orders:
                print("Opción no válida.")
                continue

            sequence = sequence % MAX_SEQUENCE + 1
            try:
                response = send_command(port, sequence, orders[option])
            except (serial.SerialException, OSError) as error:
                print(f"Conexión serie interrumpida: {error}")
                break
            show_response(response)


if __name__ == "__main__":
    main()

