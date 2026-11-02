"""TFG Teresa Tirado Dotor - Modo 4 LoRa. Revisión 2026-10-18.

Interfaz para el XIAO base. Recibe telemetria no confirmada, envia las
ordenes S/F/Q y permite fijar un punto geográfico de referencia para la distancia directa.
La base se fija colocando la caja junto al ordenador; permanece fija hasta
que se pulsa de nuevo el botón. No hay GNSS en la estación XIAO.
ACK confirma marca en cola, no persistencia física en microSD.

Dependencia: py -m pip install pyserial
"""

from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Falta pyserial. Instálalo con: py -m pip install pyserial") from exc


BAUD = 115200
RESPONSE_TIMEOUT_S = 4.0
MAX_RETRIES = 3
COMMAND_COOLDOWN_S = 3.0
TELEMETRY_STALE_S = 4.0
GRAPH_SECONDS = 90.0
GRAPH_MAX_KMH = 30.0
DISPLAY_ZERO_THRESHOLD_MPS = 0.15
SEQUENCE_MAX = 9999


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    test_id: int
    active: bool
    gps_valid: bool
    satellites: int
    latitude: float | None
    longitude: float | None
    speed_mps: float | None


@dataclass(frozen=True)
class Response:
    accepted: bool
    sequence: int
    order: str
    active: bool
    test_id: int
    samples_written: int


def optional_float(value: str) -> float | None:
    if value == "N":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def parse_telemetry(frame: str) -> Telemetry | None:
    fields = frame.split(";")
    if len(fields) != 10 or fields[:2] != ["T", "D"]:
        return None
    try:
        sequence = int(fields[2])
        test_id = int(fields[3])
        active = fields[4] == "1"
        gps_valid = fields[5] == "1"
        satellites = int(fields[6])
        latitude = optional_float(fields[7])
        longitude = optional_float(fields[8])
        speed_mps = optional_float(fields[9])
    except ValueError:
        return None
    if not 1 <= sequence <= SEQUENCE_MAX:
        return None
    if fields[4] not in ("0", "1") or fields[5] not in ("0", "1"):
        return None
    if test_id < 0 or not 0 <= satellites <= 255:
        return None
    if latitude is not None and not -90 <= latitude <= 90:
        return None
    if longitude is not None and not -180 <= longitude <= 180:
        return None
    if speed_mps is not None and speed_mps < 0:
        return None
    if gps_valid and (latitude is None or longitude is None):
        return None
    return Telemetry(
        sequence, test_id, active, gps_valid, satellites,
        latitude, longitude, speed_mps,
    )


def parse_response(frame: str) -> Response | None:
    fields = frame.split(";")
    if len(fields) != 7 or fields[0] != "T" or fields[1] not in ("A", "N"):
        return None
    try:
        sequence = int(fields[2])
        active = fields[4] == "1"
        test_id = int(fields[5])
        samples = int(fields[6])
    except ValueError:
        return None
    if not 1 <= sequence <= SEQUENCE_MAX or len(fields[3]) != 1:
        return None
    if fields[4] not in ("0", "1") or test_id < 0 or samples < 0:
        return None
    return Response(fields[1] == "A", sequence, fields[3], active, test_id, samples)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    a = min(1.0, max(0.0, a))
    return 2 * radius * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class Mode4App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Telemetría y control remoto LoRa · Modo 4")
        self.root.geometry("1080x800")
        self.root.minsize(850, 700)

        self.serial_port: serial.Serial | None = None
        self.serial_stop = threading.Event()
        self.ui_queue: queue.Queue[tuple] = queue.Queue()
        self.response_queue: queue.Queue[tuple[int, Response]] = queue.Queue()
        self.write_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.connection_generation = 0
        self.reader_thread: threading.Thread | None = None
        self.command_sequence = 0
        self.last_command_started = float("-inf")

        self.last_tel_sequence: int | None = None
        self.last_tel_time: float | None = None
        self.last_state_time: float | None = None
        self.last_link_time: float | None = None
        self.tel_received = 0
        self.tel_lost = 0
        self.tel_invalid = 0
        self.current_position: tuple[float, float] | None = None
        self.temporary_base: tuple[float, float] | None = None
        self.speed_history: deque[tuple[float, float]] = deque()

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Desconectado")
        self.message_var = tk.StringVar(value="Selecciona el puerto del XIAO base.")
        self.speed_var = tk.StringVar(value="—")
        self.state_var = tk.StringVar(value="Sin datos actuales")
        self.test_var = tk.StringVar(value="—")
        self.samples_var = tk.StringVar(value="—")
        self.satellites_var = tk.StringVar(value="—")
        self.position_var = tk.StringVar(value="—")
        self.rssi_var = tk.StringVar(value="—")
        self.snr_var = tk.StringVar(value="—")
        self.distance_var = tk.StringVar(value="Punto base no fijado")
        self.stats_var = tk.StringVar(value="Recibidas 0 · Perdidas 0 · Inválidas 0")

        self.configure_styles()
        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_ui_queue)
        self.root.after(500, self.refresh_connection_state)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def configure_styles(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("Segoe UI", 19, "bold"))
        style.configure("Subtitle.TLabel", foreground="#667085", font=("Segoe UI", 10))
        style.configure("Card.TLabelframe", padding=7)
        style.configure("Value.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Speed.TLabel", foreground="#1570ef", font=("Segoe UI", 28, "bold"))

    def build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text="Telemetría y control remoto LoRa", style="Title.TLabel").pack()
        ttk.Label(
            outer,
            text="Modo 4",
            style="Subtitle.TLabel",
        ).pack(pady=(0, 6))

        dashboard = ttk.Frame(outer)
        dashboard.pack(fill="x", pady=3)
        dashboard.columnconfigure(0, weight=1, uniform="dashboard")
        dashboard.columnconfigure(1, weight=1, uniform="dashboard")

        left = ttk.Frame(dashboard)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right = ttk.Frame(dashboard)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        connection = ttk.LabelFrame(left, text="Conexión", style="Card.TLabelframe")
        connection.pack(fill="x", pady=(0, 4))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="Puerto").grid(row=0, column=0, padx=4)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=4)
        self.refresh_button = ttk.Button(connection, text="Actualizar", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(connection, text="Conectar", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(6, 2), ipady=2)
        ttk.Label(connection, textvariable=self.connection_var, style="Value.TLabel").grid(
            row=2, column=0, columnspan=2, pady=(3, 0)
        )

        controls = ttk.LabelFrame(left, text="Control remoto", style="Card.TLabelframe")
        controls.pack(fill="x", pady=(4, 0))
        controls.columnconfigure(0, weight=1)
        self.start_button = tk.Button(
            controls, text="MARCAR INICIO", bg="#12b76a", fg="white",
            activebackground="#039855", command=lambda: self.send_command("S"), state="disabled",
        )
        self.start_button.grid(row=0, column=0, sticky="ew", padx=4, pady=2, ipady=3)
        self.end_button = tk.Button(
            controls, text="MARCAR FINAL", bg="#f04438", fg="white",
            activebackground="#d92d20", command=lambda: self.send_command("F"), state="disabled",
        )
        self.end_button.grid(row=1, column=0, sticky="ew", padx=4, pady=2, ipady=3)
        self.query_button = ttk.Button(controls, text="CONSULTAR ESTADO", command=lambda: self.send_command("Q"), state="disabled")
        self.query_button.grid(row=2, column=0, sticky="ew", padx=4, pady=2, ipady=3)
        self.base_button = ttk.Button(controls, text="FIJAR PUNTO BASE", command=self.set_temporary_base, state="disabled")
        self.base_button.grid(row=3, column=0, sticky="ew", padx=4, pady=2, ipady=3)
        ttk.Label(controls, textvariable=self.message_var, wraplength=410, justify="center").grid(
            row=4, column=0, sticky="ew", pady=(6, 0)
        )

        speed = ttk.LabelFrame(right, text="Velocidad", style="Card.TLabelframe")
        speed.pack(fill="x", pady=(0, 4))
        ttk.Label(speed, textvariable=self.speed_var, style="Speed.TLabel").pack(pady=(2, 0))
        ttk.Label(speed, text="km/h").pack()

        test = ttk.LabelFrame(right, text="Registro", style="Card.TLabelframe")
        test.pack(fill="x", pady=4)
        self.value_row(test, "Estado", self.state_var)
        self.value_row(test, "Test", self.test_var)
        self.value_row(test, "Muestras escritas (consulta)", self.samples_var)

        link = ttk.LabelFrame(right, text="GNSS y enlace", style="Card.TLabelframe")
        link.pack(fill="x", pady=(4, 0))
        self.value_row(link, "Satélites", self.satellites_var)
        self.value_row(link, "RSSI", self.rssi_var)
        self.value_row(link, "SNR", self.snr_var)
        ttk.Label(link, textvariable=self.position_var, wraplength=280).pack(pady=(4, 0))

        distance = ttk.LabelFrame(outer, text="Distancia respecto al punto base fijado", style="Card.TLabelframe")
        distance.pack(fill="x", pady=5)
        ttk.Label(distance, textvariable=self.distance_var, style="Value.TLabel").pack(pady=2)

        ttk.Label(outer, textvariable=self.stats_var, style="Subtitle.TLabel").pack(pady=(4, 0))
        graph_frame = ttk.LabelFrame(outer, text="Velocidad · últimos 90 segundos", style="Card.TLabelframe")
        graph_frame.pack(fill="both", expand=True, pady=(3, 0))
        self.graph = tk.Canvas(graph_frame, bg="white", highlightthickness=0, height=250)
        self.graph.pack(fill="both", expand=True)
        self.graph.bind("<Configure>", lambda _event: self.draw_graph())
       

    @staticmethod
    def value_row(parent: ttk.Widget, name: str, variable: tk.StringVar) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=name).pack(side="left")
        ttk.Label(row, textvariable=variable, style="Value.TLabel").pack(side="right")

    def refresh_ports(self) -> None:
        ports = [f"{item.device} — {item.description}" for item in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        if not self.port_var.get():
            messagebox.showwarning("Puerto", "Selecciona el puerto del XIAO base.")
            return
        port_name = self.port_var.get().split(" — ", 1)[0]
        self.connection_generation += 1
        generation = self.connection_generation
        self.connect_button.configure(state="disabled")
        self.refresh_button.configure(state="disabled")
        self.port_combo.configure(state="disabled")
        self.connection_var.set("Conectando…")
        threading.Thread(target=self.connect_worker, args=(port_name, generation), daemon=True).start()

    def connect_worker(self, port_name: str, generation: int) -> None:
        port = None
        try:
            port = serial.Serial(port_name, BAUD, timeout=0.2, write_timeout=2.0)
            time.sleep(3.0)
            if generation != self.connection_generation:
                port.close()
                return
            port.reset_input_buffer()
            self.ui_queue.put(("connected", port_name, port, generation))
        except (serial.SerialException, OSError) as exc:
            if port:
                port.close()
            self.ui_queue.put(("connection_error", str(exc), generation))

    def disconnect(self) -> None:
        self.connection_generation += 1
        self.serial_stop.set()
        port = self.serial_port
        self.serial_port = None
        if port:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.last_tel_time = None
        self.last_state_time = None
        self.last_link_time = None
        self.last_tel_sequence = None
        self.current_position = None
        # La referencia geográfica fijada por el usuario se conserva al desconectar.
        self.speed_history.clear()
        self.speed_var.set("—")
        self.position_var.set("Sin posición actual")
        self.state_var.set("Sin datos actuales")
        self.test_var.set("—")
        self.samples_var.set("—")
        self.satellites_var.set("—")
        self.rssi_var.set("—")
        self.snr_var.set("—")
        self.update_distance()
        self.draw_graph()
        self.set_controls(False)
        self.connect_button.configure(text="Conectar", state="normal")
        self.refresh_button.configure(state="normal")
        self.port_combo.configure(state="readonly")
        self.connection_var.set("Desconectado")

    def reader_loop(self, port, stop_event, generation: int) -> None:
        while not stop_event.is_set() and generation == self.connection_generation:
            try:
                raw = port.readline()
            except (serial.SerialException, OSError) as exc:
                if not stop_event.is_set():
                    self.ui_queue.put(("serial_error", str(exc), generation))
                return
            if raw and generation == self.connection_generation:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.process_serial_line(line, generation)

    def process_serial_line(self, line: str, generation: int) -> None:
        if not line.startswith("BASE_RX;"):
            if line.startswith("ERROR_BASE;"):
                self.ui_queue.put(("message", line, generation))
            return
        parts = line.split(";", 4)
        if len(parts) != 5:
            self.ui_queue.put(("invalid", generation))
            return
        try:
            rssi, snr = float(parts[1]), float(parts[2])
            if not math.isfinite(rssi) or not math.isfinite(snr):
                raise ValueError
        except ValueError:
            self.ui_queue.put(("invalid", generation))
            return
        frame = parts[4]
        if frame.startswith("T;D;"):
            tel = parse_telemetry(frame)
            self.ui_queue.put(("telemetry", tel, rssi, snr, generation) if tel else ("invalid", generation))
        elif frame.startswith(("T;A;", "T;N;")):
            response = parse_response(frame)
            if response:
                self.response_queue.put((generation, response))
                self.ui_queue.put(("response", response, rssi, snr, generation))
            else:
                self.ui_queue.put(("invalid", generation))

    def send_command(self, order: str) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Conexión", "Conecta primero el XIAO base.")
            return
        remaining = COMMAND_COOLDOWN_S - (time.monotonic() - self.last_command_started)
        if remaining > 0:
            self.message_var.set(f"Espera {remaining:.1f} s antes de otra orden.")
            return
        # Se reserva antes de lanzar el hilo para impedir dos envíos simultáneos.
        if not self.command_lock.acquire(blocking=False):
            self.message_var.set("Ya hay una orden en curso.")
            return
        self.command_sequence = self.command_sequence % SEQUENCE_MAX + 1
        sequence = self.command_sequence
        frame = f"T;C;{sequence};{order}"
        self.last_command_started = time.monotonic()
        threading.Thread(target=self.command_worker,
            args=(sequence, order, frame, self.serial_port, self.connection_generation), daemon=True).start()

    def command_worker(self, sequence: int, order: str, frame: str, port, generation: int) -> None:
        try:
            while True:
                try:
                    self.response_queue.get_nowait()
                except queue.Empty:
                    break
            for attempt in range(1, MAX_RETRIES + 1):
                if generation != self.connection_generation or not port.is_open:
                    return
                try:
                    with self.write_lock:
                        port.write((frame + "\n").encode("ascii"))
                        port.flush()
                except (serial.SerialException, OSError) as exc:
                    self.ui_queue.put(("serial_error", str(exc), generation))
                    return
                self.ui_queue.put(("message", f"Orden {order}, intento {attempt}/{MAX_RETRIES}…", generation))
                deadline = time.monotonic() + RESPONSE_TIMEOUT_S
                while time.monotonic() < deadline:
                    if generation != self.connection_generation:
                        return
                    try:
                        response_generation, response = self.response_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if response_generation == generation and response.sequence == sequence:
                        if response.accepted and response.order != order:
                            continue
                        self.ui_queue.put(("command_finished", response, generation))
                        return
            self.ui_queue.put(("message", "Sin confirmación. Consulta el estado antes de asumir si se aplicó la orden.", generation))
        finally:
            self.command_lock.release()

    def handle_telemetry(self, tel: Telemetry, rssi: float, snr: float) -> None:
        if self.last_tel_sequence is not None:
            difference = (tel.sequence - self.last_tel_sequence) % SEQUENCE_MAX
            if difference == 0:
                return  # Duplicado: no es una nueva posición ni actualiza su antigüedad.
            if difference > SEQUENCE_MAX // 2:
                self.message_var.set("Secuencia reiniciada o fuera de orden; se toma una nueva referencia.")
            elif difference > 1:
                self.tel_lost += difference - 1
        self.last_tel_sequence = tel.sequence
        self.last_tel_time = time.monotonic()
        self.last_state_time = self.last_tel_time
        self.last_link_time = self.last_tel_time
        self.tel_received += 1

        self.test_var.set(str(tel.test_id))
        self.state_var.set("EN CURSO" if tel.active else "SIN TEST ACTIVO")
        self.satellites_var.set(str(tel.satellites) if tel.gps_valid else "Sin posición")
        self.rssi_var.set(f"{rssi:.1f} dBm")
        self.snr_var.set(f"{snr:.1f} dB")

        if tel.gps_valid and tel.latitude is not None and tel.longitude is not None:
            self.current_position = (tel.latitude, tel.longitude)
            self.position_var.set(f"{tel.latitude:.6f}, {tel.longitude:.6f}")
        else:
            self.current_position = None
            self.position_var.set("Posición no válida")

        if tel.speed_mps is None:
            self.speed_var.set("—")
        else:
            speed = 0.0 if abs(tel.speed_mps) < DISPLAY_ZERO_THRESHOLD_MPS else tel.speed_mps
            speed_kmh = speed * 3.6
            self.speed_var.set(f"{speed_kmh:.2f}")
            now = time.monotonic()
            self.speed_history.append((now, speed_kmh))
            while self.speed_history and now - self.speed_history[0][0] > GRAPH_SECONDS:
                self.speed_history.popleft()
            self.draw_graph()

        self.update_distance()
        self.update_stats()

    def handle_response(self, response: Response) -> None:
        self.last_state_time = time.monotonic()
        self.test_var.set(str(response.test_id))
        self.state_var.set("EN CURSO" if response.active else "SIN TEST ACTIVO")
        self.samples_var.set(str(response.samples_written))
        if response.accepted:
            text = {"S": "Inicio aceptado en cola", "F": "Final aceptado en cola", "Q": "Estado actualizado"}.get(response.order, "Orden aceptada")
            self.message_var.set(f"{text}: test {response.test_id}, {response.samples_written} muestras.")
        else:
            reasons = {
                "1": "ya existe un test activo",
                "2": "no existe ningún test activo",
                "3": "la marca anterior continúa pendiente",
                "4": "cola llena: marca rechazada; el test no ha cambiado",
                "5": "orden desconocida o conflicto de secuencia",
            }
            self.message_var.set(f"Orden rechazada: {reasons.get(response.order, 'motivo desconocido')}.")

    def set_temporary_base(self) -> None:
        if not self.position_is_fresh():
            self.message_var.set("Necesitamos una posición GNSS válida recibida hace menos de 4 s para fijar la base.")
            return
        self.temporary_base = self.current_position
        self.update_distance()
        self.message_var.set("Punto base fijado. Permanecerá fijo aunque se aleje la piragua.")

    def position_is_fresh(self) -> bool:
        return (self.serial_port is not None and self.serial_port.is_open
            and self.current_position is not None and self.last_tel_time is not None
            and time.monotonic() - self.last_tel_time <= TELEMETRY_STALE_S)

    def update_distance(self) -> None:
        if self.temporary_base is None:
            self.distance_var.set("Punto base no fijado")
        elif not self.position_is_fresh():
            self.distance_var.set("Base conservada · sin posición actual")
        else:
            distance = haversine_m(*self.temporary_base, *self.current_position)
            self.distance_var.set(f"{distance:.1f} m (distancia directa)")

    def draw_graph(self) -> None:
        canvas = self.graph
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        left, right, top, bottom = 45, 12, 12, 30
        plot_w = width - left - right
        plot_h = height - top - bottom
        graph_step_kmh = 2
        for value in range(0, int(GRAPH_MAX_KMH) + 1, graph_step_kmh):
            y = top + plot_h - plot_h * value / GRAPH_MAX_KMH
            canvas.create_line(left, y, left + plot_w, y, fill="#eaecf0")
            canvas.create_text(left - 6, y, text=str(value), anchor="e", fill="#667085")
        if not self.speed_history:
            canvas.create_text(width / 2, height / 2, text="Esperando velocidad GNSS", fill="#98a2b3")
            return
        now = time.monotonic()
        points: list[float] = []
        for timestamp, speed in self.speed_history:
            age = now - timestamp
            if age > GRAPH_SECONDS:
                continue
            x = left + plot_w * (1 - min(age, GRAPH_SECONDS) / GRAPH_SECONDS)
            y = top + plot_h - min(max(speed, 0), GRAPH_MAX_KMH) / GRAPH_MAX_KMH * plot_h
            points.extend((x, y))
        if len(points) >= 4:
            canvas.create_line(*points, fill="#1570ef", width=2, smooth=True)
        elif len(points) == 2:
            x, y = points
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#1570ef", outline="")

    def update_stats(self) -> None:
        self.stats_var.set(
            f"Recibidas {self.tel_received} · Perdidas {self.tel_lost} · Inválidas {self.tel_invalid}"
        )

    def set_controls(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for button in (self.start_button, self.end_button, self.query_button, self.base_button):
            button.configure(state=state)

    def process_ui_queue(self) -> None:
        try:
            while True:
                event = self.ui_queue.get_nowait()
                kind = event[0]
                generation = event[-1]
                if generation != self.connection_generation:
                    if kind == "connected":
                        event[2].close()
                    continue
                if kind == "connected":
                    self.serial_port = event[2]
                    self.serial_stop = threading.Event()
                    self.last_tel_sequence = None
                    self.last_tel_time = None
                    self.last_state_time = None
                    self.last_link_time = None
                    self.tel_received = self.tel_lost = self.tel_invalid = 0
                    self.current_position = None
                    self.speed_history.clear()
                    self.update_stats()
                    self.reader_thread = threading.Thread(target=self.reader_loop,
                        args=(self.serial_port, self.serial_stop, generation), daemon=True)
                    self.reader_thread.start()
                    self.connection_var.set("Conectado")
                    self.connect_button.configure(text="Desconectar", state="normal")
                    self.set_controls(True)
                    self.message_var.set(f"Conectado a {event[1]}. Esperando telemetría…")
                elif kind == "connection_error":
                    self.connection_var.set("Desconectado")
                    self.connect_button.configure(text="Conectar", state="normal")
                    self.refresh_button.configure(state="normal")
                    self.port_combo.configure(state="readonly")
                    self.message_var.set(f"No se pudo abrir el puerto: {event[1]}")
                elif kind == "serial_error":
                    self.message_var.set(f"Error serie: {event[1]}")
                    self.disconnect()
                elif kind == "telemetry":
                    self.handle_telemetry(event[1], event[2], event[3])
                elif kind == "invalid":
                    self.tel_invalid += 1
                    self.update_stats()
                elif kind == "response":
                    self.last_link_time = time.monotonic()
                    self.rssi_var.set(f"{event[2]:.1f} dBm")
                    self.snr_var.set(f"{event[3]:.1f} dB")
                elif kind == "command_finished":
                    self.handle_response(event[1])
                elif kind == "message":
                    self.message_var.set(event[1])
        except queue.Empty:
            pass
        self.root.after(100, self.process_ui_queue)

    def refresh_connection_state(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            if self.last_tel_time is None:
                self.connection_var.set("Conectado · sin datos")
            else:
                age = time.monotonic() - self.last_tel_time
                if age > TELEMETRY_STALE_S:
                    self.connection_var.set(f"Sin telemetría · último dato hace {age:.0f} s")
                    self.speed_var.set("— (dato antiguo)")
                    self.position_var.set("Sin posición actual · último dato antiguo")
                    if self.last_state_time is None or time.monotonic() - self.last_state_time > TELEMETRY_STALE_S:
                        self.state_var.set("Sin estado actual")
                        self.test_var.set("—")
                    self.satellites_var.set("Dato antiguo")
                    if self.last_link_time is None or time.monotonic() - self.last_link_time > TELEMETRY_STALE_S:
                        self.rssi_var.set("Dato antiguo")
                        self.snr_var.set("Dato antiguo")
                else:
                    self.connection_var.set("Conectado")
            self.base_button.configure(state="normal" if self.position_is_fresh() else "disabled")
        self.update_distance()
        self.draw_graph()
        self.root.after(500, self.refresh_connection_state)

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    Mode4App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
