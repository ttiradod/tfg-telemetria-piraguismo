import math
import re
import unicodedata

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.ticker import AutoMinorLocator, FormatStrFormatter, MaxNLocator, MultipleLocator

from geographiclib.geodesic import Geodesic

# ============================================================
# AYUDAS GENERALES
# ============================================================

def elegir_paso_tiempo(duracion_s):
    if duracion_s <= 30: return 1
    if duracion_s <= 60: return 2
    if duracion_s <= 120: return 5
    if duracion_s <= 300: return 10
    return 20


def elegir_paso_distancia(distancia_m):
    if distancia_m <= 100: return 10
    if distancia_m <= 250: return 25
    if distancia_m <= 500: return 50
    return 100




def configurar_cuadricula_distancia(grafica, paso_m):
    grafica.xaxis.set_major_locator(MultipleLocator(paso_m))
    grafica.xaxis.set_minor_locator(MultipleLocator(paso_m / 2))
    grafica.yaxis.set_minor_locator(AutoMinorLocator(2))
    grafica.grid(True, which="major", color="gray", alpha=0.45, linewidth=0.8)
    grafica.grid(True, which="minor", color="gray", alpha=0.15, linestyle=":", linewidth=0.6)


# ============================================================
# TRAYECTORIA GNSS
# ============================================================

def nombre_direccion_cardinal(rumbo):
    nombres = ("norte", "noreste", "este", "sureste", "sur", "suroeste", "oeste", "noroeste")
    return nombres[int((rumbo + 22.5) // 45) % 8]


def calcular_rumbo_general(latitud, longitud):
    cantidad = min(10, max(1, len(latitud) // 10))
    lat1 = float(np.mean(latitud[:cantidad]))
    lon1 = float(np.mean(longitud[:cantidad]))
    lat2 = float(np.mean(latitud[-cantidad:]))
    lon2 = float(np.mean(longitud[-cantidad:]))

    if abs(lat2 - lat1) < 1e-12 and abs(lon2 - lon1) < 1e-12:
        return None

    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    este = math.sin(dlon) * math.cos(lat2r)
    norte = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(este, norte)) + 360) % 360


def interpretar_direccion_viento(texto):
    if texto is None:
        return None

    original = str(texto).strip()
    if not original:
        return None

    normalizado = unicodedata.normalize("NFD", original)
    normalizado = "".join(c for c in normalizado if unicodedata.category(c) != "Mn").lower()
    if normalizado in ("no indicado", "sin dato", "desconocido"):
        return None

    normalizado = re.sub(r"\b(viento|del|de|desde|procedente|procedencia)\b", " ", normalizado)
    normalizado = re.sub(r"[\s_-]+", "", normalizado)

    rumbos = {
        "n": 0,
        "nne": 22.5,
        "ne": 45,
        "ene": 67.5,
        "e": 90,
        "ese": 112.5,
        "se": 135,
        "sse": 157.5,
        "s": 180,
        "sso": 202.5,
        "so": 225,
        "oso": 247.5,
        "o": 270,
        "ono": 292.5,
        "no": 315,
        "nno": 337.5,
    }
    if normalizado in rumbos:
        return float(rumbos[normalizado])

    numero = re.search(r"[-+]?\d+(?:[.,]\d+)?", original)
    if numero is None:
        return None
    try:
        return float(numero.group().replace(",", ".")) % 360
    except ValueError:
        return None


def abreviatura_direccion(rumbo):
    nombres = (
        "N", "NNE", "NE", "ENE",
        "E", "ESE", "SE", "SSE",
        "S", "SSO", "SO", "OSO",
        "O", "ONO", "NO", "NNO",
    )
    return nombres[int((rumbo + 11.25) // 22.5) % 16]


def formatear_direccion_viento(texto, rumbo):
    abreviatura = abreviatura_direccion(rumbo)
    return f"{abreviatura} ({rumbo:g}°)"


def dibujar_brujula(eje):
    x, y = 0.17, 0.80
    flecha = {"arrowstyle": "-|>", "color": "black", "linewidth": 1.5}
    eje.annotate("", xy=(x, y + 0.08), xytext=(x, y - 0.08), xycoords="axes fraction", arrowprops=flecha)
    eje.annotate("", xy=(x + 0.07, y), xytext=(x - 0.07, y), xycoords="axes fraction", arrowprops=flecha)
    eje.text(x, y + 0.10, "N", transform=eje.transAxes, ha="center", va="center", fontweight="bold")
    eje.text(x, y - 0.10, "S", transform=eje.transAxes, ha="center", va="center", fontweight="bold")
    eje.text(x + 0.09, y, "E", transform=eje.transAxes, ha="center", va="center", fontweight="bold")
    eje.text(x - 0.09, y, "O", transform=eje.transAxes, ha="center", va="center", fontweight="bold")


def dibujar_viento(eje, direccion, velocidad):
    rumbo = interpretar_direccion_viento(direccion)
    if rumbo is None:
        return

    destino = math.radians((rumbo + 180) % 360)
    dx, dy = math.sin(destino), math.cos(destino)
    x, y, largo = 0.78, 0.70, 0.10

    eje.annotate(
        "",
        xy=(x + dx * largo, y + dy * largo),
        xytext=(x - dx * largo, y - dy * largo),
        xycoords="axes fraction",
        arrowprops={"arrowstyle": "-|>", "color": "#1f77b4", "linewidth": 2.4, "mutation_scale": 16},
        zorder=8,
    )

    texto = f"Viento\nDirección: {formatear_direccion_viento(direccion, rumbo)}"
    if velocidad not in (None, "", "No indicado"):
        texto += f"\nVelocidad: {velocidad}"

    eje.text(
        x, 0.96, texto, transform=eje.transAxes, ha="center", va="top",
        fontsize=8.2, color="#174a73",
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "#1f77b4", "alpha": 0.90},
        zorder=9,
    )


# ============================================================
# VELOCIDAD RESPECTO A LA DISTANCIA
# ============================================================

def guardar_velocidad(datos_prueba, parciales, meta, distancia_total, ruta):
    """Velocidad GNSS frente al avance proyectado sobre el eje de la pista."""
    from analisis import medias_velocidad_1s
    import pandas as pd

    filas = ((datos_prueba.gps_new == 1) & (datos_prueba.gps_valid == 1) &
             np.isfinite(datos_prueba[["lat_deg", "lon_deg", "speed_2d_mps"]]).all(axis=1))
    gps = datos_prueba.loc[filas, ["t_us", "lat_deg", "lon_deg", "speed_2d_mps"]].copy()
    if meta is None:
        raise ValueError("Falta la meta para representar la velocidad sobre el eje.")
    distancias = []
    for _, fila in gps.iterrows():
        posicion = Geodesic.WGS84.Inverse(meta["lat_inicio"], meta["lon_inicio"], fila.lat_deg, fila.lon_deg)
        distancias.append(posicion["s12"] * math.cos(math.radians(posicion["azi1"] - meta["azimut_deg"])))
    gps["distancia_m"] = distancias
    # Puntos de borde interpolados: se identifican como tales en el registro,
    # no se añaden al CSV ni al recuento de muestras de la IMU.
    bordes = pd.DataFrame([
        dict(t_us=meta["tiempo_inicio_us"], distancia_m=0., speed_2d_mps=meta["velocidad_inicio_mps"]),
        dict(t_us=meta["tiempo_cruce_us"], distancia_m=distancia_total, speed_2d_mps=meta["velocidad_cruce_mps"]),
    ])
    # La media y su máximo se calculan sobre las mismas muestras que en resultados.
    gps["media_1s"] = medias_velocidad_1s(gps)
    figura, grafica = plt.subplots(figsize=(14, 7))
    bruto = pd.concat([gps, bordes], ignore_index=True).sort_values("t_us").drop_duplicates("t_us")
    grafica.plot(bruto.distancia_m, bruto.speed_2d_mps * 3.6, linewidth=1, alpha=.6, label="GNSS y bordes interpolados")
    grafica.plot(gps.distancia_m, gps.media_1s * 3.6, linewidth=2, label="Media móvil ~1 s")
    for parcial in parciales[:-1]:
        grafica.axvline(parcial["hasta_m"], linestyle="--", linewidth=1)
    grafica.axvline(distancia_total, color="red", linewidth=2, label="Meta virtual")
    if gps.media_1s.notna().any():
        fila = gps.loc[gps.media_1s.idxmax()]
        grafica.scatter(fila.distancia_m, fila.media_1s * 3.6, s=50)
        grafica.annotate(
    f"Máxima 1 s: {fila.media_1s * 3.6:.2f} km/h\n"
    f"a {fila.distancia_m:.1f} m",
    xy=(fila.distancia_m, fila.media_1s * 3.6),
    xytext=(8, -35),
    textcoords="offset points",
    fontsize=9,
    ha="left",
    va="top",
    bbox={
        "facecolor": "white",
        "alpha": 0.8,
        "edgecolor": "none",
        "pad": 1.5
    }
)
    grafica.set(title="Velocidad respecto a la distancia", xlabel="Distancia desde la salida (m)",
                ylabel="Velocidad (km/h)", xlim=(-10, distancia_total + 10), ylim=(0, None))
    configurar_cuadricula_distancia(grafica, elegir_paso_distancia(distancia_total))
    figura.tight_layout()
    figura.savefig(ruta, dpi=180, bbox_inches="tight")
    plt.close(figura)
    return True


def guardar_trayectoria(datos_prueba, datos_informe, ruta, meta=None, parciales=None):
    filas = (
        (datos_prueba["gps_new"] == 1)
        & (datos_prueba["gps_valid"] == 1)
        & datos_prueba["lat_deg"].notna()
        & datos_prueba["lon_deg"].notna()
        & datos_prueba["speed_2d_mps"].notna()
    )
    gps = datos_prueba.loc[filas, ["lat_deg", "lon_deg", "speed_2d_mps"]].copy()
    if len(gps) < 2:
        return False

    lat = gps["lat_deg"].to_numpy(float)
    lon = gps["lon_deg"].to_numpy(float)
    velocidad = gps["speed_2d_mps"].to_numpy(float) * 3.6

    puntos = np.column_stack((lon, lat))
    segmentos = np.stack((puntos[:-1], puntos[1:]), axis=1)
    velocidad_segmentos = (velocidad[:-1] + velocidad[1:]) / 2
    minimo, maximo = float(np.min(velocidad_segmentos)), float(np.max(velocidad_segmentos))
    if maximo == minimo:
        maximo += 0.1

    figura, eje = plt.subplots(figsize=(10.8, 4.8))


    linea = LineCollection(segmentos, cmap="turbo", norm=Normalize(minimo, maximo), linewidth=3)
    linea.set_array(velocidad_segmentos)
    eje.add_collection(linea)

    lon_salida, lat_salida = lon[0], lat[0]

    eje.scatter(lon_salida, lat_salida, color="#17823b", s=30, edgecolor="black", linewidth=0.5, zorder=7)

    lon_corregida = lon * np.cos(np.radians(float(np.mean(lat))))
    acumulada = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(lon_corregida), np.diff(lat)))))
    if acumulada[-1] > 0:
        x1 = np.interp(acumulada[-1] * 0.44, acumulada, lon)
        y1 = np.interp(acumulada[-1] * 0.44, acumulada, lat)
        x2 = np.interp(acumulada[-1] * 0.56, acumulada, lon)
        y2 = np.interp(acumulada[-1] * 0.56, acumulada, lat)
        eje.annotate("", xy=(x2, y2), xytext=(x1, y1), arrowprops={"arrowstyle": "-|>", "color": "black", "linewidth": 2.5, "mutation_scale": 18}, zorder=8)

    # Cada segmento une dos posiciones GNSS consecutivas del CSV. No se
    # sustituye la trayectoria por el eje recto utilizado para la pista virtual.
    eje.margins(x=0.08, y=0.08)
    eje.set_aspect(1 / np.cos(np.radians(float(np.mean(lat)))), adjustable="datalim")
    eje.grid(True, linestyle=":", linewidth=0.7, alpha=0.7)
    eje.set_axisbelow(True)
    eje.set_xlabel("Longitud (°)")
    eje.set_ylabel("Latitud (°)")
    eje.xaxis.set_major_locator(MaxNLocator(5))
    eje.yaxis.set_major_locator(MaxNLocator(6))
    eje.xaxis.set_major_formatter(FormatStrFormatter("%.6f"))
    eje.yaxis.set_major_formatter(FormatStrFormatter("%.6f"))
    plt.setp(eje.get_xticklabels(), rotation=25, ha="right")

    rumbo = calcular_rumbo_general(lat, lon)
    if rumbo is None:
        titulo = "Trayectoria GNSS según coordenadas geográficas"
    else:
        titulo = f"Trayectoria GNSS según coordenadas geográficas\nSalida → final | Rumbo {rumbo:.2f}° ({nombre_direccion_cardinal(rumbo)})"
    eje.set_title(titulo)

    dibujar_brujula(eje)
    if datos_informe is not None:
        dibujar_viento(eje, datos_informe.get("direccion_viento"), datos_informe.get("velocidad_viento"))

    barra = figura.colorbar(linea, ax=eje, pad=0.02)
    barra.set_label("Velocidad GNSS (km/h)")
    figura.tight_layout()
    figura.savefig(ruta, dpi=180, bbox_inches="tight")
    plt.close(figura)
    return True



# ============================================================
# ACELERÓMETRO Y GIROSCOPIO
# ============================================================

def configurar_eje_imu(grafica, duracion, parciales, positivo, negativo):
    grafica.axhline(0, color="black", linestyle="--", linewidth=1)
    for parcial in parciales[:-1]:
        grafica.axvline(parcial["tiempo_acumulado_s"], color="purple", linestyle=":", linewidth=1, alpha=0.65)

    caja = {"facecolor": "white", "alpha": 0.75, "edgecolor": "none"}
    grafica.text(0.99, 0.92, positivo, transform=grafica.transAxes, ha="right", va="top", fontsize=9, color="darkgreen", bbox=caja)
    grafica.text(0.99, 0.08, negativo, transform=grafica.transAxes, ha="right", va="bottom", fontsize=9, color="darkred", bbox=caja)

    paso = elegir_paso_tiempo(duracion)
    grafica.xaxis.set_major_locator(MultipleLocator(paso))
    grafica.xaxis.set_minor_locator(MultipleLocator(paso / 2))
    grafica.yaxis.set_minor_locator(AutoMinorLocator(2))
    grafica.grid(True, which="major", color="gray", alpha=0.45, linewidth=0.8)
    grafica.grid(True, which="minor", color="gray", alpha=0.15, linestyle=":", linewidth=0.6)
    grafica.set_xlim(0, duracion)


def guardar_grupo_imu(datos_prueba, parciales, titulo, configuracion, unidad, ruta):
    if len(datos_prueba) == 0 or len(parciales) == 0:
        return False

    tiempo = (datos_prueba["t_us"] - datos_prueba["t_us"].iloc[0]) / 1_000_000
    duracion = datos_prueba.attrs.get("duracion_s", float(tiempo.iloc[-1]))
    figura, graficas = plt.subplots(3, 1, figsize=(16, 9), sharex=True)

    for grafica, (columna, titulo_eje, color, positivo, negativo) in zip(graficas, configuracion):
        grafica.plot(tiempo, datos_prueba[columna], color=color, linewidth=1)
        grafica.set_title(titulo_eje)
        grafica.set_ylabel(unidad)
        configurar_eje_imu(grafica, duracion, parciales, positivo, negativo)

    graficas[-1].set_xlabel("Tiempo desde la salida (s)")
    figura.suptitle(titulo, fontsize=16, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    figura.savefig(ruta, dpi=200, bbox_inches="tight")
    plt.close(figura)
    return True


def guardar_acelerometro(datos_prueba, parciales, ruta):
    ejes = [
        ("ax_g", "Acelerómetro ax_g - longitudinal (avance/frenado)", "red", "Respuesta longitudinal positiva (+) - avance", "Respuesta longitudinal negativa (-) - frenado"),
        ("ay_g", "Acelerómetro ay_g - aceleración lateral", "darkorange", "Respuesta lateral hacia la izquierda (+)", "Respuesta lateral hacia la derecha (-)"),
        ("az_g", "Acelerómetro az_g - vertical y gravedad", "green", "Eje Z positivo (+)", "Eje Z negativo (-) - reposo cercano a -1 g"),
    ]
    return guardar_grupo_imu(datos_prueba, parciales, "Acelerómetro: respuesta longitudinal, lateral y vertical durante la prueba", ejes, "Aceleración (g)", ruta)


def guardar_giroscopio(datos_prueba, parciales, ruta):
    ejes = [
        ("gx_dps", "Giroscopio gx_dps - velocidad de ladeo", "purple", "Velocidad de ladeo hacia la derecha (+)", "Velocidad de ladeo hacia la izquierda (-)"),
        ("gy_dps", "Giroscopio gy_dps - velocidad de cabeceo", "brown", "La proa se levanta (+)", "La proa se hunde (-)"),
        ("gz_dps", "Giroscopio gz_dps - velocidad de giro horizontal", "teal", "Velocidad de giro hacia la derecha (+)", "Velocidad de giro hacia la izquierda (-)"),
    ]
    return guardar_grupo_imu(datos_prueba, parciales, "Giroscopio: ladeo, cabeceo y giro horizontal durante la prueba", ejes, "Velocidad angular (grados/s)", ruta)




def guardar_comprobacion_paladas(datos_prueba, cadencia, parciales, ruta):
    """Vista completa y detalle inicial; los puntos rojos son las paladas contadas."""
    tiempo = (datos_prueba.t_us.to_numpy() - datos_prueba.t_us.iloc[0]) / 1e6
    suave = np.asarray(cadencia["senal_suave"])
    picos = np.asarray(cadencia["indices_picos"], dtype=int)
    figura, ejes = plt.subplots(2, 1, figsize=(16, 8))
    for eje in ejes:
        eje.plot(tiempo, datos_prueba.gy_dps, alpha=.35, linewidth=.7, label="gy original")
        eje.plot(tiempo, suave, linewidth=1, label="Media móvil ~0,1 s")
        eje.scatter(tiempo[picos], suave[picos], color="red", s=16, zorder=5, label="Paladas detectadas")
        eje.set(xlabel="Tiempo desde la salida (s)", ylabel="gy (°/s)")
        eje.grid(True, alpha=.3)
        eje.legend(fontsize=8, loc="upper right")
    ejes[0].set_xlim(0, cadencia["duracion_total_s"])
    for parcial in parciales[:-1]:
        ejes[0].axvline(parcial["tiempo_acumulado_s"], linestyle=":", color="purple", alpha=.5)
    ejes[0].set_title(f"Ensayo completo: {cadencia['numero_paladas']} referencias de palada detectadas")
    ejes[1].set_xlim(0, min(10, cadencia["duracion_total_s"]))
    ejes[1].set_title("Detalle de los primeros 10 s: comprobar omisiones y dobles detecciones")
    mascara = tiempo <= min(10, cadencia["duracion_total_s"])
    valores = datos_prueba.gy_dps.to_numpy()[mascara]
    margen = max(1., np.ptp(valores) * .15)
    ejes[1].set_ylim(np.min(valores) - margen, np.max(valores) + margen)
    figura.tight_layout()
    figura.savefig(ruta, dpi=180, bbox_inches="tight")
    plt.close(figura)
    return True
