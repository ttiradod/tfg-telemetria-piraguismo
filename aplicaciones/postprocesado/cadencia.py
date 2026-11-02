import numpy as np
from scipy.signal import find_peaks

# Parámetros originales, conservados para contrastar las detecciones con
# paladas contadas manualmente antes de realizar cualquier ajuste.
SUAVIZADO_S = 0.10
SEPARACION_MINIMA_S = 0.25



# ============================================================
# FRECUENCIA DE LA IMU
# ============================================================

def calcular_frecuencia_imu(datos_prueba):
    """Calcula la frecuencia real de muestreo a partir de t_us."""

    intervalos = (
        datos_prueba["t_us"]
        .diff()
        .dropna()
        / 1_000_000
    )

    if len(intervalos) == 0:
        return None

    intervalo_habitual = intervalos.median()

    if intervalo_habitual <= 0:
        return None

    return 1 / intervalo_habitual


# ============================================================
# SUAVIZADO DE gy_dps
# ============================================================

def suavizar_giroscopio(datos_prueba, frecuencia_imu):
    """
    Suaviza gy_dps con una media móvil de aproximadamente 0,1 s.

    gy_dps se utiliza porque en las pruebas presenta un máximo positivo
    claro y repetible dentro de cada ciclo de palada.
    """

    senal = (
        datos_prueba["gy_dps"]
        .astype(float)
    )

    ventana = max(
        1,
        round(frecuencia_imu * SUAVIZADO_S),
    )

    return (
        senal
        .rolling(
            window=ventana,
            center=True,
            min_periods=1,
        )
        .mean()
        .to_numpy()
    )


# ============================================================
# DETECCIÓN DE PALADAS
# ============================================================

def detectar_paladas(senal_suave, frecuencia_imu):
    """
    Detecta una referencia temporal por cada ciclo de palada.

    Se buscan máximos positivos de gy_dps suavizada, imponiendo una
    separación mínima de 0,25 s y una prominencia mínima igual a la
    desviación típica de la señal.
    """

    separacion_minima = max(
        1,
        round(frecuencia_imu * SEPARACION_MINIMA_S),
    )

    prominencia_minima = np.std(senal_suave)

    picos, _ = find_peaks(
        senal_suave,
        distance=separacion_minima,
        prominence=prominencia_minima,
        height=0,
    )

    return picos



# ============================================================
# RESULTADOS POR PARCIAL
# ============================================================

def calcular_cadencia_por_parcial(
    tiempos_paladas,
    parciales,
):
    resultados = []
    inicio_parcial = 0.0

    for posicion, parcial in enumerate(parciales):
        final_parcial = parcial["tiempo_acumulado_s"]

        if posicion < len(parciales) - 1:
            dentro = (
                (tiempos_paladas >= inicio_parcial)
                & (tiempos_paladas < final_parcial)
            )
        else:
            dentro = (
                (tiempos_paladas >= inicio_parcial)
                & (tiempos_paladas <= final_parcial)
            )

        numero_paladas = int(np.sum(dentro))
        duracion = parcial["duracion_s"]
        distancia_parcial = (
            parcial["hasta_m"]
            - parcial["desde_m"]
        )

        if numero_paladas > 0 and duracion > 0:
            cadencia_media = (
                numero_paladas
                / duracion
                * 60
            )
            avance = (
                distancia_parcial
                / numero_paladas
            )
        else:
            cadencia_media = None
            avance = None

        resultados.append(
            {
                "numero": parcial["numero"],
                "paladas": numero_paladas,
                "cadencia_ppm": cadencia_media,
                "avance_m_palada": avance,
            }
        )

        inicio_parcial = final_parcial

    return resultados


# ============================================================
# ANÁLISIS COMPLETO DE CADENCIA
# ============================================================

def analizar_cadencia(
    datos_prueba,
    parciales,
    distancia_total,
    duracion_total_s=None,
):
    frecuencia_imu = calcular_frecuencia_imu(
        datos_prueba
    )

    if frecuencia_imu is None:
        raise ValueError("Faltan muestras para calcular la frecuencia de la IMU.")
    if not np.isfinite(datos_prueba["gy_dps"]).all():
        raise ValueError("La señal gy_dps contiene datos ausentes o no finitos; no se inventan paladas.")
    intervalos = datos_prueba["t_us"].diff().dropna().to_numpy() / 1e6
    if np.any(intervalos <= 0) or np.any(intervalos > 2.5 / frecuencia_imu):
        raise ValueError("Hay un hueco de muestreo en la IMU; revisa la cadencia de este tramo.")

    senal_suave = suavizar_giroscopio(
        datos_prueba,
        frecuencia_imu,
    )

    picos = detectar_paladas(
        senal_suave,
        frecuencia_imu,
    )

    tiempo = (
        datos_prueba["t_us"].to_numpy()
        - datos_prueba["t_us"].iloc[0]
    ) / 1_000_000

    # El denominador global es la duración salida-meta, nunca el último parcial.
    duracion_total = float(tiempo[-1] if duracion_total_s is None else duracion_total_s)
    if not np.isfinite(duracion_total) or duracion_total <= 0:
        raise ValueError("La duración global debe ser positiva.")
    picos = picos[tiempo[picos] <= duracion_total]
    tiempos_paladas = tiempo[picos]
    numero_paladas = len(picos)

    if numero_paladas > 0 and duracion_total > 0:
        cadencia_media = (
            numero_paladas
            / duracion_total
            * 60
        )
        avance_medio = (
            distancia_total
            / numero_paladas
        )
    else:
        cadencia_media = None
        avance_medio = None

    resultados_parciales = calcular_cadencia_por_parcial(tiempos_paladas, parciales)
    avisos = [f"Parcial {p['numero']}: cero paladas detectadas; revisar el detector."
              for p in resultados_parciales if p["paladas"] == 0]
    if numero_paladas == 0:
        avisos.insert(0, "No se han detectado paladas en el ensayo; la cadencia no está validada.")
    return {
        "numero_paladas": numero_paladas,
        "cadencia_ppm": cadencia_media,
        "avance_m_palada": avance_medio,
        "parciales": resultados_parciales,
        "avisos": avisos,
        "duracion_total_s": duracion_total,
        "frecuencia_imu_hz": float(frecuencia_imu),
        "tiempos_paladas_s": tiempos_paladas.tolist(),
        "indices_picos": picos.tolist(),
        "senal_suave": senal_suave.tolist(),
        "parametros": {"suavizado_s": SUAVIZADO_S, "separacion_minima_s": SEPARACION_MINIMA_S,
                       "prominencia_dps": float(np.std(senal_suave))},
    }
