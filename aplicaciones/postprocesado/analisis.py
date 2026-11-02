"""Pista recta virtual: orientación, intersecciones y resultados en unidades SI."""
import math

import numpy as np
import pandas as pd
import utm
from geographiclib.geodesic import Geodesic

ANCHO_META_M = 10.0
# Límite de continuidad, no estimación de precisión. A 10 Hz equivale a
# permitir hasta cinco periodos nominales entre las posiciones del cruce.
MAX_INTERVALO_GNSS_S = 0.5
LONGITUDES_PARCIAL_M = (25, 50, 100, 250)


def pedir_distancia_nominal():
    while True:
        try:
            distancia = float(input("Distancia de la prueba en metros: ").replace(",", "."))
            if math.isfinite(distancia) and distancia > 0 and parciales_permitidos(distancia):
                return distancia
        except ValueError:
            pass
        print("Introduce una distancia positiva divisible por 25, 50, 100 o 250 m.")


def parciales_permitidos(distancia_total):
    if not math.isfinite(distancia_total) or distancia_total <= 0:
        return []
    return [p for p in LONGITUDES_PARCIAL_M if p <= distancia_total and
            math.isclose(distancia_total / p, round(distancia_total / p), abs_tol=1e-9, rel_tol=0)]


def pedir_longitud_parcial(distancia_total):
    opciones = parciales_permitidos(distancia_total)
    if not opciones:
        raise ValueError("La distancia no admite los parciales configurados.")
    while True:
        try:
            longitud = float(input(f"Longitud del parcial {opciones} m: ").replace(",", "."))
            if longitud in opciones:
                return longitud
        except ValueError:
            pass
        print("Elige una de las longitudes indicadas; todos los parciales serán iguales.")


def _gps_validos(datos):
    columnas = ["t_us", "speed_2d_mps", "lat_deg", "lon_deg"]
    filas = ((datos.gps_new == 1) & (datos.gps_valid == 1) &
             np.isfinite(datos[columnas]).all(axis=1))
    return datos.loc[filas, columnas].copy()


def _interpolar_gnss(gps, tiempo_us):
    anteriores = gps.loc[gps.t_us <= tiempo_us]
    posteriores = gps.loc[gps.t_us >= tiempo_us]
    if anteriores.empty or posteriores.empty:
        raise ValueError("Faltan posiciones GNSS a ambos lados de la selección.")
    anterior, posterior = anteriores.iloc[-1], posteriores.iloc[0]
    intervalo = float(posterior.t_us - anterior.t_us)
    if intervalo > MAX_INTERVALO_GNSS_S * 1e6:
        raise ValueError("La selección cae en un hueco GNSS superior a 0,5 s.")
    fraccion = 0.0 if intervalo == 0 else (tiempo_us - anterior.t_us) / intervalo
    resultado = {"t_us": float(tiempo_us)}
    for columna in ("speed_2d_mps", "lat_deg", "lon_deg"):
        resultado[columna] = float(anterior[columna] + fraccion * (posterior[columna] - anterior[columna]))
    return resultado


def _interseccion_segmentos(p1, p2, p3, p4):
    """t recorre la trayectoria; u recorre la meta. Ambos deben estar en [0,1]."""
    x1, y1 = p1
    dx1, dy1 = p2[0] - x1, p2[1] - y1
    dx2, dy2 = p4[0] - p3[0], p4[1] - p3[1]
    dx3, dy3 = p3[0] - x1, p3[1] - y1
    denominador = dx1 * dy2 - dy1 * dx2
    if abs(denominador) < 1e-9:
        return None
    t = (dx3 * dy2 - dy3 * dx2) / denominador
    u = (dx3 * dy1 - dy3 * dx1) / denominador
    if not (0 <= t <= 1 and 0 <= u <= 1):
        return None
    return {"t": float(t), "u": float(u), "x_m": float(x1 + t * dx1), "y_m": float(y1 + t * dy1)}


def calcular_meta_virtual(datos, indice_inicio, indice_referencia, distancia_meta_m,
                          ancho_meta_m=ANCHO_META_M, tiempo_limite_us=None):
    """Primer cruce hacia delante dentro de la zona, sin usar el clic de referencia como final.

    La geometría conserva el método original: WGS84 para construir la pista,
    UTM para intersectar segmentos e interpolación lineal del tiempo del cruce.
    """
    if not math.isfinite(distancia_meta_m) or distancia_meta_m <= 0:
        raise ValueError("La distancia de meta debe ser positiva y finita.")
    if not math.isfinite(ancho_meta_m) or ancho_meta_m <= 0:
        raise ValueError("La anchura de meta debe ser positiva y finita.")
    inicio_us = float(datos.loc[indice_inicio, "t_us"])
    referencia_us = float(datos.loc[indice_referencia, "t_us"])
    limite_us = float(datos.t_us.iloc[-1] if tiempo_limite_us is None else tiempo_limite_us)
    if not inicio_us < referencia_us <= limite_us:
        raise ValueError("La referencia debe estar después de la salida y dentro de la zona.")
    gps = _gps_validos(datos)
    if len(gps) < 2:
        raise ValueError("No hay suficientes posiciones GNSS válidas.")
    inicio = _interpolar_gnss(gps, inicio_us)
    referencia = _interpolar_gnss(gps, referencia_us)
    orientacion = Geodesic.WGS84.Inverse(inicio["lat_deg"], inicio["lon_deg"],
                                          referencia["lat_deg"], referencia["lon_deg"])
    if orientacion["s12"] < 1.0:
        raise ValueError("La referencia está a menos de 1 m de la salida; elige un punto avanzado del recorrido.")
    azimut = orientacion["azi1"]
    centro = Geodesic.WGS84.Direct(inicio["lat_deg"], inicio["lon_deg"], azimut, distancia_meta_m)
    a = Geodesic.WGS84.Direct(centro["lat2"], centro["lon2"], centro["azi2"] - 90, ancho_meta_m / 2)
    b = Geodesic.WGS84.Direct(centro["lat2"], centro["lon2"], centro["azi2"] + 90, ancho_meta_m / 2)
    este0, norte0, zona, letra = utm.from_latlon(inicio["lat_deg"], inicio["lon_deg"])

    def coordenadas(lat, lon):
        # Una única zona UTM para todos los puntos de la misma pista.
        este, norte, _, _ = utm.from_latlon(lat, lon, force_zone_number=zona, force_zone_letter=letra)
        return este - este0, norte - norte0

    pa = coordenadas(a["lat2"], a["lon2"])
    pb = coordenadas(b["lat2"], b["lon2"])
    centro_xy = coordenadas(centro["lat2"], centro["lon2"])
    anterior = {"xy": (0.0, 0.0), "t_us": inicio_us, "indice": int(indice_inicio)}
    posteriores = gps.loc[gps.t_us > inicio_us]
    for indice, fila in posteriores.iterrows():
        actual = {"xy": coordenadas(float(fila.lat_deg), float(fila.lon_deg)),
                  "t_us": float(fila.t_us), "indice": int(indice)}
        cruce = _interseccion_segmentos(anterior["xy"], actual["xy"], pa, pb)
        # Proyección positiva del desplazamiento: no aceptar un regreso.
        avance = sum((actual["xy"][j] - anterior["xy"][j]) * centro_xy[j] for j in (0, 1))
        if cruce is not None and avance > 0:
            tiempo = anterior["t_us"] + cruce["t"] * (actual["t_us"] - anterior["t_us"])
            if tiempo <= limite_us:
                intervalo = (actual["t_us"] - anterior["t_us"]) / 1e6
                if intervalo > MAX_INTERVALO_GNSS_S:
                    raise ValueError("El cruce cae en un hueco GNSS superior a 0,5 s; no se estima la llegada.")
                lat, lon = utm.to_latlon(este0 + cruce["x_m"], norte0 + cruce["y_m"], zona, letra)
                return dict(cruce, distancia_meta_m=float(distancia_meta_m), ancho_meta_m=float(ancho_meta_m),
                            azimut_deg=float(azimut), lat_inicio=inicio["lat_deg"], lon_inicio=inicio["lon_deg"],
                            lat_referencia=referencia["lat_deg"], lon_referencia=referencia["lon_deg"],
                            lat_centro=centro["lat2"], lon_centro=centro["lon2"],
                            lat_a=a["lat2"], lon_a=a["lon2"], lat_b=b["lat2"], lon_b=b["lon2"],
                            tiempo_inicio_us=inicio_us, tiempo_limite_us=limite_us,
                            tiempo_cruce_us=float(tiempo), tiempo_desde_salida_s=float((tiempo - inicio_us) / 1e6),
                            lat_cruce=float(lat), lon_cruce=float(lon), intervalo_cruce_s=float(intervalo),
                            indice_gnss_anterior=anterior["indice"], indice_gnss_posterior=actual["indice"],
                            velocidad_inicio_mps=inicio["speed_2d_mps"],
                            velocidad_cruce_mps=float(np.interp(tiempo, gps.t_us, gps.speed_2d_mps)))
        if actual["t_us"] >= limite_us:
            break
        anterior = actual
    return None


def calcular_parciales_virtuales(datos, indice_inicio, indice_referencia, distancia_total,
                                 longitud_parcial, tiempo_limite_us=None, meta_final=None):
    if longitud_parcial not in parciales_permitidos(distancia_total):
        raise ValueError("El parcial debe ser 25, 50, 100 o 250 m y dividir exactamente la prueba.")
    parciales = []
    tiempo_anterior = 0.0
    indice_anterior = int(indice_inicio)
    inicio_us = float(datos.loc[indice_inicio, "t_us"])
    for numero in range(1, round(distancia_total / longitud_parcial) + 1):
        distancia = float(numero * longitud_parcial)
        meta = meta_final if math.isclose(distancia, distancia_total) and meta_final else calcular_meta_virtual(
            datos, indice_inicio, indice_referencia, distancia, tiempo_limite_us=tiempo_limite_us)
        if meta is None:
            raise ValueError(f"No se ha encontrado el cruce de {distancia:g} m dentro de la zona.")
        acumulado = meta["tiempo_desde_salida_s"]
        duracion = acumulado - tiempo_anterior
        if duracion <= 0:
            raise ValueError("Los cruces de parciales no siguen un orden temporal creciente; revisa la pista.")
        indice_final = int((datos.t_us - meta["tiempo_cruce_us"]).abs().idxmin())
        tramo = datos.loc[(datos.t_us >= inicio_us + tiempo_anterior * 1e6) &
                          (datos.t_us <= meta["tiempo_cruce_us"])]
        parciales.append(dict(numero=numero, desde_m=distancia - longitud_parcial, hasta_m=distancia,
                              distancia_m=float(longitud_parcial), duracion_s=float(duracion),
                              tiempo_acumulado_s=float(acumulado), velocidad_media_mps=longitud_parcial / duracion,
                              velocidad_maxima_1s_mps=calcular_vmax_1s(tramo), indice_inicio=indice_anterior,
                              indice_final=indice_final, meta=meta))
        tiempo_anterior, indice_anterior = acumulado, indice_final
    return parciales


def medias_velocidad_1s(datos_gnss):
    """Media por muestras equivalente a ~1 s a la cadencia GNSS habitual.

    Se conserva el cálculo original; no se mezclan ventanas a través de
    huecos mayores que el límite de continuidad.
    """
    salida = pd.Series(np.nan, index=datos_gnss.index, dtype=float)
    if len(datos_gnss) < 2:
        return salida
    dt = datos_gnss.t_us.diff() / 1e6
    habitual = dt.median()
    if not np.isfinite(habitual) or habitual <= 0:
        return salida
    muestras = max(1, round(1 / habitual))
    grupos = (dt > MAX_INTERVALO_GNSS_S).cumsum()
    for _, bloque in datos_gnss.groupby(grupos):
        salida.loc[bloque.index] = bloque.speed_2d_mps.rolling(muestras, min_periods=muestras).mean()
    return salida


def calcular_vmax_1s(datos_tramo):
    gps = datos_tramo.loc[(datos_tramo.gps_new == 1) & (datos_tramo.gps_valid == 1) &
                          np.isfinite(datos_tramo.speed_2d_mps), ["t_us", "speed_2d_mps"]]
    medias = medias_velocidad_1s(gps)
    return float(medias.max()) if medias.notna().any() else None


def calcular_resultados(datos_prueba, distancia_m, segundo_inicio, segundo_final):
    duracion = segundo_final - segundo_inicio
    if not math.isfinite(duracion) or duracion <= 0:
        raise ValueError("La duración completa del ensayo debe ser positiva.")
    velocidades = datos_prueba.loc[(datos_prueba.gps_new == 1) & (datos_prueba.gps_valid == 1) &
                                   np.isfinite(datos_prueba.speed_2d_mps), "speed_2d_mps"]
    return dict(duracion_s=float(duracion), distancia_m=float(distancia_m), velocidad_media_mps=distancia_m / duracion,
                velocidad_maxima_mps=float(velocidades.max()) if len(velocidades) else None,
                velocidad_maxima_1s_mps=calcular_vmax_1s(datos_prueba))
