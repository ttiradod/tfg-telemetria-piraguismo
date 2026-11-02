"""Lectura del formato común de los cuatro modos, sin reordenar ni rellenar muestras."""
from pathlib import Path

import numpy as np
import pandas as pd

COLUMNAS_NECESARIAS = [
    "sample_seq", "t_us", "utc_date", "utc_time", "test_id", "event", "event_source",
    "gps_new", "gps_valid", "satellites", "hdop", "lat_deg", "lon_deg", "speed_2d_mps",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
]


def buscar_archivos_csv(carpeta=None):
    carpeta = Path(__file__).resolve().parent if carpeta is None else Path(carpeta)
    return sorted((p for p in carpeta.iterdir() if p.is_file() and p.suffix.lower() == ".csv"),
                  key=lambda p: p.name.lower())


def cargar_csv(ruta_archivo):
    # Mantener UTC como texto conserva los ceros iniciales y su formato original.
    return pd.read_csv(ruta_archivo, comment="#", low_memory=False,
                       dtype={"utc_date": "string", "utc_time": "string", "event": "string", "event_source": "string"})


def comprobar_csv(datos):
    ausentes = [c for c in COLUMNAS_NECESARIAS if c not in datos]
    if ausentes or len(datos) < 2:
        print("ERROR: faltan columnas: " + ", ".join(ausentes) if ausentes else "ERROR: faltan muestras.")
        return False
    numericas = [c for c in COLUMNAS_NECESARIAS if c not in ("utc_date", "utc_time", "event", "event_source")]
    for columna in numericas:
        original = datos[columna]
        convertida = pd.to_numeric(original, errors="coerce")
        if (original.notna() & convertida.isna()).any():
            print(f"ERROR: texto no numérico en {columna}.")
            return False
        datos[columna] = convertida
    obligatorias = ["sample_seq", "t_us", "gps_new", "gps_valid"]
    if not np.isfinite(datos[obligatorias]).all().all():
        print("ERROR: faltan tiempos, secuencias o indicadores GNSS.")
        return False
    if (datos.t_us < 0).any() or (datos.t_us.diff().dropna() <= 0).any():
        print("ERROR: t_us debe ser creciente; no se mezclan reinicios en un análisis.")
        return False
    if (datos.sample_seq % 1 != 0).any() or (datos.sample_seq.diff().dropna() <= 0).any():
        print("ERROR: sample_seq debe ser entero y creciente.")
        return False
    if not datos.gps_new.isin([0, 1]).all() or not datos.gps_valid.isin([0, 1]).all():
        print("ERROR: gps_new y gps_valid solo admiten 0 o 1.")
        return False
    validas = (datos.gps_new == 1) & (datos.gps_valid == 1)
    coordenadas = datos.loc[validas, ["lat_deg", "lon_deg", "speed_2d_mps"]]
    if (np.isinf(coordenadas).any().any() or
            not coordenadas.lat_deg.dropna().between(-80, 84).all() or
            not coordenadas.lon_deg.dropna().between(-180, 180).all() or (coordenadas.speed_2d_mps < 0).any()):
        print("ERROR: posición o velocidad GNSS válida incompatible con el análisis UTM.")
        return False
    if coordenadas.isna().any().any():
        print("AVISO: hay actualizaciones GNSS incompletas; no se usarán para interpolar posiciones.")
    if (datos.sample_seq.diff().dropna() != 1).any():
        print("AVISO: hay saltos de secuencia. Se comprobará la continuidad del tramo elegido.")
    print(f"CSV cargado correctamente: {len(datos)} muestras.")
    return True
