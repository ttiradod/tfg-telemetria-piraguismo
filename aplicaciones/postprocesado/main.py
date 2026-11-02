"""Postprocesado de un ensayo recto: selección manual y llegada geométrica."""
from pathlib import Path

import numpy as np

from analisis import (ANCHO_META_M, MAX_INTERVALO_GNSS_S, pedir_distancia_nominal,
                      pedir_longitud_parcial, calcular_meta_virtual,
                      calcular_parciales_virtuales, calcular_resultados)
from cadencia import analizar_cadencia
from informe import crear_pdf, pedir_datos_informe
from lector_csv import buscar_archivos_csv, cargar_csv, comprobar_csv
from seleccion import seleccionar_zona, seleccionar_inicio, seleccionar_referencia, formatear_tiempo

CARPETA_PROGRAMA = Path(__file__).resolve().parent


def elegir_csv():
    archivos = buscar_archivos_csv()
    if not archivos:
        raise ValueError("No se encuentran CSV junto a main.py.")
    print("\nCSV disponibles:")
    for numero, ruta in enumerate(archivos, 1):
        print(f"{numero} - {ruta.name}")
    while True:
        try:
            opcion = int(input("Número del archivo: "))
            if 1 <= opcion <= len(archivos):
                return archivos[opcion - 1]
        except ValueError:
            pass
        print("Elige un número de la lista.")


def nueva_carpeta_resultados(archivo, distancia):
    nombre = f"{archivo.stem}_test_{distancia:g}m".replace(".", "p")
    raiz = CARPETA_PROGRAMA / "RESULTADOS"
    ruta = raiz / nombre
    numero = 2
    while ruta.exists():
        ruta = raiz / f"{nombre}_{numero}"
        numero += 1
    ruta.mkdir(parents=True)
    return ruta


def ejecutar():
    archivo = elegir_csv()
    datos = cargar_csv(archivo)
    if not comprobar_csv(datos):
        raise ValueError("El CSV no cumple las comprobaciones básicas.")
    print("Amplía una zona que contenga el ensayo completo y un pequeño margen tras la llegada.")
    print("El segundo clic solo orienta la pista; el límite derecho del zoom limita la búsqueda de cruces.")
    zona = seleccionar_zona(datos)
    if zona is None or len(zona) < 2:
        raise ValueError("No se ha seleccionado una zona con suficientes muestras.")
    indice_inicio, _ = seleccionar_inicio(datos, zona)
    if indice_inicio is None:
        print("Selección cancelada.")
        return
    indice_referencia, _ = seleccionar_referencia(datos, zona, indice_inicio)
    if indice_referencia is None:
        print("Selección cancelada.")
        return
    indice_zona_inicio, indice_zona_final = int(zona.index[0]), int(zona.index[-1])
    distancia = pedir_distancia_nominal()
    longitud = pedir_longitud_parcial(distancia)
    if not (0 <= indice_zona_inicio <= indice_inicio < indice_referencia <= indice_zona_final < len(datos)):
        raise ValueError("Las selecciones no están ordenadas dentro del CSV.")
    limite_us = float(datos.loc[indice_zona_final, "t_us"])
    meta = calcular_meta_virtual(datos, indice_inicio, indice_referencia, distancia, tiempo_limite_us=limite_us)
    if meta is None:
        raise ValueError("No se encuentra el cruce dentro de la zona. Revisa su margen final, la orientación y la trayectoria.")
    parciales = calcular_parciales_virtuales(datos, indice_inicio, indice_referencia, distancia,
                                            longitud, tiempo_limite_us=limite_us, meta_final=meta)
    inicio_us, final_us = float(datos.loc[indice_inicio, "t_us"]), meta["tiempo_cruce_us"]
    # IMU: solo muestras realmente adquiridas dentro del ensayo, sin añadir una fila futura.
    prueba = datos.loc[(datos.t_us >= inicio_us) & (datos.t_us <= final_us)].copy()
    if len(prueba) < 2:
        raise ValueError("El ensayo no contiene suficientes muestras.")
    if not np.isfinite(prueba[["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]]).all().all():
        raise ValueError("Faltan valores de IMU en el ensayo; revisa el CSV antes de generar resultados.")
    segundo_inicio = (inicio_us - float(datos.t_us.iloc[0])) / 1e6
    segundo_final = segundo_inicio + meta["tiempo_desde_salida_s"]
    resultados = calcular_resultados(prueba, distancia, segundo_inicio, segundo_final)
    prueba.attrs["duracion_s"] = resultados["duracion_s"]
    avisos = []
    if (prueba.sample_seq.diff().dropna() != 1).any():
        avisos.append("Hay muestras perdidas en el ensayo; revisa especialmente el recuento de paladas.")
    try:
        cadencia = analizar_cadencia(prueba, parciales, distancia, resultados["duracion_s"])
        avisos.extend(cadencia["avisos"])
    except ValueError as error:
        cadencia = None
        avisos.append(str(error))
    resultados["avisos"] = avisos
    print(f"\nSalida: {formatear_tiempo(segundo_inicio)} | Llegada: {formatear_tiempo(segundo_final)}")
    print(f"Tiempo: {resultados['duracion_s']:.3f} s | Media: {resultados['velocidad_media_mps'] * 3.6:.2f} km/h")
    for parcial in parciales:
        print(f"Parcial {parcial['numero']}: {parcial['desde_m']:g}-{parcial['hasta_m']:g} m | "
              f"{parcial['duracion_s']:.3f} s | acumulado {parcial['tiempo_acumulado_s']:.3f} s")
    if cadencia:
        print(f"Paladas detectadas: {cadencia['numero_paladas']}")
        if cadencia["cadencia_ppm"] is not None:
            print(f"Cadencia global: {cadencia['cadencia_ppm']:.0f} pal/min")
    for aviso in avisos:
        print("REVISAR:", aviso)
    datos_informe = pedir_datos_informe()
    if datos_informe["prueba"] == "No indicado":
        datos_informe["prueba"] = f"Test {distancia:g} m"
    carpeta = nueva_carpeta_resultados(archivo, distancia)
    ruta_pdf = crear_pdf(carpeta, archivo, prueba, distancia, resultados, parciales, cadencia, datos_informe, meta)
    print(f"\nInforme: {ruta_pdf}")
    print("Análisis completado.")


if __name__ == "__main__":
    try:
        ejecutar()
    except (ValueError, OSError, KeyError) as error:
        print(f"No se ha completado el análisis: {error}")
        raise SystemExit(1)
