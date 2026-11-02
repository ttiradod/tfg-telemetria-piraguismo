"""Informe deportivo: presentación original, con cálculos revisados por separado."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.sax.saxutils import escape

from matplotlib import get_data_path
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, Image, NextPageTemplate,
                               PageBreak, PageTemplate, Paragraph, Spacer, Table, TableStyle)

from graficas import (guardar_acelerometro, guardar_giroscopio, guardar_trayectoria,
                      guardar_velocidad)

AZUL = colors.HexColor("#17365D")
FUENTES = Path(get_data_path()) / "fonts" / "ttf"
for nombre, archivo in (("DejaVuSans", "DejaVuSans.ttf"), ("DejaVuSans-Bold", "DejaVuSans-Bold.ttf")):
    pdfmetrics.registerFont(TTFont(nombre, str(FUENTES / archivo)))


def pedir_datos_informe():
    print("\nDatos de portada: Enter deja el campo como 'No indicado'.")
    preguntas = dict(deportista="Nombre de la deportista", categoria="Categoría", embarcacion="Embarcación",
                     prueba="Nombre o tipo de prueba", lugar="Lugar", temperatura="Temperatura ambiente (°C)",
                     direccion_viento="Dirección de procedencia del viento (N, NE, E...)",
                     velocidad_viento="Velocidad del viento (km/h)")
    datos = {}
    for clave, pregunta in preguntas.items():
        while True:
            texto = input(pregunta + ": ").strip()
            if not texto:
                datos[clave] = "No indicado"
                break
            if clave not in ("temperatura", "velocidad_viento"):
                datos[clave] = texto
                break
            try:
                numero = float(texto.replace(",", "."))
                if not math.isfinite(numero) or (clave == "velocidad_viento" and numero < 0):
                    raise ValueError
                datos[clave] = f"{numero:g} {'°C' if clave == 'temperatura' else 'km/h'}"
                break
            except ValueError:
                print("Introduce un número finito válido.")
    return datos


def texto_decimal(valor, decimales=2, unidad=""):
    if valor is None or not math.isfinite(float(valor)):
        return "No disponible"

    numero = float(valor)

    if decimales == 0:
        texto_numero = str(math.floor(numero + 0.5))
    else:
        texto_numero = f"{numero:.{decimales}f}"

    return texto_numero + (f" {unidad}" if unidad else "")

def imagen_ajustada(ruta, ancho, alto):
    imagen = Image(str(ruta))
    escala = min(ancho / imagen.imageWidth, alto / imagen.imageHeight)
    imagen.drawWidth, imagen.drawHeight = imagen.imageWidth * escala, imagen.imageHeight * escala
    return imagen


def agregar_pie_pagina(lienzo, documento):
    lienzo.saveState()
    lienzo.setFont("DejaVuSans", 7)
    lienzo.setFillColor(colors.HexColor("#666666"))
    lienzo.drawString(1.5 * cm, 0.7 * cm, "Trabajo Fin de Grado · Universidad de Extremadura")
    lienzo.drawRightString(lienzo._pagesize[0] - 1.5 * cm, 0.7 * cm, str(documento.page))
    lienzo.restoreState()


def tabla_formateada(filas, anchos, tamano=8):
    tabla = Table(filas, colWidths=anchos, repeatRows=1, hAlign="LEFT")
    tabla.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), AZUL), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "DejaVuSans-Bold"),
        ("FONTNAME", (0, 1), (-1, -1), "DejaVuSans"), ("FONTSIZE", (0, 0), (-1, -1), tamano),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F2F2F2")]),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"), ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
    ]))
    return tabla


def crear_tabla_parciales(parciales, cadencia):
    """Bloques de hasta cuatro parciales para mantener legibles todas las métricas."""
    if not parciales:
        raise ValueError("Faltan parciales; revisa su longitud y los cruces antes de generar el informe.")
    por_numero = {} if cadencia is None else {p["numero"]: p for p in cadencia["parciales"]}
    filas = [["Métrica"] + [f"Parcial {p['numero']}\n{p['desde_m']:g}-{p['hasta_m']:g} m" for p in parciales]]
    for nombre, clave, decimales, unidad, factor in (
        ("Tiempo del parcial", "duracion_s", 3, "s", 1),
        ("Tiempo acumulado", "tiempo_acumulado_s", 3, "s", 1),
        ("Velocidad media", "velocidad_media_mps", 2, "km/h", 3.6),
        ("Velocidad máxima 1 s", "velocidad_maxima_1s_mps", 2, "km/h", 3.6),
    ):
        filas.append([nombre] + [texto_decimal(None if p.get(clave) is None else p[clave] * factor,
                                               decimales, unidad) for p in parciales])
    for nombre, clave, decimales, unidad in (
        ("Paladas del parcial", "paladas", 0, ""), ("Cadencia media", "cadencia_ppm", 0, "pal/min"),
        
        ("Avance medio por palada", "avance_m_palada", 2, "m/palada"),
    ):
        filas.append([nombre] + [texto_decimal(por_numero.get(p["numero"], {}).get(clave), decimales, unidad)
                                 for p in parciales])
    acumuladas = []
    for p in parciales:
        anteriores = [por_numero.get(n) for n in range(1, p["numero"] + 1)]
        acumuladas.append(str(sum(x["paladas"] for x in anteriores)) if all(x is not None for x in anteriores)
                          else "No disponible")
    filas.insert(6, ["Paladas acumuladas"] + acumuladas)
    ancho = 17.5 * cm
    return tabla_formateada(filas, [4.8 * cm] + [(ancho - 4.8 * cm) / len(parciales)] * len(parciales), 7.5)


def crear_pdf(carpeta_resultados, archivo_csv, datos_prueba, distancia_m, resultados,
              parciales, cadencia, datos_informe=None, meta=None):
    if not parciales or meta is None:
        raise ValueError("No se genera un informe sin pista y parciales completos.")
    carpeta = Path(carpeta_resultados)
    carpeta.mkdir(parents=True, exist_ok=True)
    ruta_pdf = carpeta / "informe_postprocesado.pdf"
    datos_informe = pedir_datos_informe() if datos_informe is None else datos_informe
    estilos = getSampleStyleSheet()
    for nombre in ("Normal", "Title", "Heading2"):
        estilos[nombre].fontName = "DejaVuSans-Bold" if nombre != "Normal" else "DejaVuSans"
    estilos["Normal"].fontSize = 8.5
    estilos["Normal"].leading = 12
    estilos["Title"].fontSize = 15
    estilos["Title"].leading = 19
    estilos["Title"].spaceAfter = 16
    estilos["Heading2"].fontSize = 13
    estilos["Heading2"].leading = 17
    estilos.add(ParagraphStyle(name="Nota", parent=estilos["Normal"], fontSize=8, leading=11, spaceAfter=6))
    contenido = []

    def texto(valor, estilo="Normal"):
        return Paragraph(escape(str(valor)), estilos[estilo])

    def pagina(titulo):
        if contenido:
            contenido.append(PageBreak())
        contenido.append(texto(titulo, "Heading2"))
        contenido.append(Spacer(1, 8))

    documento = BaseDocTemplate(str(ruta_pdf), pagesize=A4, title="Análisis GNSS e IMU de piragüismo",
                               author="Teresa Tirado Dotor")
    plantillas = []
    for nombre, (ancho, alto) in (("Vertical", A4), ("Horizontal", landscape(A4))):
        marco = Frame(1.5 * cm, 1.3 * cm, ancho - 3 * cm, alto - 2.8 * cm,
                      leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
        plantillas.append(PageTemplate(id=nombre, pagesize=(ancho, alto), frames=[marco], onPage=agregar_pie_pagina))
    documento.addPageTemplates(plantillas)
    # Los auxiliares viven en una carpeta temporal: nunca se borran archivos del usuario.
    with TemporaryDirectory(prefix="graficas_", dir=carpeta) as temporal:
        temporal = Path(temporal)
        rutas = {n: temporal / f"{n}.png" for n in ("trayectoria", "velocidad", "acelerometro", "giroscopio")}
        guardar_trayectoria(datos_prueba, datos_informe, rutas["trayectoria"])
        guardar_velocidad(datos_prueba, parciales, meta, distancia_m, rutas["velocidad"])
        guardar_acelerometro(datos_prueba, parciales, rutas["acelerometro"])
        guardar_giroscopio(datos_prueba, parciales, rutas["giroscopio"])
        contenido.append(texto("INFORME DE ANÁLISIS DEL RENDIMIENTO EN PIRAGÜISMO", "Title"))
        contenido.append(texto("Postprocesado de datos GNSS e IMU"))
        contenido.append(Spacer(1, 18))
        def dato(clave):
            return texto(datos_informe.get(clave, "No indicado"))
        def utc(clave):
            validos = datos_prueba[clave].dropna()
            return texto(validos.iloc[0] if len(validos) else "No disponible")
        filas = [
            ["Deportista", dato("deportista"), "Categoría", dato("categoria")],
            ["Embarcación", dato("embarcacion"), "Prueba", dato("prueba")],
            ["Distancia", f"{distancia_m:.1f} m", "Lugar", dato("lugar")],
            ["Fecha UTC", utc("utc_date"), "Hora UTC", utc("utc_time")],
            ["Temperatura", dato("temperatura"), "Viento", texto(datos_informe.get("direccion_viento", "No indicado") +
                                                                  " / " + datos_informe.get("velocidad_viento", "No indicado"))],
            ["Archivo", texto(Path(archivo_csv).name), "", ""],
        ]
        portada = Table(filas, colWidths=[3.0 * cm, 5.5 * cm, 3.0 * cm, 6.0 * cm], hAlign="LEFT")
        portada.setStyle(TableStyle([
            ("FONTNAME", (0, 0), (-1, -1), "DejaVuSans"), ("FONTSIZE", (0, 0), (-1, -1), 8),
            ("GRID", (0, 0), (-1, -1), .4, colors.grey), ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#F2F2F2")),
            ("BACKGROUND", (2, 0), (2, -1), colors.HexColor("#F2F2F2")),
            ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
        ]))
        contenido.append(portada)
        contenido.append(Spacer(1, 30))
        contenido.append(texto("Trayectoria GNSS", "Heading2"))
        contenido.append(Spacer(1, 10))
        contenido.append(imagen_ajustada(rutas["trayectoria"], 17.5 * cm, 10.5 * cm))
        pagina("Velocidad y parciales")
        contenido.append(imagen_ajustada(rutas["velocidad"], 17.5 * cm, 10 * cm))
        contenido.append(Spacer(1, 24))
        for inicio in range(0, len(parciales), 4):
            if inicio:
                pagina("Parciales (continuación)")
            contenido.append(crear_tabla_parciales(parciales[inicio:inicio + 4], cadencia))
            contenido.append(Spacer(1, 10))
        pagina("Resultados principales")
        c = {} if cadencia is None else cadencia
        filas = [["Métrica", "Resultado"]]
        for etiqueta, valor, decimales, unidad in (
            ("Tiempo", resultados["duracion_s"], 3, "s"),
            ("Distancia", distancia_m, 1, "m"),
            ("Velocidad media", resultados["velocidad_media_mps"] * 3.6, 2, "km/h"),
            ("Velocidad máxima GNSS", None if resultados["velocidad_maxima_mps"] is None else resultados["velocidad_maxima_mps"] * 3.6, 2, "km/h"),
            ("Velocidad máxima GNSS 1 s", None if resultados["velocidad_maxima_1s_mps"] is None else resultados["velocidad_maxima_1s_mps"] * 3.6, 2, "km/h"),
            ("Cadencia media", c.get("cadencia_ppm"), 0, "paladas/min"),
            ("Paladas", c.get("numero_paladas"), 0, ""),
            ("Avance medio por palada", c.get("avance_m_palada"), 2, "m/palada"),
        ):
            filas.append([etiqueta, texto_decimal(valor, decimales, unidad)])
        tabla_resultados = tabla_formateada(filas, [10 * cm, 7.5 * cm], 9)
        tabla_resultados.setStyle(TableStyle([("TOPPADDING", (0, 0), (-1, -1), 12),
                                             ("BOTTOMPADDING", (0, 0), (-1, -1), 12)]))
        contenido.append(Spacer(1, 14))
        contenido.append(tabla_resultados)
        contenido.append(Spacer(1, 12))
        contenido.append(NextPageTemplate("Horizontal"))
        for clave, titulo in (("acelerometro", "Acelerómetro"), ("giroscopio", "Giroscopio")):
            pagina(titulo)
            if rutas[clave].exists():
                contenido.append(imagen_ajustada(rutas[clave], 26.5 * cm, 15.2 * cm))
            else:
                contenido.append(texto("No se ha podido calcular la cadencia. Consulta los avisos de resultados."))
        documento.build(contenido)
    return ruta_pdf
