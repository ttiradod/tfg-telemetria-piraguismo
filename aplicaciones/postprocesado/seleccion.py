import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import FuncFormatter, AutoMinorLocator, MaxNLocator



# Matplotlib usa por defecto ← y → para navegar por el historial de vistas.
# Las liberamos para que en este programa sirvan exclusivamente para mover
# la selección una muestra IMU hacia atrás o hacia delante.
mpl.rcParams["keymap.back"] = [
    tecla for tecla in mpl.rcParams["keymap.back"]
    if tecla != "left"
]
mpl.rcParams["keymap.forward"] = [
    tecla for tecla in mpl.rcParams["keymap.forward"]
    if tecla != "right"
]


# ============================================================
# FORMATO DE TIEMPO
# ============================================================

def formatear_tiempo(segundos):
    """Convierte segundos desde el inicio del registro a mm:ss.mmm."""
    minutos = int(segundos // 60)
    resto = segundos - minutos * 60
    return f"{minutos:02d}:{resto:06.3f}"


def _formateador_tiempo(segundos, _posicion):
    return formatear_tiempo(segundos)


def _tiempo_desde_inicio(datos, datos_vista):
    t0 = datos["t_us"].iloc[0]
    return (datos_vista["t_us"] - t0) / 1_000_000



# ============================================================
# DIBUJAR LAS CUATRO SEÑALES
# ============================================================

def _dibujar_cuatro_senales(
    datos,
    datos_vista,
    titulo,
):
    tiempo = _tiempo_desde_inicio(
        datos,
        datos_vista,
    )

    filas_gps = (
        (datos_vista["gps_new"] == 1)
        & (datos_vista["gps_valid"] == 1)
        & datos_vista["speed_2d_mps"].notna()
    )

    tiempo_gps = tiempo[filas_gps]
    velocidad_kmh = (
        datos_vista.loc[
            filas_gps,
            "speed_2d_mps",
        ]
        * 3.6
    )

    figura, graficas = plt.subplots(
        4,
        1,
        figsize=(15, 10),
        sharex=True,
    )

    graficas[0].plot(
        tiempo_gps,
        velocidad_kmh,
        linewidth=1,
    )
    graficas[0].set_ylabel(
        "Velocidad\n(km/h)"
    )

    graficas[1].plot(
        tiempo,
        datos_vista["ax_g"],
        linewidth=1,
        color="tab:red",
    )
    graficas[1].axhline(
        0,
        color="black",
        linewidth=0.7,
        linestyle="--",
        alpha=0.5,
    )
    graficas[1].set_ylabel(
        "ax_g\n(g)"
    )

    graficas[2].plot(
        tiempo,
        datos_vista["gy_dps"],
        linewidth=1,
        color="tab:brown",
    )
    graficas[2].axhline(
        0,
        color="black",
        linewidth=0.7,
        linestyle="--",
        alpha=0.5,
    )
    graficas[2].set_ylabel(
        "gy_dps\n(°/s)"
    )

    graficas[3].plot(
        tiempo,
        datos_vista["gz_dps"],
        linewidth=1,
        color="tab:green",
    )
    graficas[3].axhline(
        0,
        color="black",
        linewidth=0.7,
        linestyle="--",
        alpha=0.5,
    )
    graficas[3].set_ylabel(
        "gz_dps\n(°/s)"
    )
    graficas[3].set_xlabel(
        "Tiempo desde el inicio del registro (mm:ss.mmm)"
    )


    for grafica in graficas:
        grafica.grid(
            True,
            which="major",
            alpha=0.45,
        )
        grafica.grid(
            True,
            which="minor",
            alpha=0.12,
            linestyle=":",
        )
        grafica.margins(x=0)
        grafica.xaxis.set_major_locator(
            MaxNLocator(nbins=12)
        )
        grafica.xaxis.set_minor_locator(
            AutoMinorLocator(5)
        )

    graficas[-1].xaxis.set_major_formatter(
        FuncFormatter(
            _formateador_tiempo
        )
    )

    plt.setp(
        graficas[-1].get_xticklabels(),
        rotation=30,
        ha="right",
    )

    figura.suptitle(
        titulo,
        fontsize=12,
    )

    plt.tight_layout(
        rect=[0, 0.04, 1, 0.94]
    )

    return figura, graficas, tiempo


# ============================================================
# SELECTOR INTERACTIVO
# ============================================================

def _seleccionar_muestra_interactiva(
    datos,
    datos_vista,
    titulo,
):
    """
    Clic: coloca una selección provisional.
    Flecha izquierda/derecha: mueve una muestra IMU.
    Enter: confirma.
    Esc: borra la selección.

    El usuario puede utilizar libremente el zoom/pan de Matplotlib.
    Mientras una herramienta de navegación esté activa, los clics no
    cambian la selección.
    """
    figura, graficas, tiempo = _dibujar_cuatro_senales(
        datos,
        datos_vista,
        titulo,
    )

    indices_vista = datos_vista.index.to_numpy()

    if len(indices_vista) == 0:
        plt.close(figura)
        return None, None

    lineas = [
        grafica.axvline(
            np.nan,
            color="black",
            linestyle="--",
            linewidth=1.3,
            zorder=10,
        )
        for grafica in graficas
    ]

    texto_estado = figura.text(
        0.5,
        0.012,
        "Sin selección provisional",
        ha="center",
        va="bottom",
        fontsize=10,
    )

    estado = {
        "indice": None,
        "confirmado": False,
    }


    def velocidad_cercana(indice):
        tiempo_us = datos.loc[
            indice,
            "t_us",
        ]

        filas = datos[
            (datos["gps_new"] == 1)
            & (datos["gps_valid"] == 1)
            & datos["speed_2d_mps"].notna()
        ]

        if len(filas) == 0:
            return None

        indice_gps = (
            filas["t_us"]
            - tiempo_us
        ).abs().idxmin()

        return float(
            datos.loc[
                indice_gps,
                "speed_2d_mps",
            ]
        ) * 3.6

    def actualizar(indice):
        if indice not in datos_vista.index:
            return

        estado["indice"] = int(indice)

        segundo = float(
            (
                datos.loc[
                    indice,
                    "t_us",
                ]
                - datos["t_us"].iloc[0]
            )
            / 1_000_000
        )

        for linea in lineas:
            linea.set_xdata(
                [segundo, segundo]
            )

        velocidad = velocidad_cercana(
            indice
        )

        texto_velocidad = (
            "-"
            if velocidad is None
            else f"{velocidad:.2f} km/h"
        )

        texto_estado.set_text(
            "Selección provisional: "
            f"{formatear_tiempo(segundo)}"
            f"   |   v={texto_velocidad}"
            f"   |   ax={float(datos.loc[indice, 'ax_g']):.4f} g"
            f"   |   gy={float(datos.loc[indice, 'gy_dps']):.3f} °/s"
            f"   |   gz={float(datos.loc[indice, 'gz_dps']):.3f} °/s"
            "   |   ←/→ una muestra   |   Enter confirmar"
        )

        figura.canvas.draw_idle()

    def navegacion_activa():
        toolbar = getattr(
            figura.canvas.manager,
            "toolbar",
            None,
        )

        if toolbar is None:
            return False

        modo = getattr(
            toolbar,
            "mode",
            "",
        )

        return bool(modo)

    def al_hacer_clic(evento):
        if evento.xdata is None:
            return

        if navegacion_activa():
            return

        segundo_clic = float(
            evento.xdata
        )

        diferencias = (
            tiempo
            - segundo_clic
        ).abs()

        indice = int(
            diferencias.idxmin()
        )

        actualizar(indice)

    def al_pulsar_tecla(evento):
        tecla = evento.key

        if tecla == "escape":
            estado["indice"] = None

            for linea in lineas:
                linea.set_xdata(
                    [np.nan, np.nan]
                )

            texto_estado.set_text(
                "Sin selección provisional"
            )

            figura.canvas.draw_idle()
            return

        if tecla in ("left", "right"):
            if estado["indice"] is None:
                return

            posiciones = np.where(
                indices_vista
                == estado["indice"]
            )[0]

            if len(posiciones) == 0:
                return

            posicion = int(
                posiciones[0]
            )

            if tecla == "left":
                posicion = max(
                    0,
                    posicion - 1,
                )
            else:
                posicion = min(
                    len(indices_vista) - 1,
                    posicion + 1,
                )

            actualizar(
                int(indices_vista[posicion])
            )
            return

        if tecla in ("enter", "return"):
            if estado["indice"] is None:
                return

            estado["confirmado"] = True
            plt.close(figura)

    figura.canvas.mpl_connect(
        "button_press_event",
        al_hacer_clic,
    )

    figura.canvas.mpl_connect(
        "key_press_event",
        al_pulsar_tecla,
    )

    plt.show()

    if (
        not estado["confirmado"]
        or estado["indice"] is None
    ):
        return None, None

    indice = estado["indice"]

    segundo = float(
        (
            datos.loc[
                indice,
                "t_us",
            ]
            - datos["t_us"].iloc[0]
        )
        / 1_000_000
    )

    return indice, segundo


# ============================================================
# SELECCIONAR ZONA APROXIMADA
# ============================================================

def seleccionar_zona(datos):
    tiempo = (
        datos["t_us"]
        - datos["t_us"].iloc[0]
    ) / 1_000_000

    filas_gps = (
        (datos["gps_new"] == 1)
        & (datos["gps_valid"] == 1)
        & datos["speed_2d_mps"].notna()
    )

    tiempo_gps = tiempo[filas_gps]
    velocidad_kmh = (
        datos.loc[
            filas_gps,
            "speed_2d_mps",
        ]
        * 3.6
    )

    if len(tiempo_gps) == 0:
        print(
            "No hay datos GNSS válidos para representar la velocidad."
        )
        return None

    figura, grafica = plt.subplots(
        figsize=(14, 7)
    )

    grafica.plot(
        tiempo_gps,
        velocidad_kmh,
        linewidth=1,
    )

    eventos = (
        datos["event"]
        .fillna("")
        .astype(str)
        .str.strip()
    )

    for indice in datos.index[
        eventos != ""
    ]:
        segundo_marca = tiempo.loc[
            indice
        ]
        nombre_marca = eventos.loc[
            indice
        ]

        grafica.axvline(
            segundo_marca,
            linestyle="--",
            linewidth=1,
        )

        grafica.text(
            segundo_marca,
            0.97,
            nombre_marca,
            rotation=90,
            fontsize=8,
            verticalalignment="top",
            transform=grafica.get_xaxis_transform(),
        )

    grafica.set_title(
        "Velocidad GNSS - selección aproximada\n"
        "Amplía el ensayo completo, con margen tras la llegada, y cierra la ventana. El límite derecho acota la búsqueda de meta.",
        fontsize=12,
    )

    grafica.set_xlabel(
        "Tiempo desde el inicio del registro (mm:ss.mmm)"
    )
    grafica.set_ylabel(
        "Velocidad (km/h)"
    )
    grafica.grid(True)
    grafica.margins(x=0)
    grafica.xaxis.set_major_formatter(
        FuncFormatter(
            _formateador_tiempo
        )
    )

    plt.setp(
        grafica.get_xticklabels(),
        rotation=30,
        ha="right",
    )

    plt.tight_layout()

    print()
    print(
        "Amplía la zona donde se encuentra la prueba y después cierra la gráfica."
    )

    plt.show()

    inicio_visible, final_visible = (
        grafica.get_xlim()
    )

    inicio_visible = max(
        inicio_visible,
        tiempo.iloc[0],
    )

    final_visible = min(
        final_visible,
        tiempo.iloc[-1],
    )

    dentro_zona = (
        (tiempo >= inicio_visible)
        & (tiempo <= final_visible)
    )

    datos_zona = datos.loc[
        dentro_zona
    ].copy()

    plt.close(figura)
    return datos_zona


# ============================================================
# SELECCIONAR INICIO
# ============================================================

def seleccionar_inicio(datos, datos_zona):
    """
    Criterio:

    - ax_g es la señal principal.
    - Se selecciona la última muestra del estado previo, justo antes
      del aumento sostenido asociado a la primera acción propulsiva.
    - gy_dps y gz_dps sirven como confirmación de la primera palada.
    - La velocidad GNSS es una referencia adicional.
    """
    print()
    print("SALIDA")
    print("------")
    print(
        "Clic: selección provisional | ←/→: una muestra IMU | Enter: confirmar | Esc: borrar"
    )

    return _seleccionar_muestra_interactiva(
        datos,
        datos_zona,
        "SALIDA - selección precisa\n"
        "Referencia principal: último punto previo a la subida sostenida de ax_g. "
        "gy_dps y gz_dps ayudan a confirmar la primera palada.\n"
        "Puedes hacer zoom manualmente. Desactiva la lupa antes de marcar.",
    )



# ============================================================
# SELECCIONAR REFERENCIA DE ORIENTACIÓN
# ============================================================

def seleccionar_referencia(
    datos,
    datos_zona,
    indice_inicio,
):
    """
    Selecciona un punto posterior a la salida que se utilizará
    únicamente para conocer la dirección general del recorrido.

    No representa la meta ni determina el tiempo final.
    """

    datos_referencia = datos_zona.loc[
        datos_zona.index > indice_inicio
    ].copy()

    if len(datos_referencia) < 10:
        return None, None

    print()
    print("REFERENCIA DE ORIENTACIÓN")
    print("-------------------------")
    print(
        "Selecciona un punto posterior de la trayectoria."
    )
    print(
        "Este punto solo se utilizará para calcular la dirección "
        "general de la pista."
    )
    print(
        "No selecciona el tiempo final: orienta el eje sobre el que se coloca la meta."
    )

    return _seleccionar_muestra_interactiva(
        datos,
        datos_referencia,
        "REFERENCIA DE ORIENTACIÓN\n"
        "Selecciona un punto posterior de la trayectoria para indicar "
        "la dirección general del recorrido.\n"
        "No necesitas marcar un instante exacto.",
    )