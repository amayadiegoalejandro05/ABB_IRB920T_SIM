"""
SIMULADOR ABB IRB 920T-6/0.65
Robot SCARA de 4 GDL - Configuración R-R-P-R

Interfaz:
- PyBullet: visualización 3D limpia.
- Tkinter: panel de control completo en español.

Modelo cinemático:
J1 = Revoluta
J2 = Revoluta
J3 = Prismática
J4 = Revoluta

Dimensiones:
L1 = 260 mm
L2 = 390 mm
Alcance máximo = 650 mm
Carrera Z = 180 mm
"""

from __future__ import annotations

import math
import os
import time
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass

import numpy as np
import pybullet as p
import pybullet_data


# ============================================================
# 1. ESPECIFICACIONES DEL ABB IRB 920T-6/0.65
# ============================================================

@dataclass(frozen=True)
class IRB920T:
    # Longitudes principales [m]
    L1: float = 0.260
    L2: float = 0.390

    alcance_maximo: float = 0.650
    radio_minimo: float = 0.2315

    # Altura del eje principal del modelo
    altura_hombro: float = 0.345

    # Carrera vertical
    carrera_z: float = 0.180

    # Límites articulares
    q1_min: float = math.radians(-150)
    q1_max: float = math.radians(150)

    q2_min: float = math.radians(-145)
    q2_max: float = math.radians(145)

    q3_min: float = -0.180
    q3_max: float = 0.0

    q4_min: float = math.radians(-400)
    q4_max: float = math.radians(400)

    # Velocidades máximas
    v1_max: float = math.radians(396)
    v2_max: float = math.radians(780)
    v3_max: float = 1.2
    v4_max: float = math.radians(2500)

    carga_maxima: float = 6.0


ROBOT = IRB920T()


# Índices del URDF
J1 = 0
J2 = 1
J3 = 2
J4 = 3
TCP_LINK = 4


# ============================================================
# 2. COLORES
# ============================================================

NARANJA = [1.0, 0.45, 0.05]
VERDE = [0.1, 1.0, 0.25]
CYAN = [0.15, 0.9, 1.0]
ROJO = [1.0, 0.15, 0.15]
GRIS = [0.55, 0.55, 0.55]
AMARILLO = [1.0, 0.8, 0.1]
VIOLETA = [0.65, 0.35, 1.0]


# ============================================================
# 3. CINEMÁTICA DIRECTA
# ============================================================

def cinematica_directa(q1, q2, q3, q4):
    """
    Cinemática directa planar del SCARA.
    """

    x = (
        ROBOT.L1 * math.cos(q1)
        + ROBOT.L2 * math.cos(q1 + q2)
    )

    y = (
        ROBOT.L1 * math.sin(q1)
        + ROBOT.L2 * math.sin(q1 + q2)
    )

    z = q3

    yaw = q1 + q2 + q4

    return x, y, z, yaw


# ============================================================
# 4. CINEMÁTICA INVERSA
# ============================================================

def dentro(valor, minimo, maximo):
    return minimo <= valor <= maximo


def cinematica_inversa(x, y, z, yaw, codo=1):

    r2 = x**2 + y**2

    c2 = (
        r2
        - ROBOT.L1**2
        - ROBOT.L2**2
    ) / (
        2 * ROBOT.L1 * ROBOT.L2
    )

    if c2 < -1 or c2 > 1:
        return None

    c2 = np.clip(c2, -1, 1)

    s2 = codo * math.sqrt(
        max(0, 1 - c2**2)
    )

    q2 = math.atan2(s2, c2)

    k1 = ROBOT.L1 + ROBOT.L2 * c2
    k2 = ROBOT.L2 * s2

    q1 = (
        math.atan2(y, x)
        - math.atan2(k2, k1)
    )

    q1 = math.atan2(
        math.sin(q1),
        math.cos(q1)
    )

    q3 = z

    q4 = yaw - q1 - q2

    # Buscar equivalente de J4 dentro de ±400°
    opciones = [
        q4 + k * 2 * math.pi
        for k in range(-2, 3)
    ]

    q4_valido = None

    for valor in opciones:
        if dentro(
            valor,
            ROBOT.q4_min,
            ROBOT.q4_max
        ):
            q4_valido = valor
            break

    if q4_valido is None:
        return None

    q4 = q4_valido

    if not dentro(q1, ROBOT.q1_min, ROBOT.q1_max):
        return None

    if not dentro(q2, ROBOT.q2_min, ROBOT.q2_max):
        return None

    if not dentro(q3, ROBOT.q3_min, ROBOT.q3_max):
        return None

    return np.array(
        [q1, q2, q3, q4],
        dtype=float
    )


def mejor_solucion_ik(x, y, z, yaw, actual):

    soluciones = []

    for codo in (+1, -1):

        solucion = cinematica_inversa(
            x,
            y,
            z,
            yaw,
            codo
        )

        if solucion is not None:

            error = np.linalg.norm([
                solucion[0] - actual[0],
                solucion[1] - actual[1],
                5 * (solucion[2] - actual[2]),
                0.2 * (solucion[3] - actual[3])
            ])

            soluciones.append(
                (error, solucion)
            )

    if not soluciones:
        return None

    soluciones.sort(
        key=lambda elemento: elemento[0]
    )

    return soluciones[0][1]


# ============================================================
# 5. TRAYECTORIA AUTOMÁTICA
# ============================================================

WAYPOINTS = [

    # X     Y       Z       Yaw      Tiempo

    (0.42, -0.22, -0.030, 0,   0.70),

    (0.42, -0.22, -0.145, 0,   0.35),

    (0.42, -0.22, -0.030, 0,   0.35),

    (0.30,  0.34, -0.030, 90,  0.85),

    (0.30,  0.34, -0.145, 90,  0.35),

    (0.30,  0.34, -0.030, 90,  0.35),

    (0.42, -0.22, -0.030, 0,   0.85)
]


def interpolacion_quintica(s):

    s = np.clip(s, 0, 1)

    return (
        10 * s**3
        - 15 * s**4
        + 6 * s**5
    )


def trayectoria_automatica(t):

    tiempos = [
        punto[4]
        for punto in WAYPOINTS
    ]

    periodo = sum(tiempos)

    t = t % periodo

    acumulado = 0

    for i, dt in enumerate(tiempos):

        if t <= acumulado + dt:

            A = WAYPOINTS[i]

            B = WAYPOINTS[
                (i + 1) % len(WAYPOINTS)
            ]

            s = (
                t - acumulado
            ) / dt

            s = interpolacion_quintica(s)

            x = (1-s)*A[0] + s*B[0]

            y = (1-s)*A[1] + s*B[1]

            z = (1-s)*A[2] + s*B[2]

            yaw = math.radians(
                (1-s)*A[3]
                + s*B[3]
            )

            return x, y, z, yaw

        acumulado += dt

    return (
        WAYPOINTS[0][0],
        WAYPOINTS[0][1],
        WAYPOINTS[0][2],
        math.radians(WAYPOINTS[0][3])
    )


# ============================================================
# 6. DIBUJO DE EJES
# ============================================================

def dibujar_ejes_mundo():

    longitud = 0.22

    p.addUserDebugLine(
        [0, 0, 0.002],
        [longitud, 0, 0.002],
        [1, 0, 0],
        4
    )

    p.addUserDebugLine(
        [0, 0, 0.002],
        [0, longitud, 0.002],
        [0, 1, 0],
        4
    )

    p.addUserDebugLine(
        [0, 0, 0.002],
        [0, 0, longitud],
        [0, 0, 1],
        4
    )

    p.addUserDebugText(
        "X",
        [longitud + 0.015, 0, 0],
        [1, 0, 0],
        1.4
    )

    p.addUserDebugText(
        "Y",
        [0, longitud + 0.015, 0],
        [0, 1, 0],
        1.4
    )

    p.addUserDebugText(
        "Z",
        [0, 0, longitud + 0.015],
        [0, 0, 1],
        1.4
    )


# ============================================================
# 7. MARCOS DE REFERENCIA
# ============================================================

def dibujar_frame(
    posicion,
    orientacion,
    longitud=0.055,
    tiempo=0.05
):

    matriz = np.array(
        p.getMatrixFromQuaternion(
            orientacion
        )
    ).reshape(3, 3)

    origen = np.array(posicion)

    colores = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ]

    for i in range(3):

        destino = (
            origen
            + longitud * matriz[:, i]
        )

        p.addUserDebugLine(
            origen.tolist(),
            destino.tolist(),
            colores[i],
            2.5,
            lifeTime=tiempo
        )


# ============================================================
# 8. WORKSPACE
# ============================================================

def punto_xy(q1, q2):

    return np.array([

        ROBOT.L1 * math.cos(q1)
        + ROBOT.L2
        * math.cos(q1 + q2),

        ROBOT.L1 * math.sin(q1)
        + ROBOT.L2
        * math.sin(q1 + q2)

    ])


def circulo_debug(
    radio,
    z,
    color,
    ancho=1.5
):

    ids = []

    angulos = np.linspace(
        -math.pi,
        math.pi,
        180
    )

    puntos = [

        [
            radio * math.cos(a),
            radio * math.sin(a),
            z
        ]

        for a in angulos
    ]

    for A, B in zip(
        puntos[:-1],
        puntos[1:]
    ):

        ids.append(
            p.addUserDebugLine(
                A,
                B,
                color,
                ancho
            )
        )

    return ids


def dibujar_workspace():

    ids = []

    z_superior = ROBOT.altura_hombro

    z_inferior = (
        ROBOT.altura_hombro
        - ROBOT.carrera_z
    )

    n = 100

    curvas = []

    for q1 in (
        ROBOT.q1_min,
        ROBOT.q1_max
    ):

        curvas.append([
            punto_xy(q1, q2)
            for q2
            in np.linspace(
                ROBOT.q2_min,
                ROBOT.q2_max,
                n
            )
        ])

    for q2 in (
        ROBOT.q2_min,
        ROBOT.q2_max
    ):

        curvas.append([
            punto_xy(q1, q2)
            for q1
            in np.linspace(
                ROBOT.q1_min,
                ROBOT.q1_max,
                n
            )
        ])

    # Frontera superior
    for curva in curvas:

        for A, B in zip(
            curva[:-1],
            curva[1:]
        ):

            ids.append(
                p.addUserDebugLine(
                    [A[0], A[1], z_superior],
                    [B[0], B[1], z_superior],
                    GRIS,
                    1.5
                )
            )

    # Frontera inferior
    for curva in curvas:

        for A, B in zip(
            curva[:-1],
            curva[1:]
        ):

            ids.append(
                p.addUserDebugLine(
                    [A[0], A[1], z_inferior],
                    [B[0], B[1], z_inferior],
                    CYAN,
                    1.3
                )
            )

    # Alcance máximo
    ids += circulo_debug(
        ROBOT.alcance_maximo,
        z_superior + 0.002,
        AMARILLO
    )

    # Radio interior
    ids += circulo_debug(
        ROBOT.radio_minimo,
        z_superior + 0.004,
        VIOLETA
    )

    return ids


# ============================================================
# 9. MARCADORES PICK / PLACE
# ============================================================

def crear_marcadores():

    forma_pick = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.025, 0.025, 0.012],
        rgbaColor=[0.1, 0.9, 0.2, 1]
    )

    forma_place = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.025, 0.025, 0.012],
        rgbaColor=[0.95, 0.15, 0.1, 1]
    )

    z = (
        ROBOT.altura_hombro
        - 0.145
        - 0.175
    )

    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=forma_pick,
        basePosition=[
            0.42,
            -0.22,
            z
        ]
    )

    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=forma_place,
        basePosition=[
            0.30,
            0.34,
            z
        ]
    )

    p.addUserDebugText(
        "PICK",
        [0.42, -0.22, z + 0.055],
        VERDE,
        1.2
    )

    p.addUserDebugText(
        "PLACE",
        [0.30, 0.34, z + 0.055],
        ROJO,
        1.2
    )


# ============================================================
# 10. INTERFAZ TKINTER
# ============================================================

class PanelControl:

    def __init__(self, root):

        self.root = root

        self.root.title(
            "ABB IRB 920T | Panel de Control"
        )

        self.root.geometry(
            "540x810+25+30"
        )

        self.root.minsize(
            500,
            700
        )

        self.root.configure(
            bg="#17191d"
        )

        # ---------------------------
        # VARIABLES
        # ---------------------------

        self.modo = tk.IntVar(value=0)

        self.j1 = tk.DoubleVar(value=20)
        self.j2 = tk.DoubleVar(value=-55)
        self.j3 = tk.DoubleVar(value=-45)
        self.j4 = tk.DoubleVar(value=35)

        self.velocidad = tk.DoubleVar(
            value=0.85
        )

        self.mostrar_workspace = tk.BooleanVar(
            value=True
        )

        self.mostrar_frames = tk.BooleanVar(
            value=True
        )

        self.mostrar_trayectoria = tk.BooleanVar(
            value=True
        )

        # Estado
        self.estado = tk.StringVar(
            value="Robot inicializando..."
        )

        self.estado_tcp = tk.StringVar(
            value="TCP: ---"
        )

        self.estado_joints = tk.StringVar(
            value="J1 --- | J2 --- | J3 --- | J4 ---"
        )

        self.estado_singularidad = tk.StringVar(
            value="Configuración estable"
        )

        # ---------------------------
        # ESTILO
        # ---------------------------

        estilo = ttk.Style()

        estilo.theme_use("clam")

        estilo.configure(
            "TFrame",
            background="#17191d"
        )

        estilo.configure(
            "Titulo.TLabel",
            background="#17191d",
            foreground="#ff851b",
            font=("Segoe UI", 16, "bold")
        )

        estilo.configure(
            "Subtitulo.TLabel",
            background="#17191d",
            foreground="#dadada",
            font=("Segoe UI", 10)
        )

        estilo.configure(
            "Seccion.TLabelframe",
            background="#202329",
            foreground="white"
        )

        estilo.configure(
            "Seccion.TLabelframe.Label",
            background="#17191d",
            foreground="#49d9ff",
            font=("Segoe UI", 11, "bold")
        )

        estilo.configure(
            "TLabel",
            background="#202329",
            foreground="white",
            font=("Segoe UI", 10)
        )

        estilo.configure(
            "TRadiobutton",
            background="#202329",
            foreground="white"
        )

        estilo.configure(
            "TCheckbutton",
            background="#202329",
            foreground="white"
        )

        estilo.configure(
            "TButton",
            font=("Segoe UI", 9, "bold")
        )

        # ---------------------------
        # CABECERA
        # ---------------------------

        cabecera = ttk.Frame(
            root
        )

        cabecera.pack(
            fill="x",
            padx=16,
            pady=(14, 6)
        )

        ttk.Label(
            cabecera,
            text="ABB IRB 920T-6/0.65",
            style="Titulo.TLabel"
        ).pack()

        ttk.Label(
            cabecera,
            text="Robot SCARA industrial | 4 GDL | R-R-P-R | Alcance 650 mm",
            style="Subtitulo.TLabel"
        ).pack(
            pady=(3, 0)
        )

        # ---------------------------
        # MODO
        # ---------------------------

        frame_modo = ttk.LabelFrame(
            root,
            text=" Modo de operación ",
            style="Seccion.TLabelframe"
        )

        frame_modo.pack(
            fill="x",
            padx=15,
            pady=7
        )

        ttk.Radiobutton(
            frame_modo,
            text="Control manual",
            variable=self.modo,
            value=0
        ).pack(
            side="left",
            padx=25,
            pady=9
        )

        ttk.Radiobutton(
            frame_modo,
            text="Pick and Place automático",
            variable=self.modo,
            value=1
        ).pack(
            side="left",
            padx=12
        )

        # ---------------------------
        # ARTICULACIONES
        # ---------------------------

        articulaciones = ttk.LabelFrame(
            root,
            text=" Control de articulaciones ",
            style="Seccion.TLabelframe"
        )

        articulaciones.pack(
            fill="x",
            padx=15,
            pady=7
        )

        self.crear_slider(
            articulaciones,
            "J1 — Rotación de la base θ1",
            self.j1,
            -150,
            150,
            "°",
            0.5
        )

        self.crear_slider(
            articulaciones,
            "J2 — Rotación del codo θ2",
            self.j2,
            -145,
            145,
            "°",
            0.5
        )

        self.crear_slider(
            articulaciones,
            "J3 — Desplazamiento vertical Z",
            self.j3,
            -180,
            0,
            "mm",
            1
        )

        self.crear_slider(
            articulaciones,
            "J4 — Rotación de la herramienta θ4",
            self.j4,
            -400,
            400,
            "°",
            1
        )

        # ---------------------------
        # AUTOMÁTICO
        # ---------------------------

        automatico = ttk.LabelFrame(
            root,
            text=" Movimiento automático ",
            style="Seccion.TLabelframe"
        )

        automatico.pack(
            fill="x",
            padx=15,
            pady=7
        )

        self.crear_slider(
            automatico,
            "Velocidad de la demostración",
            self.velocidad,
            0.2,
            2.0,
            "x",
            0.05
        )

        # ---------------------------
        # VISUALIZACIÓN
        # ---------------------------

        visual = ttk.LabelFrame(
            root,
            text=" Visualización ",
            style="Seccion.TLabelframe"
        )

        visual.pack(
            fill="x",
            padx=15,
            pady=7
        )

        ttk.Checkbutton(
            visual,
            text="Mostrar espacio de trabajo",
            variable=self.mostrar_workspace
        ).grid(
            row=0,
            column=0,
            padx=12,
            pady=7,
            sticky="w"
        )

        ttk.Checkbutton(
            visual,
            text="Mostrar sistemas de referencia",
            variable=self.mostrar_frames
        ).grid(
            row=0,
            column=1,
            padx=12,
            pady=7,
            sticky="w"
        )

        ttk.Checkbutton(
            visual,
            text="Mostrar trayectoria del TCP",
            variable=self.mostrar_trayectoria
        ).grid(
            row=1,
            column=0,
            padx=12,
            pady=7,
            sticky="w"
        )

        # ---------------------------
        # CÁMARA
        # ---------------------------

        camara = ttk.LabelFrame(
            root,
            text=" Cámara 3D ",
            style="Seccion.TLabelframe"
        )

        camara.pack(
            fill="x",
            padx=15,
            pady=7
        )

        botones = ttk.Frame(
            camara
        )

        botones.pack(
            pady=8
        )

        ttk.Button(
            botones,
            text="Isométrica",
            command=self.vista_isometrica
        ).grid(
            row=0,
            column=0,
            padx=4
        )

        ttk.Button(
            botones,
            text="Superior",
            command=self.vista_superior
        ).grid(
            row=0,
            column=1,
            padx=4
        )

        ttk.Button(
            botones,
            text="Frontal",
            command=self.vista_frontal
        ).grid(
            row=0,
            column=2,
            padx=4
        )

        ttk.Button(
            botones,
            text="Lateral",
            command=self.vista_lateral
        ).grid(
            row=0,
            column=3,
            padx=4
        )

        ttk.Label(
            camara,
            text=(
                "La vista puede girarse libremente con el mouse en la ventana 3D."
            )
        ).pack(
            pady=(0, 7)
        )

        # ---------------------------
        # ESTADO
        # ---------------------------

        estado = ttk.LabelFrame(
            root,
            text=" Estado en tiempo real ",
            style="Seccion.TLabelframe"
        )

        estado.pack(
            fill="x",
            padx=15,
            pady=7
        )

        ttk.Label(
            estado,
            textvariable=self.estado
        ).pack(
            anchor="w",
            padx=12,
            pady=(8, 2)
        )

        ttk.Label(
            estado,
            textvariable=self.estado_joints
        ).pack(
            anchor="w",
            padx=12,
            pady=2
        )

        ttk.Label(
            estado,
            textvariable=self.estado_tcp
        ).pack(
            anchor="w",
            padx=12,
            pady=2
        )

        self.label_singularidad = tk.Label(
            estado,
            textvariable=self.estado_singularidad,
            bg="#202329",
            fg="#51ff6e",
            font=("Segoe UI", 10, "bold")
        )

        self.label_singularidad.pack(
            anchor="w",
            padx=12,
            pady=(2, 8)
        )

    # ========================================================
    # Slider ancho
    # ========================================================

    def crear_slider(
        self,
        padre,
        titulo,
        variable,
        minimo,
        maximo,
        unidad,
        resolucion
    ):

        frame = tk.Frame(
            padre,
            bg="#202329"
        )

        frame.pack(
            fill="x",
            padx=14,
            pady=5
        )

        cabecera = tk.Frame(
            frame,
            bg="#202329"
        )

        cabecera.pack(
            fill="x"
        )

        tk.Label(
            cabecera,
            text=titulo,
            bg="#202329",
            fg="#ffffff",
            font=("Segoe UI", 10, "bold"),
            anchor="w"
        ).pack(
            side="left"
        )

        valor = tk.Label(
            cabecera,
            bg="#202329",
            fg="#49d9ff",
            font=("Consolas", 10, "bold")
        )

        valor.pack(
            side="right"
        )

        def actualizar(_=None):

            numero = variable.get()

            if resolucion >= 1:
                texto = f"{numero:.0f} {unidad}"
            else:
                texto = f"{numero:.2f} {unidad}"

            valor.config(
                text=texto
            )

        escala = tk.Scale(
            frame,
            from_=minimo,
            to=maximo,
            variable=variable,
            orient="horizontal",
            resolution=resolucion,
            showvalue=False,
            length=455,
            sliderlength=18,
            bg="#202329",
            fg="white",
            troughcolor="#444952",
            activebackground="#ff851b",
            highlightthickness=0,
            bd=0,
            command=actualizar
        )

        escala.pack(
            fill="x"
        )

        actualizar()

    # ========================================================
    # Vistas de cámara
    # ========================================================

    @staticmethod
    def vista_isometrica():

        p.resetDebugVisualizerCamera(
            1.20,
            45,
            -28,
            [0.18, 0.02, 0.23]
        )

    @staticmethod
    def vista_superior():

        p.resetDebugVisualizerCamera(
            1.15,
            0,
            -89,
            [0.15, 0.0, 0.20]
        )

    @staticmethod
    def vista_frontal():

        p.resetDebugVisualizerCamera(
            1.20,
            0,
            -15,
            [0.20, 0.0, 0.22]
        )

    @staticmethod
    def vista_lateral():

        p.resetDebugVisualizerCamera(
            1.20,
            90,
            -15,
            [0.20, 0.0, 0.22]
        )


# ============================================================
# 11. SIMULACIÓN
# ============================================================

def main():

    # --------------------------------------------------------
    # PyBullet
    # --------------------------------------------------------

    cliente = p.connect(
        p.GUI
    )

    if cliente < 0:
        raise RuntimeError(
            "No fue posible abrir PyBullet."
        )

    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )

    p.setGravity(
        0,
        0,
        -9.81
    )

    p.setTimeStep(
        1 / 240
    )

    p.setPhysicsEngineParameter(
        numSolverIterations=120
    )

    # --------------------------------------------------------
    # LIMPIAR INTERFAZ NATIVA DE PYBULLET
    # --------------------------------------------------------

    # Esto elimina el panel angosto de sliders
    # y las ventanas RGB/Depth/Segmentation.
    p.configureDebugVisualizer(
        p.COV_ENABLE_GUI,
        0
    )

    p.configureDebugVisualizer(
        p.COV_ENABLE_RGB_BUFFER_PREVIEW,
        0
    )

    p.configureDebugVisualizer(
        p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
        0
    )

    p.configureDebugVisualizer(
        p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
        0
    )

    p.configureDebugVisualizer(
        p.COV_ENABLE_SHADOWS,
        1
    )

    # --------------------------------------------------------
    # CÁMARA INICIAL
    # --------------------------------------------------------

    # IMPORTANTE:
    # Solo la configuramos UNA vez.
    # NO se actualiza continuamente.
    # Por eso puedes moverla libremente en 360°.
    p.resetDebugVisualizerCamera(
        cameraDistance=1.20,
        cameraYaw=45,
        cameraPitch=-28,
        cameraTargetPosition=[
            0.18,
            0.02,
            0.23
        ]
    )

    # --------------------------------------------------------
    # SUELO
    # --------------------------------------------------------

    piso = p.loadURDF(
        "plane.urdf"
    )

    p.changeDynamics(
        piso,
        -1,
        lateralFriction=0.8
    )

    p.changeVisualShape(
        piso,
        -1,
        rgbaColor=[
            0.86,
            0.91,
            0.97,
            1
        ]
    )

    # --------------------------------------------------------
    # CARGAR ROBOT
    # --------------------------------------------------------

    ruta = os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)
        ),
        "irb920t_6_065.urdf"
    )

    robot = p.loadURDF(
        ruta,
        basePosition=[
            0,
            0,
            0
        ],
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    # Evitar falsas colisiones entre links vecinos
    pares = [
        (-1, 0),
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4)
    ]

    for A, B in pares:

        p.setCollisionFilterPair(
            robot,
            robot,
            A,
            B,
            0
        )

    # --------------------------------------------------------
    # POSTURA INICIAL
    # --------------------------------------------------------

    q_inicial = np.array([

        math.radians(20),

        math.radians(-55),

        -0.045,

        math.radians(35)

    ])

    for articulacion, valor in zip(
        [J1, J2, J3, J4],
        q_inicial
    ):

        p.resetJointState(
            robot,
            articulacion,
            valor
        )

    # --------------------------------------------------------
    # ELEMENTOS VISUALES
    # --------------------------------------------------------

    dibujar_ejes_mundo()

    workspace_ids = (
        dibujar_workspace()
    )

    crear_marcadores()

    # Título 3D discreto
    p.addUserDebugText(
        "ABB IRB 920T | SCARA R-R-P-R",
        [-0.15, -0.12, 0.62],
        NARANJA,
        1.4
    )

    # --------------------------------------------------------
    # MARCADOR TCP
    # --------------------------------------------------------

    forma_tcp = p.createVisualShape(

        p.GEOM_SPHERE,

        radius=0.012,

        rgbaColor=[
            0.1,
            1,
            0.2,
            1
        ]
    )

    marcador_tcp = p.createMultiBody(

        baseMass=0,

        baseVisualShapeIndex=forma_tcp,

        basePosition=[
            0,
            0,
            1
        ]
    )

    # --------------------------------------------------------
    # INTERFAZ TKINTER
    # --------------------------------------------------------

    root = tk.Tk()

    panel = PanelControl(
        root
    )

    # Mostrar la ventana de control al frente al iniciar
    root.lift()

    root.attributes(
        "-topmost",
        True
    )

    root.after(
        500,
        lambda:
        root.attributes(
            "-topmost",
            False
        )
    )

    ejecutando = True

    def cerrar():

        nonlocal ejecutando

        ejecutando = False

        root.destroy()

    root.protocol(
        "WM_DELETE_WINDOW",
        cerrar
    )

    # --------------------------------------------------------
    # VARIABLES
    # --------------------------------------------------------

    velocidad_max = [

        ROBOT.v1_max,

        ROBOT.v2_max,

        ROBOT.v3_max,

        ROBOT.v4_max
    ]

    fuerzas = [
        260,
        200,
        500,
        80
    ]

    tiempo_auto = 0

    ultimo_tiempo = time.perf_counter()

    ultimo_workspace = True

    trayectoria = []

    ultimo_punto_traza = 0

    # --------------------------------------------------------
    # LOOP PRINCIPAL
    # --------------------------------------------------------

    try:

        while (
            ejecutando
            and p.isConnected()
        ):

            # Actualizar Tkinter
            try:

                root.update_idletasks()

                root.update()

            except tk.TclError:
                break

            ahora = time.perf_counter()

            dt = min(
                max(
                    ahora - ultimo_tiempo,
                    0.0001
                ),
                0.05
            )

            ultimo_tiempo = ahora

            # ------------------------------------------------
            # Mostrar / ocultar workspace
            # ------------------------------------------------

            mostrar_ws = (
                panel.mostrar_workspace.get()
            )

            if mostrar_ws != ultimo_workspace:

                if mostrar_ws:

                    workspace_ids = (
                        dibujar_workspace()
                    )

                else:

                    for item in workspace_ids:

                        p.removeUserDebugItem(
                            item
                        )

                    workspace_ids = []

                ultimo_workspace = mostrar_ws

            # ------------------------------------------------
            # Estado actual
            # ------------------------------------------------

            q_actual = np.array([

                p.getJointState(
                    robot,
                    articulacion
                )[0]

                for articulacion
                in [J1, J2, J3, J4]
            ])

            # ------------------------------------------------
            # MANUAL
            # ------------------------------------------------

            if panel.modo.get() == 0:

                q_objetivo = np.array([

                    math.radians(
                        panel.j1.get()
                    ),

                    math.radians(
                        panel.j2.get()
                    ),

                    panel.j3.get()
                    / 1000,

                    math.radians(
                        panel.j4.get()
                    )

                ])

                texto_estado = (
                    "Modo manual"
                )

            # ------------------------------------------------
            # AUTOMÁTICO
            # ------------------------------------------------

            else:

                tiempo_auto += (
                    dt
                    * panel.velocidad.get()
                )

                x_d, y_d, z_d, yaw_d = (
                    trayectoria_automatica(
                        tiempo_auto
                    )
                )

                solucion = mejor_solucion_ik(

                    x_d,

                    y_d,

                    z_d,

                    yaw_d,

                    q_actual
                )

                if solucion is None:

                    q_objetivo = (
                        q_actual.copy()
                    )

                    texto_estado = (
                        "AUTO | Objetivo no alcanzable"
                    )

                else:

                    q_objetivo = solucion

                    texto_estado = (
                        "AUTO | Pick and Place"
                    )

            # ------------------------------------------------
            # SATURAR LÍMITES
            # ------------------------------------------------

            q_objetivo[0] = np.clip(
                q_objetivo[0],
                ROBOT.q1_min,
                ROBOT.q1_max
            )

            q_objetivo[1] = np.clip(
                q_objetivo[1],
                ROBOT.q2_min,
                ROBOT.q2_max
            )

            q_objetivo[2] = np.clip(
                q_objetivo[2],
                ROBOT.q3_min,
                ROBOT.q3_max
            )

            q_objetivo[3] = np.clip(
                q_objetivo[3],
                ROBOT.q4_min,
                ROBOT.q4_max
            )

            # ------------------------------------------------
            # CONTROL DE MOTORES
            # ------------------------------------------------

            for i, articulacion in enumerate(
                [J1, J2, J3, J4]
            ):

                p.setJointMotorControl2(

                    robot,

                    articulacion,

                    p.POSITION_CONTROL,

                    targetPosition=float(
                        q_objetivo[i]
                    ),

                    force=fuerzas[i],

                    positionGain=(
                        0.35
                        if articulacion == J3
                        else 0.25
                    ),

                    velocityGain=0.85,

                    maxVelocity=(
                        velocidad_max[i]
                    )
                )

            p.stepSimulation()

            # ------------------------------------------------
            # LEER NUEVO ESTADO
            # ------------------------------------------------

            q = np.array([

                p.getJointState(
                    robot,
                    articulacion
                )[0]

                for articulacion
                in [J1, J2, J3, J4]
            ])

            # ------------------------------------------------
            # TCP REAL SEGÚN URDF
            # ------------------------------------------------

            estado_tcp = p.getLinkState(

                robot,

                TCP_LINK,

                computeForwardKinematics=True

            )

            posicion_tcp = estado_tcp[4]

            orientacion_tcp = estado_tcp[5]

            p.resetBasePositionAndOrientation(

                marcador_tcp,

                posicion_tcp,

                [0, 0, 0, 1]
            )

            # ------------------------------------------------
            # FRAMES
            # ------------------------------------------------

            if panel.mostrar_frames.get():

                # Base
                pos_base, orn_base = (
                    p.getBasePositionAndOrientation(
                        robot
                    )
                )

                dibujar_frame(
                    pos_base,
                    orn_base,
                    0.065
                )

                # Articulaciones
                for link in range(
                    p.getNumJoints(robot)
                ):

                    estado = p.getLinkState(

                        robot,

                        link,

                        computeForwardKinematics=True
                    )

                    dibujar_frame(

                        estado[4],

                        estado[5],

                        0.04
                    )

                # TCP más grande
                dibujar_frame(

                    posicion_tcp,

                    orientacion_tcp,

                    0.07
                )

            # ------------------------------------------------
            # TRAYECTORIA TCP
            # ------------------------------------------------

            if (
                panel.mostrar_trayectoria.get()
                and ahora - ultimo_punto_traza > 0.05
            ):

                ultimo_punto_traza = ahora

                trayectoria.append(
                    list(posicion_tcp)
                )

                if len(trayectoria) >= 2:

                    p.addUserDebugLine(

                        trayectoria[-2],

                        trayectoria[-1],

                        VERDE,

                        2,

                        lifeTime=6
                    )

                trayectoria = (
                    trayectoria[-150:]
                )

            # ------------------------------------------------
            # SINGULARIDAD
            # ------------------------------------------------

            criterio = abs(
                math.sin(q[1])
            )

            singular = (
                criterio < 0.08
            )

            # ------------------------------------------------
            # YAW DEL TCP
            # ------------------------------------------------

            euler_tcp = (
                p.getEulerFromQuaternion(
                    orientacion_tcp
                )
            )

            yaw_tcp = math.degrees(
                euler_tcp[2]
            )

            # ------------------------------------------------
            # RADIO ACTUAL
            # ------------------------------------------------

            radio = math.sqrt(

                posicion_tcp[0]**2
                + posicion_tcp[1]**2
            )

            # ------------------------------------------------
            # ACTUALIZAR PANEL
            # ------------------------------------------------

            panel.estado.set(
                texto_estado
            )

            panel.estado_joints.set(

                f"J1 = {math.degrees(q[0]):7.2f}°   |   "
                f"J2 = {math.degrees(q[1]):7.2f}°   |   "
                f"J3 = {q[2]*1000:7.1f} mm   |   "
                f"J4 = {math.degrees(q[3]):7.2f}°"
            )

            panel.estado_tcp.set(

                f"TCP → "
                f"X = {posicion_tcp[0]*1000:7.1f} mm   |   "
                f"Y = {posicion_tcp[1]*1000:7.1f} mm   |   "
                f"Z = {posicion_tcp[2]*1000:7.1f} mm\n"
                f"Radio = {radio*1000:7.1f} mm   |   "
                f"Yaw = {yaw_tcp:7.2f}°"
            )

            if singular:

                panel.estado_singularidad.set(
                    "⚠ CERCA DE UNA CONFIGURACIÓN SINGULAR"
                )

                panel.label_singularidad.config(
                    fg="#ff4141"
                )

            else:

                panel.estado_singularidad.set(
                    "✓ Configuración cinemáticamente estable"
                )

                panel.label_singularidad.config(
                    fg="#51ff6e"
                )

            # ------------------------------------------------
            # TIEMPO REAL
            # ------------------------------------------------

            espera = (
                1 / 240
                - (
                    time.perf_counter()
                    - ahora
                )
            )

            if espera > 0:

                time.sleep(
                    espera
                )

    except KeyboardInterrupt:

        pass

    finally:

        if p.isConnected():

            p.disconnect()

        try:

            root.destroy()

        except:

            pass


# ============================================================
# 12. EJECUCIÓN
# ============================================================

if __name__ == "__main__":

    main()