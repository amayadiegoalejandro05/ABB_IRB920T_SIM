"""
Comprobación de instalación
ABB IRB 920T-6/0.65
"""

from __future__ import annotations

import os
import platform
import sys


def ok(mensaje: str) -> None:
    print(f"[OK] {mensaje}")


def error(mensaje: str) -> None:
    print(f"[ERROR] {mensaje}")


print()
print("=" * 58)
print(" ABB IRB 920T - COMPROBACIÓN DEL SISTEMA")
print("=" * 58)
print()

errores = 0


# ============================================================
# PYTHON
# ============================================================

print(f"Sistema operativo : {platform.system()} {platform.release()}")
print(f"Arquitectura       : {platform.machine()}")
print(f"Python             : {sys.version.split()[0]}")
print(f"Ejecutable Python  : {sys.executable}")
print()


if sys.version_info[:2] == (3, 11):
    ok("Python 3.11")
else:
    print(
        "[AVISO] Se recomienda Python 3.11. "
        f"Actualmente se está usando Python {sys.version.split()[0]}"
    )


# ============================================================
# NUMPY
# ============================================================

try:
    import numpy as np

    ok(f"NumPy {np.__version__}")

except Exception as exc:
    error(f"NumPy no disponible: {exc}")
    errores += 1


# ============================================================
# MATPLOTLIB
# ============================================================

try:
    import matplotlib

    ok(f"Matplotlib {matplotlib.__version__}")

except Exception as exc:
    error(f"Matplotlib no disponible: {exc}")
    errores += 1


# ============================================================
# TKINTER
# ============================================================

try:
    import tkinter

    ok(f"Tkinter {tkinter.TkVersion}")

except Exception as exc:
    error(f"Tkinter no disponible: {exc}")
    errores += 1


# ============================================================
# PYBULLET
# ============================================================

try:
    import pybullet as p
    import pybullet_data

    ok("PyBullet")

except Exception as exc:
    error(f"PyBullet no disponible: {exc}")
    errores += 1
    p = None


# ============================================================
# ARCHIVO URDF
# ============================================================

directorio = os.path.dirname(
    os.path.abspath(__file__)
)

ruta_urdf = os.path.join(
    directorio,
    "irb920t_6_065.urdf"
)


if os.path.isfile(ruta_urdf):

    ok("Archivo irb920t_6_065.urdf")

else:

    error(
        "No se encontró irb920t_6_065.urdf "
        "en la carpeta del proyecto."
    )

    errores += 1


# ============================================================
# COMPROBAR QUE PYBULLET PUEDA CARGAR EL URDF
# ============================================================

if p is not None and os.path.isfile(ruta_urdf):

    cliente = -1

    try:

        cliente = p.connect(
            p.DIRECT
        )

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        robot = p.loadURDF(
            ruta_urdf,
            useFixedBase=True
        )

        numero_articulaciones = (
            p.getNumJoints(robot)
        )

        ok(
            f"El URDF pudo cargarse "
            f"({numero_articulaciones} joints encontrados)"
        )

        if numero_articulaciones < 4:

            error(
                "El modelo debería contener "
                "al menos cuatro articulaciones."
            )

            errores += 1

    except Exception as exc:

        error(
            f"PyBullet no pudo cargar el URDF: {exc}"
        )

        errores += 1

    finally:

        if cliente >= 0:

            p.disconnect(
                cliente
            )


# ============================================================
# RESULTADO
# ============================================================

print()
print("=" * 58)

if errores == 0:

    print(
        " SISTEMA LISTO PARA EJECUTAR LA SIMULACIÓN"
    )

    print()
    print(
        " Ejecuta:"
    )

    print()
    print(
        "     python irb920t_sim.py"
    )

else:

    print(
        f" SE ENCONTRARON {errores} ERROR(ES)"
    )

    print()
    print(
        " Revisa la instalación antes de "
        "ejecutar la simulación."
    )

print("=" * 58)
print()


sys.exit(
    0 if errores == 0 else 1
)