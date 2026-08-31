# ABB IRB 920T-6/0.65 — Simulador SCARA 4 GDL

Simulación educativa en Python del robot industrial **ABB IRB 920T-6/0.65**, un manipulador tipo **SCARA de 4 grados de libertad** con configuración cinemática:

```text
R - R - P - R
```

El proyecto combina:

- PyBullet para la simulación y visualización 3D.
- Tkinter para la interfaz gráfica de control.
- NumPy para los cálculos cinemáticos.
- Matplotlib para visualizar el espacio de trabajo 2D.

> Este proyecto es un modelo educativo y no corresponde al modelo CAD oficial ni a un gemelo digital certificado por ABB.

---

## Características principales

El simulador incluye:

- Robot SCARA de 4 GDL.
- Cinemática directa.
- Cinemática inversa analítica.
- Dos soluciones posibles de codo para la IK.
- Límites articulares.
- Espacio de trabajo.
- Visualización del TCP.
- Sistemas de referencia.
- Detección aproximada de singularidades.
- Control manual de las cuatro articulaciones.
- Ciclo automático Pick and Place.
- Trayectoria suavizada mediante interpolación quíntica.
- Visualización de la trayectoria del TCP.
- Cámara 3D libre.
- Vistas predefinidas:
  - Isométrica.
  - Superior.
  - Frontal.
  - Lateral.
- Interfaz gráfica completamente en español.

---

# Especificaciones utilizadas

El modelo utiliza los siguientes parámetros cinemáticos para la variante IRB 920T-6/0.65:

| Parámetro | Valor |
|---|---:|
| Tipo | SCARA |
| Grados de libertad | 4 |
| Configuración | R-R-P-R |
| Longitud L1 | 260 mm |
| Longitud L2 | 390 mm |
| Alcance máximo | 650 mm |
| Carrera vertical J3 | 180 mm |
| Carga útil nominal | 6 kg |
| J1 | ±150° |
| J2 | ±145° |
| J3 | -180 a 0 mm |
| J4 | ±400° |

Velocidades máximas utilizadas en el modelo:

| Articulación | Velocidad máxima |
|---|---:|
| J1 | 396 °/s |
| J2 | 780 °/s |
| J3 | 1.2 m/s |
| J4 | 2500 °/s |

---

# Configuración de las articulaciones

El robot utiliza la configuración:

```text
J1        J2        J3        J4
R    -    R    -    P    -    R
```

donde:

```text
R = articulación revoluta
P = articulación prismática
```

La función principal de cada articulación es:

```text
J1 → rotación de la base
J2 → rotación del segundo eslabón o codo
J3 → desplazamiento vertical en Z
J4 → orientación de la herramienta
```

---

# Cinemática directa

Para los primeros dos grados de libertad se utiliza el modelo planar 2R:

```text
x = L1 cos(q1) + L2 cos(q1 + q2)

y = L1 sin(q1) + L2 sin(q1 + q2)
```

El desplazamiento vertical depende de J3:

```text
z = q3
```

La orientación de la herramienta alrededor de Z se modela como:

```text
yaw = q1 + q2 + q4
```

---

# Requisitos

La forma recomendada de ejecutar este proyecto es utilizando **Miniconda o Anaconda**.

Configuración recomendada:

```text
Python 3.11
PyBullet
NumPy
Matplotlib
Tkinter
```

## ¿Por qué Conda?

En algunos sistemas Windows, `pip` no encuentra un wheel precompilado compatible de PyBullet e intenta compilarlo localmente.

Esto puede producir errores como:

```text
Microsoft Visual C++ 14.0 or greater is required
```

Por esa razón, para este proyecto se recomienda instalar PyBullet desde **conda-forge** mediante el archivo `environment.yml`.

---

# Instalación recomendada

## 1. Instalar Miniconda

Instala Miniconda o Anaconda en el computador.

Después comprueba:

```bash
conda --version
```

Si PowerShell no reconoce `conda`, abre:

```text
Miniconda Prompt
```

o ejecuta una vez:

```powershell
conda init powershell
```

y reinicia la terminal.

---

## 2. Clonar el repositorio

```bash
git clone URL_DEL_REPOSITORIO
```

Entra en la carpeta:

```bash
cd ABB_IRB920T_SIM
```

---

## 3. Crear el entorno

Ejecuta:

```bash
conda env create -f environment.yml
```

Esto creará automáticamente un entorno llamado:

```text
irb920t
```

---

## 4. Activar el entorno

```bash
conda activate irb920t
```

La terminal debería mostrar algo similar a:

```text
(irb920t) C:\...\ABB_IRB920T_SIM>
```

---

## 5. Comprobar la instalación

Ejecuta:

```bash
python check_install.py
```

Si todo está correctamente instalado debería aparecer:

```text
==============================================
 ABB IRB 920T - COMPROBACIÓN DEL SISTEMA
==============================================

[OK] Python
[OK] NumPy
[OK] PyBullet
[OK] Matplotlib
[OK] Tkinter
[OK] Archivo URDF
[OK] El URDF pudo cargarse

SISTEMA LISTO PARA EJECUTAR LA SIMULACIÓN
```

---

# Ejecutar la simulación

Con el entorno activado:

```bash
python irb920t_sim.py
```

Se abrirán dos ventanas.

La primera corresponde al:

```text
VISOR 3D DE PYBULLET
```

La segunda corresponde al:

```text
PANEL DE CONTROL ABB IRB 920T
```

---

# Panel de control

La interfaz permite controlar independientemente:

```text
J1 — Rotación de la base
J2 — Rotación del codo
J3 — Movimiento vertical
J4 — Rotación de la herramienta
```

En modo manual los sliders controlan directamente la configuración:

```text
q = [q1, q2, q3, q4]
```

---

# Modo automático

Seleccionando:

```text
Pick and Place automático
```

el simulador ejecuta una trayectoria cartesiana predeterminada.

La trayectoria utiliza una interpolación quíntica:

```text
s(t) = 10t³ - 15t⁴ + 6t⁵
```

Esto genera transiciones suaves de velocidad y aceleración entre los puntos de la trayectoria.

---

# Cámara 3D

La cámara puede moverse libremente alrededor del robot utilizando el mouse dentro de la ventana de PyBullet.

La interfaz también proporciona las vistas:

```text
Isométrica
Superior
Frontal
Lateral
```

Esto facilita analizar el robot desde diferentes perspectivas.

---

# Espacio de trabajo

El simulador puede mostrar el espacio de trabajo generado por los límites de:

```text
J1 = ±150°
J2 = ±145°
```

El círculo exterior corresponde al alcance máximo aproximado:

```text
Rmax = 650 mm
```

El espacio real alcanzable no es una circunferencia completa debido a los límites articulares.

Para visualizar adicionalmente el workspace en 2D:

```bash
python workspace_plot.py
```

---

# TCP

El simulador muestra la posición del TCP en tiempo real:

```text
X
Y
Z
```

además de su orientación alrededor del eje vertical:

```text
Yaw
```

---

# Singularidades

Para la cadena planar formada por J1 y J2, una condición importante aparece cuando:

```text
sin(q2) ≈ 0
```

El programa supervisa este valor y muestra una advertencia cuando el robot se aproxima a una configuración singular.

---

# Organización del proyecto

```text
ABB_IRB920T_SIM/
│
├── irb920t_sim.py
│   Programa principal.
│   Simulación, cinemática, interfaz y control.
│
├── irb920t_6_065.urdf
│   Modelo estructural utilizado por PyBullet.
│
├── workspace_plot.py
│   Visualización independiente del workspace XY.
│
├── environment.yml
│   Entorno Conda recomendado y reproducible.
│
├── requirements.txt
│   Dependencias para instalación alternativa mediante pip.
│
├── check_install.py
│   Comprueba que el computador esté correctamente configurado.
│
├── README.md
│   Documentación del proyecto.
│
├── .gitignore
│   Archivos que Git no debe almacenar.
│
├── .gitattributes
│   Configuración de finales de línea entre Windows/Linux/macOS.
│
└── LICENSE
    Licencia del código.
```

---

# Instalación alternativa con pip

Conda es el método recomendado.

Si el sistema dispone de una versión de PyBullet compatible con pip, se puede utilizar:

```bash
python -m venv .venv
```

### Windows

```powershell
.\.venv\Scripts\Activate.ps1
```

### Linux/macOS

```bash
source .venv/bin/activate
```

Después:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

y:

```bash
python irb920t_sim.py
```

> En Windows se recomienda Conda. Si PyBullet intenta compilarse y solicita Microsoft Visual C++ Build Tools, utiliza el método mediante `environment.yml`.

---

# Actualizar el entorno

Si posteriormente cambia `environment.yml`:

```bash
conda env update -f environment.yml --prune
```

---

# Eliminar el entorno

Si se desea eliminar completamente el entorno:

```bash
conda deactivate
```

y:

```bash
conda env remove -n irb920t
```

---

# Consideraciones del modelo

Este proyecto busca representar correctamente la arquitectura y cinemática principal del ABB IRB 920T-6/0.65.

Sin embargo:

- No utiliza el CAD oficial de ABB.
- Las carcasas son aproximaciones geométricas.
- Las masas e inercias de los eslabones no representan una identificación dinámica oficial.
- No debe utilizarse para análisis estructural, certificación industrial o programación directa de una celda ABB real.

Es un modelo orientado a:

```text
educación
robótica
cinemática
simulación
análisis de workspace
estudio de manipuladores SCARA
```

---

# Tecnologías

```text
Python 3.11
PyBullet
NumPy
Matplotlib
Tkinter
URDF
Git
```

---

# Licencia

Este proyecto se distribuye bajo la licencia MIT incluida en el archivo:

```text
LICENSE
```

ABB e IRB son marcas asociadas a sus respectivos propietarios.

Este repositorio es un proyecto educativo independiente y no está afiliado, patrocinado ni aprobado oficialmente por ABB.