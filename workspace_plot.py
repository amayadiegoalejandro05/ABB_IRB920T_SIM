"""Grafica el espacio de trabajo XY real del modelo usando los límites J1/J2."""
import math
import numpy as np
import matplotlib.pyplot as plt

L1 = 0.260
L2 = 0.390
Q1 = np.deg2rad(np.linspace(-150, 150, 241))
Q2 = np.deg2rad(np.linspace(-145, 145, 241))

q1g, q2g = np.meshgrid(Q1, Q2)
x = L1*np.cos(q1g) + L2*np.cos(q1g + q2g)
y = L1*np.sin(q1g) + L2*np.sin(q1g + q2g)

plt.figure(figsize=(8, 8))
plt.scatter(x.ravel()*1000, y.ravel()*1000, s=0.35, alpha=0.35)
plt.scatter([0], [0], marker="x", s=80, label="Eje 1 / base")
plt.gca().set_aspect("equal", adjustable="box")
plt.xlabel("X [mm]")
plt.ylabel("Y [mm]")
plt.title("ABB IRB 920T-6/0.65 — Workspace XY con límites J1/J2")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
