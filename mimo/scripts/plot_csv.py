import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================
# Cargar datos
# ==========================
df = pd.read_csv('healthy_hip_reference.csv')

traj = df.iloc[:, 1].to_numpy()

N = len(traj)

# ==========================
# Cadera derecha
# Buscar el punto más cercano a 0°
# ==========================
idx_zero = np.argmin(np.abs(traj))

# Desfase circular para empezar en 0
hip_right = np.roll(traj, -idx_zero)

# ==========================
# Cadera izquierda
# Desfase adicional del 50%
# ==========================
half_cycle = N // 2

hip_left_shifted = np.roll(
    traj,
    -(idx_zero + half_cycle)
)

# ==========================
# Mantener en 0 hasta que
# la trayectoria desplazada
# pase por 0°
# ==========================
idx_zero_left = np.argmin(np.abs(hip_left_shifted))

hip_left = hip_left_shifted.copy()

# Todo lo anterior se fija a 0
hip_left[:idx_zero_left] = 0

# ==========================
# Repetir 3 ciclos
# ==========================
num_cycles = 3

# Cadera derecha: repetir directamente
hip_right_plot = np.tile(hip_right, num_cycles)

# Cadera izquierda:
# Primer ciclo con el tramo a 0
# Ciclos restantes continuos
hip_left_plot = np.concatenate([
    hip_left,
    np.tile(hip_left_shifted, num_cycles - 1)
])

# Eje X en porcentaje de marcha
x = np.linspace(
    0,
    100 * num_cycles,
    len(hip_right_plot)
)

# ==========================
# Gráfico
# ==========================
plt.figure(figsize=(12, 6))

plt.plot(
    x,
    hip_right_plot,
    label='Cadera derecha'
)

plt.plot(
    x,
    hip_left_plot,
    label='Cadera izquierda'
)

plt.xlabel('Porcentaje de paso (%)')
plt.ylabel('Ángulo de cadera (°)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()

plt.show()