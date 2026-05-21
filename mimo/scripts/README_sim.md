# visualiza_standup.py — Documentación

Este archivo documenta `visualiza_standup.py` (ubicado en `examples/`) y explica su propósito, uso y formato esperado de datos.

**Resumen / propósito**
- `visualiza_standup.py` reproduce señales de activación musculares (telemetry CSV) sobre un entorno MuJoCo/Gym personalizado de MIMo y visualiza las trayectorias de cadera y activaciones musculares.
- Sirve para inspección visual y diagnóstico de runs reales o simulados: aplica las activaciones leídas del CSV al modelo de actuación del entorno y genera una imagen con las últimas ~20s de datos y metadatos.

Requisitos
- Python con dependencias del proyecto (ver `requirements.txt`).
- `mimoEnv` registrado e importable (el script hace `import mimoEnv`).
- `gymnasium`, `numpy`, `matplotlib`, `mimoActuation.muscle.MuscleModel` (opcional si el entorno usa MuscleModel).

Archivo principal y ruta
- Script: `examples/visualiza_standup.py`
- README generado: `examples/visualiza_standup_README.md`

Configuración principal dentro del script
- `ENV_ID`: id del entorno Gym a usar. Por defecto en el script: `MIMoVelocityLowerBody-v0`.
- `CSV_PATH`: ruta al CSV de telemetry que contiene activaciones musculares. Ejemplo: `results/hip_motor_real/.../telemetry.csv`.
- `DESFASE_CADERA`: desfase entre señales izquierda/derecha (fracción del ciclo, 0.5 = 50%).
- `CAM_DISTANCIA`: distancia de cámara (visualización).
- `HIP_SIGN_LEFT`, `HIP_SIGN_RIGHT`: 1 o -1 para invertir la polaridad de la señal de cadera si la trayectoria aparece invertida.
- `MASS_SCALE`, `FMAX_SCALE`: factores de escala aplicables en runtime para masas y fmax respectivamente.

Formato de CSV esperado
- El script espera un CSV con columnas (cabecera) que incluyan al menos:
  - `u_flex_0`, `u_flex_1`, `u_flex_2`, `u_ext_0`, `u_ext_1`, `u_ext_2` (seis columnas por cadera)
- Cuando el script lee `run_config.txt` del directorio del CSV, intentará extraer bloques `actuator:` con `source_units_neg` / `source_units_pos` o las líneas `flex_muscles:` / `ext_muscles:` para obtener `fmax`/`vmax` por unidad y ponderar combinaciones.

Qué hace el script (pasos clave)
1. Carga `mimoEnv` y crea el entorno `ENV_ID` en modo `render_mode='human'`.
2. Lee las activaciones desde `CSV_PATH` (columnas `u_flex_*` y `u_ext_*`).
3. Construye señales para ambas caderas: la original (derecha) y otra desfasada (`DESFASE_CADERA`) para simular marcha alternante.
4. Intenta aplicar matrices `fmax`/`vmax` extraídas de `run_config.txt` (si existe) al `env.actuation_model` cuando es instancia de `MuscleModel`.
5. En cada paso de simulación: combina las 3 unidades flex/ext en una sola señal por cadera (promedio o ponderado por `fmax`), calcula torque proxy y mapea ese torque/activación al vector `action` esperado por el entorno (varias heurísticas según `action_space` y modelo de actuacion).
6. Renderiza la simulación y registra ángulos/activaciones en memoria.
7. Al finalizar (o con Ctrl-C): guarda CSV de ángulos, imagen con los últimos ~20s y un archivo `simulation_info.txt` con metadatos y análisis rápido (frecuencia dominante, picos, etc.).

Salida generada (por ejecución)
- Carpeta: `results/simulation/<timestamp>_simulacion/`
  - `hip_angles_log.csv` — CSV con ángulos registrados y activaciones usadas.
  - `hip_and_activations.png` — figura PNG con ángulos y activaciones (últimos ~20s).
  - `simulation_info.txt` — metadatos y análisis rápido (frecuencias, picos, configuraciones aplicadas).

Nota sobre el nuevo flujo
- Los nuevos scripts de simulación usan por defecto la carpeta `results/new_workflow/simulator/<timestamp>_trajectory_tracking/`. Ahí se encuentra el `hip_tracking_telemetry.csv`, `hip_tracking_summary.png`, `run_config.txt` y `simulation_info.txt`.

Ejemplo de ejecución del nuevo simulador (usa `--run-dir` para cambiar destino):
```bash
python3 examples/visualiza_standup_trajectory_tracking.py --reference-csv mimoEnv/exo_motor_custom/healthy_hip_reference.csv
```

Ejemplos de uso

1) Ejecutar con el CSV por defecto embebido en el script (ajusta `CSV_PATH` antes):

```bash
python examples/visualiza_standup.py
```

2) Visualización rápida desde otro CSV (editar `CSV_PATH` al inicio del script) y luego ejecutar:

```bash
# editar examples/visualiza_standup.py: CSV_PATH = 'results/hip_motor_real/.../telemetry.csv'
python examples/visualiza_standup.py
```

3) Cambiar entorno o parámetros en la cabecera del script:

```python
ENV_ID = 'MIMoStandup-v0'
CSV_PATH = 'results/hip_motor_real/.../telemetry.csv'
DESFASE_CADERA = 0.5
MASS_SCALE = 1.0
```

Notas y recomendaciones
- Antes de ejecutar, verifica que `CSV_PATH` apunte a un `telemetry.csv` compatible (tiene cabecera con `u_flex_*` y `u_ext_*`).
- Si el entorno no expone `env.actuation_model` compatible o las matrices `fmax`/`vmax`, el script continuará usando un fallback de medias.
- Si usas `MUSCLE` actuation model asegúrate de que `env.action_space` acepte vectores `2*n_actuators` (neg/pos) o el script aplicará heurísticas de mapeo.
- Para depuración, revisa los prints periódicos (cada `PRINT_EVERY` steps) que muestran índices de actuadores, valores enviados y estimaciones de torque.

Extensiones posibles
- Añadir argumento CLI para pasar `--csv` y `--env` en vez de editar el archivo.
- Exportar la imagen y CSV con nombre determinista en función del CSV origen (actualmente usa timestamp por ejecución).

---

## Seguimiento de trayectoria de cadera

Este repositorio también incluye el script `examples/visualiza_standup_trajectory_tracking.py`, que genera una simulación de seguimiento de trayectoria para ambas caderas usando una referencia CSV, un controlador PID y el modelo muscular del entorno.

### Qué genera

La ejecución guarda su salida por defecto en `results/new_workflow/simulator/<timestamp>_trajectory_tracking/`:

- `hip_tracking_telemetry.csv` — telemetría con ángulos, torque proxy y activaciones flex/ext de cada cadera.
- `hip_tracking_summary.png` — figura resumen con ángulos, torque y activaciones.
- `simulation_summary.txt` — un único TXT combinado con la configuración de ejecución, parámetros PID, parámetros del modelo y nombres de actuadores.

### Ejemplo de uso

```bash
python3 examples/visualiza_standup_trajectory_tracking.py \
  --reference-csv mimoEnv/exo_motor_custom/healthy_hip_reference.csv
```

### Ritmo de la trayectoria

El argumento `--trajectory-period-s` define el período completo de la referencia en segundos.

- Si lo subes, la trayectoria avanza más despacio y la simulación dura más.
- Si lo bajas, la trayectoria avanza más rápido y la simulación dura menos.

En este script no hay un segundo argumento separado para la velocidad del CSV: el período de la trayectoria es el control principal del ritmo.

### Parámetros de control PID

La simulación usa un controlador PID para convertir el error de posición en activación muscular.

#### `--kp FACTOR` (por defecto: 1.8)
- Ganancia proporcional.
- Si sube, la respuesta es más agresiva y corrige antes el error instantáneo.
- Si baja, la respuesta es más suave pero también más lenta.
- Si es demasiado alto, puede haber oscilación o sobreimpulso.

#### `--ki FACTOR` (por defecto: 0.15)
- Ganancia integral.
- Si sube, corrige mejor errores persistentes y reduce el error estacionario.
- Si baja, el control es más conservador, pero puede dejar sesgo residual.
- Si es demasiado alto, puede acumular demasiado y saturar.

#### `--kd FACTOR` (por defecto: 0.08)
- Ganancia derivativa.
- Si sube, amortigua más la respuesta y reduce oscilaciones.
- Si baja, la respuesta es más rápida pero menos estable.
- Si es demasiado alto, puede volver el control muy sensible al ruido.

### Parámetros de modelo mecánico

Todos estos parámetros usan escalado multiplicativo. Valor `1.0` significa sin cambios.

#### `--mass-scale FACTOR`
- Escala todas las masas del modelo.
- Subirlo hace el sistema más pesado y lento.
- Bajarlo hace el sistema más ligero y fácil de mover.

#### `--joint-stiffness-scale FACTOR`
- Escala la rigidez pasiva de las articulaciones.
- Subirlo hace las articulaciones más rígidas y estables.
- Bajarlo hace las articulaciones más flexibles y laxas.

#### `--joint-damping-scale FACTOR`
- Escala el amortiguamiento viscoso.
- Subirlo reduce oscilaciones pero vuelve el movimiento más pesado.
- Bajarlo hace la respuesta más viva, pero con más riesgo de oscilación.

#### `--joint-frictionloss-scale FACTOR`
- Escala la fricción estática/cinética de las articulaciones.
- Subirlo aumenta el arrastre y el torque necesario para mover.
- Bajarlo hace el movimiento más fluido y con menos resistencia.

#### `--joint-armature-scale FACTOR`
- Escala la inercia rotacional de los motores articulares.
- Subirlo hace el motor más lento para acelerar y frenar.
- Bajarlo lo hace más reactivo.

### Regla rápida de ajuste

- Sube `kp` si la cadera sigue la referencia con demasiado retraso.
- Sube `ki` si queda un error constante.
- Sube `kd` si aparecen oscilaciones.
- Sube `mass-scale`, `joint-stiffness-scale`, `joint-damping-scale`, `joint-frictionloss-scale` o `joint-armature-scale` para hacer el sistema más pesado o más difícil de mover.
- Bájalos para hacer el sistema más ligero, rápido o fluido.

### Ejemplos prácticos

```bash
# Más peso y algo más de rigidez
python3 examples/visualiza_standup_trajectory_tracking.py \
  --mass-scale 1.3 \
  --joint-stiffness-scale 1.2

# Más amortiguamiento y menor fricción
python3 examples/visualiza_standup_trajectory_tracking.py \
  --joint-damping-scale 1.5 \
  --joint-frictionloss-scale 0.8

# Control más agresivo
python3 examples/visualiza_standup_trajectory_tracking.py \
  --kp 2.3 --ki 0.18 --kd 0.12
```

### Nota sobre el README raíz

La documentación detallada de este flujo se mantiene aquí, en `examples/README.md`, para no mezclarla con la introducción general del proyecto.

Contacto
- Si quieres que expanda el README con ejemplos de CSV concretos, o que añada opciones CLI, dime qué flags prefieres y lo incorporo.
