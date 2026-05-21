# visualiza_standup_trajectory_tracking.py — Documentación

Este archivo documenta `visualiza_standup_trajectory_tracking.py` (ubicado en `examples/`) y explica su propósito, uso y formato esperado de datos.

**Resumen / propósito**
- `visualiza_standup_trajectory_tracking.py` genera una simulación de seguimiento de trayectoria para ambas caderas usando una referencia CSV, un controlador PID y el modelo muscular del entorno.
- Sirve para inspección visual y diagnóstico del seguimiento de cadera: compara la referencia con la simulación y guarda telemetría, figura resumen y metadatos.

**Script legado**
- `visualiza_standup.py` queda como flujo antiguo de reproducción de activaciones musculares. Si necesitas el simulador principal, usa `visualiza_standup_trajectory_tracking.py`.

Requisitos
- Python con dependencias del proyecto (ver `requirements.txt`).
- `mimoEnv` registrado e importable (el script hace `import mimoEnv`).
- `gymnasium`, `numpy`, `matplotlib`, `mimoActuation.muscle.MuscleModel` (opcional si el entorno usa MuscleModel).

Archivo principal y ruta
- Script: `examples/visualiza_standup_trajectory_tracking.py`
- README generado: `examples/visualiza_standup_README.md`

Configuración principal dentro del script
- `ENV_ID`: id del entorno Gym a usar. Por defecto en el script: `MIMoVelocityLowerBody-v0`.
- `REFERENCE_CSV`: ruta al CSV de referencia de cadera usado para el seguimiento. Por defecto apunta al CSV incluido junto al script.
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
2. Lee la referencia de cadera desde `REFERENCE_CSV`.
3. Ejecuta un controlador PID para generar activaciones musculares y seguir la trayectoria objetivo.
4. Ajusta el modelo mecánico con los factores de escala (`--mass-scale`, `--joint-stiffness-scale`, etc.) si se pasan por CLI.
5. Renderiza la simulación y registra ángulos, torque proxy y activaciones.
6. Al finalizar (o con Ctrl-C): guarda CSV de telemetría, imagen resumen y un archivo `simulation_summary.txt` con configuración y metadatos.

Salida generada (por ejecución)
- Carpeta: `code_tfg/outputs/mimo/new_workflow/simulator/<timestamp>_trajectory_tracking/`
  - `hip_tracking_telemetry.csv` — telemetría con ángulos, torque proxy y activaciones flex/ext.
  - `hip_tracking_summary.png` — figura PNG con ángulos, torque y activaciones.
  - `simulation_summary.txt` — metadatos, parámetros PID y configuración del modelo.

Nota sobre el nuevo flujo
- Este es el flujo principal del simulador. Los archivos se guardan por defecto en `code_tfg/outputs/mimo/new_workflow/simulator/<timestamp>_trajectory_tracking/`.

Ejemplo de ejecución del nuevo simulador (usa `--run-dir` para cambiar destino):
```bash
python3 examples/visualiza_standup_trajectory_tracking.py --reference-csv mimo/scripts/healthy_hip_reference.csv
```

Ejemplos de uso

1) Ejecutar con la referencia por defecto embebida en el script:

```bash
python examples/visualiza_standup_trajectory_tracking.py
```

2) Usar otra referencia CSV y ajustar el directorio de salida:

```bash
python examples/visualiza_standup_trajectory_tracking.py \
  --reference-csv mimo/scripts/healthy_hip_reference.csv \
  --run-dir code_tfg/outputs/mimo/new_workflow/simulator/test_run
```

3) Cambiar entorno o parámetros en la cabecera del script:

```python
ENV_ID = 'MIMoStandup-v0'
REFERENCE_CSV = 'mimo/scripts/healthy_hip_reference.csv'
TRAJECTORY_PERIOD_S = 4.0
MASS_SCALE = 1.0
```

Notas y recomendaciones
- Antes de ejecutar, verifica que `REFERENCE_CSV` apunte a un CSV de cadera válido.
- Si quieres reproducir el flujo antiguo, consulta `visualiza_standup.py` como referencia histórica.
- Para depuración, revisa los prints periódicos del seguimiento de trayectoria (error, activaciones y torque proxy).

Extensiones posibles
- Añadir argumento CLI para pasar `--reference-csv` y `--env` en vez de editar el archivo.
- Exportar la imagen y CSV con nombre determinista en función de la referencia origen.

---

## Seguimiento de trayectoria de cadera

Este es el flujo principal del simulador: `examples/visualiza_standup_trajectory_tracking.py` genera una simulación de seguimiento de trayectoria para ambas caderas usando una referencia CSV, un controlador PID y el modelo muscular del entorno.

### Qué genera

La ejecución guarda su salida por defecto en `code_tfg/outputs/mimo/new_workflow/simulator/<timestamp>_trajectory_tracking/`:

- `hip_tracking_telemetry.csv` — telemetría con ángulos, torque proxy y activaciones flex/ext de cada cadera.
- `hip_tracking_summary.png` — figura resumen con ángulos, torque y activaciones.
- `simulation_summary.txt` — un único TXT combinado con la configuración de ejecución, parámetros PID, parámetros del modelo y nombres de actuadores.

### Ejemplo de uso

```bash
python3 examples/visualiza_standup_trajectory_tracking.py \
  --reference-csv mimo/scripts/healthy_hip_reference.csv
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
