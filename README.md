# Desarrollo y validación de una plataforma experimental para el entrenamiento de exoesqueletos robóticos mediante aprendizaje por refuerzo

> 🎓 **Trabajo de Fin de Grado (TFG)** — Grado en Ingeniería de Computadores, **Escuela Politécnica Superior, Universidad de Alcalá (UAH)**.
> Realizado en colaboración con el **grupo de investigación NAIR (CSIC)**.
> Calificación obtenida: **10 / 10**.

**Autor:** Carlos Eguren Esteban
**Tutora académica:** Ana Jiménez Martín (UAH)
**Cotutor:** David Rodríguez Cianca (NAIR, CSIC)
**Año:** 2026

La memoria completa del trabajo está disponible en este mismo repositorio: [`TFG-EgurenEsteban-Carlos.pdf`](./TFG-EgurenEsteban-Carlos.pdf).

---

## 📖 Descripción general

Los exoesqueletos robóticos buscan mejorar la calidad de vida de las personas, ya sea asistiendo la movilidad de usuarios con discapacidad o potenciando la fuerza y resistencia de usuarios sanos. Entrenar los algoritmos de control de estos dispositivos directamente sobre hardware real es inviable, ya que requiere un número muy elevado de interacciones y supone un riesgo para el usuario. Este proyecto aborda ese problema construyendo una **plataforma experimental completa** que permite entrenar, validar y desplegar controladores de exoesqueletos de forma segura, sin necesidad de un usuario humano durante las fases de entrenamiento.

La plataforma se apoya en tres piezas fundamentales:

1. **Un modelo musculoesquelético simplificado de cadera**, simulado en [SCONE](https://scone.software) (con motor de simulación [Hyfydy](https://hyfydy.com)) a través de un entorno Gym personalizado (`sconegym`), que reproduce la dinámica muscular de la marcha humana.
2. **Un "dummy" robótico** (maniquí físico) que imita la morfología y el comportamiento dinámico del cuerpo humano, controlado mediante motores reales **MD80 (MAB Robotics)**. El dummy actúa como paso intermedio entre la simulación y un exoesqueleto real, permitiendo validar los algoritmos de control sin poner en riesgo a un usuario y mitigando el conocido **_sim-to-real gap_**.
3. **Un agente de aprendizaje por refuerzo (SAC, Soft Actor-Critic)**, entrenado en simulación sobre el modelo musculoesquelético para asistir el patrón de marcha de un exoesqueleto de cadera, y posteriormente desplegado sobre el dummy físico combinando su salida con un controlador PID clásico.

El resultado es un flujo de trabajo completo **simulación → entrenamiento → transferencia a hardware real → validación**, junto con una interfaz gráfica (GUI) en ROS2 para operar y monitorizar el sistema físico en tiempo real.

```
 ┌─────────────────────┐      ┌──────────────────────┐      ┌───────────────────────┐
 │  Modelo musculo-     │      │   Agente RL (SAC)     │      │  Hardware real         │
 │  esquelético (SCONE  │ ───► │   entrenado con       │ ───► │  Motores MD80 + dummy  │
 │  / Hyfydy, sconegym) │      │   Stable-Baselines3   │      │  vía CANdle / ROS2     │
 └─────────────────────┘      └──────────────────────┘      └───────────────────────┘
        entrenamiento/               entrenamiento/                inferencia/ · interfaz/
                                                                       (control PID + RL)
```

---

## 📑 Tabla de contenidos

- [Estructura del repositorio](#-estructura-del-repositorio)
- [Módulos del proyecto](#-módulos-del-proyecto)
  - [`entrenamiento/`](#entrenamiento)
  - [`mimo/`](#mimo)
  - [`inferencia/`](#inferencia)
  - [`interfaz/`](#interfaz)
  - [`instalacion_entornos/`](#instalacion_entornos)
  - [`outputs/`](#outputs)
  - [`multimedia/`](#multimedia)
- [Instalación](#️-instalación)
- [Guía rápida de uso](#-guía-rápida-de-uso)
- [Hardware utilizado](#-hardware-utilizado)
- [Advertencias de seguridad](#️-advertencias-de-seguridad)
- [Tecnologías y librerías principales](#-tecnologías-y-librerías-principales)
- [Créditos y proyectos de terceros](#-créditos-y-proyectos-de-terceros)
- [Documentación completa](#-documentación-completa)
- [Licencia](#-licencia)

---

## 📂 Estructura del repositorio

```
code_tfg/
├── TFG-EgurenEsteban-Carlos.pdf   # Memoria completa del TFG (108 páginas)
├── entrenamiento/                 # Entrenamiento RL sobre el modelo musculoesquelético (SCONE)
│   └── sconegym/                  # Entorno Gym personalizado (sconegym + entornos "nair_*")
├── mimo/                          # Framework MIMo (simulación MuJoCo) + scripts sim-to-real
│   ├── mimoEnv/ mimoActuation/ …  # Módulos originales de MIMo
│   └── scripts/                   # Scripts de validación en simulación y en motor real
├── inferencia/                    # Inferencia en tiempo real: PID + agente SAC sobre motores reales
├── interfaz/                      # Workspace ROS2: control de motores MD80, GUI e interfaces de mensajes
│   └── src/ROS2/
│       ├── candle_ros2/           # Driver ROS2 de los motores MD80 (MAB Robotics)
│       ├── gui/                   # Interfaz gráfica (PyQt) de control y monitorización
│       ├── msg_interface/         # Mensajes ROS2 personalizados
│       └── srv_interface/         # Servicios ROS2 personalizados
├── instalacion_entornos/          # Entornos conda (.yml) y script de instalación de ROS2
├── multimedia/                    # Vídeos y figuras demostrativas del funcionamiento del sistema
└── outputs/                       # Resultados de las ejecuciones: modelos, logs, CSV, gráficas
    ├── entrenamiento/              # Checkpoints SAC, TensorBoard y telemetría de cada entrenamiento
    ├── inferencia/                 # Telemetría de las pruebas en el dummy real
    ├── interfaz/                   # Grabaciones (rosbags) y registros de la GUI
    └── mimo/                       # Resultados de los scripts de validación de mimo/scripts
```

> 💡 El repositorio integra tanto código propio del autor (marcado con el prefijo `nair_` en `sconegym`, los scripts de `entrenamiento/`, `inferencia/` y las extensiones de `mimo/scripts/`) como librerías de terceros adaptadas para este proyecto (`mimo/`, `sconegym` base y `candle_ros2`). Ver la sección [Créditos](#-créditos-y-proyectos-de-terceros).

---

## 🧩 Módulos del proyecto

### `entrenamiento/`

Contiene el flujo de **entrenamiento por refuerzo** del agente que controla el exoesqueleto de cadera, usando el modelo musculoesquelético simulado en SCONE.

- **`sconegym/`** — Fork/extensión del paquete [sconegym](https://github.com/tgeijten/sconegym), con entornos propios bajo el prefijo `nair_*` (`nair_gaitEnv.py`, `nair_mimoEnv.py`, `nair_sconegym.py`, `nair_motor_disorders.py`) y el modelo musculoesquelético de cadera con exoesqueleto en `sconegym/nair_envs/H0404_MimoExo/` (modelos `.hfd`/`.osim`, controladores en Lua, medidas y trayectorias de referencia).
- **`training.py`** — Script principal de entrenamiento. Entrena un agente **SAC** (Stable-Baselines3) sobre el entorno `nair_gait_h0404MimoExo-v0`, con wrappers de *domain randomization* (ruido en las observaciones, retardo de acciones) para mejorar la robustez ante la transferencia a hardware real. Guarda checkpoints, logs de TensorBoard y un CSV biomecánico (posiciones/velocidades de cadera, activaciones y fuerzas musculares, torque del exo, salida del PID) en cada checkpoint.
- **`example_mimo.py`** / **`example_mimo_terminal.py`** — Scripts de ejemplo para ejecutar y visualizar el entorno (con gráficas o con métricas por terminal: MAE, MSE, RMSE, NRMSE del seguimiento de cadera).
- **`plot_checkpoint.py`** / **`plot_monitor.py`** — Utilidades para graficar la evolución del entrenamiento (episodios y curva de recompensa de `monitor.csv`).

Lanzamiento típico de un entrenamiento (ver `lanzamiento_training.txt`):

```bash
conda activate scone
python training.py --total-timesteps 300000 --checkpoint-freq 48000 --save-scone-episodes --progress-bar
```

### `mimo/`

Integra el framework **[MIMo](https://github.com/trieschlab/MIMo)** (*Multimodal Infant Model*), un entorno Gymnasium sobre MuJoCo desarrollado originalmente para estudiar el desarrollo cognitivo infantil. En este proyecto se reutiliza y se **extiende con scripts propios** para el control de motores de cadera y rodilla en simulación y en hardware real:

- **`mimoEnv/`, `mimoActuation/`, `mimoGrowth/`, `mimoProprioception/`, `mimoTouch/`, `mimoVestibular/`, `mimoVision/`** — Módulos originales de MIMo (modelo corporal, actuación muscular, crecimiento, propiocepción, tacto, sistema vestibular y visión).
- **`mimo/scripts/`** — Scripts propios de este TFG:
  - `visualiza_standup_trajectory_tracking.py` — Simulador de seguimiento de trayectoria de ambas caderas mediante un controlador PID + modelo muscular, usado para validar el seguimiento antes de pasar a hardware real.
  - `hip_motor_torque_replay_runner*.py` (variantes ROS2 / pyCandle / con control de impedancia) — Reproducen sobre el motor real el par calculado en simulación.
  - `plot_sim.py`, `plot_real.py`, `plot_real_1dof.py`, `plot_csv.py` — Visualización y comparación de resultados simulados vs. reales.
  - `README_sim.md` y `README_real.md` — Documentación detallada de parámetros (ganancias PID, escalado de masa/rigidez/amortiguamiento, límites de par, estrategias de seguridad, etc.).

### `inferencia/`

Contiene el script de **despliegue final del controlador híbrido PID + RL sobre los motores reales**:

- **`inferencia.py`** — Carga un agente SAC entrenado y una trayectoria de referencia de marcha sana (`healthy_hip_reference.csv`). En cada ciclo de control calcula un par mediante un **PID clásico** y un par mediante el **agente SAC** (ejecutado en un proceso aislado para evitar conflictos entre PyTorch y las librerías nativas de los motores), combina ambos y los envía a los motores de cadera reales (IDs `348`/`349`) a través de **pyCandle** o de **ROS2** (`candle_ros2`). Incluye límites de seguridad de par, registro de telemetría en CSV y generación automática de gráficas de seguimiento.
- **`plot2.py`, `plot_results.py`** — Comparativas de seguimiento de trayectoria y métricas de error (posición, velocidad, par) entre el modo solo-PID y el modo PID + agente.

### `interfaz/`

Workspace de **ROS2** (probado en Foxy/Jazzy) que implementa el control de bajo nivel del exoesqueleto/dummy y su interfaz de usuario:

| Paquete | Función |
|---|---|
| `src/ROS2/candle_ros2/` | Driver ROS2 de los motores **MD80** (MAB Robotics) sobre bus CAN vía CANdle: control de posición, velocidad, impedancia y par, y publicación del estado de las articulaciones. |
| `src/ROS2/gui/` | Interfaz gráfica (PyQt6) para habilitar/deshabilitar motores, elegir modo de control (PID de posición, PID de velocidad o impedancia), enviar trayectorias sinusoidales, visualizar señales en tiempo real y grabar experimentos (`rosbag`). |
| `src/ROS2/msg_interface/` y `src/ROS2/srv_interface/` | Mensajes y servicios ROS2 personalizados (p. ej. datos de plantillas/insoles, parámetros de impedancia). |

Scripts de lanzamiento incluidos: `launch_gui.sh`, `launch_gui_completa.sh`, `launch_gui_impedance.sh`. La documentación `README.md`/`README_nuevo.md` de esta carpeta incluye, además, una guía completa de grabación y análisis de datos con `ros2 bag` y **PlotJuggler**.

### `instalacion_entornos/`

Recoge los **entornos reproducibles** usados a lo largo del proyecto:

| Entorno (`.yml`) | Python | Uso principal |
|---|---|---|
| `scone.yml` | 3.9 | Entrenamiento RL con `sconegym` + Stable-Baselines3 |
| `mimo.yml` | 3.11 | Simulación con MIMo/MuJoCo |
| `mimo312.yml` | 3.12 | Ejecución en hardware real (incluye `pycandlemab`) |
| `ros2rl.yml` | 3.10 | GUI (PyQt6) + ROS2 + inferencia RL combinadas |

También incluye `install_ros2.sh` (instala ROS2 Jazzy y sus dependencias APT a partir de `ros2-system-packages.txt`).

### `outputs/`

Carpeta de resultados generados por las distintas ejecuciones (no es código, sino datos experimentales):

- `outputs/entrenamiento/<timestamp>/` — Más de 30 ejecuciones de entrenamiento, cada una con los checkpoints del agente SAC (`sac_mimo_*_steps.zip`, `sac_mimo_final.zip`), logs de **TensorBoard**, `monitor.csv` y CSV biomecánicos por checkpoint.
- `outputs/inferencia/<timestamp>/` — Telemetría y gráficas de las pruebas de inferencia sobre el dummy real.
- `outputs/mimo/` — Resultados de los scripts de validación en simulación y de los *runners* sobre motor real.
- `outputs/interfaz/` — Registros generados desde la GUI (rosbags, configuraciones de sesión).

### `multimedia/`

Vídeos y figuras que documentan visualmente el funcionamiento del sistema: el agente de RL controlando el exoesqueleto sobre el modelo en SCONE, el control del dummy físico a partir del modelo musculoesquelético, el seguimiento de trayectoria en MuJoCo/SCONE y el control de los motores reales a partir del modelo.

---

## ⚙️ Instalación

El proyecto usa **varios entornos conda independientes** según la etapa del pipeline (entrenamiento, simulación MIMo, ejecución en hardware real y GUI/ROS2), definidos en `instalacion_entornos/`.

1. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/cegurene/code_tfg.git
   cd code_tfg
   ```

2. **Crear los entornos conda necesarios** (puedes crear solo los que necesites según la etapa en la que vayas a trabajar):
   ```bash
   conda env create -f instalacion_entornos/scone.yml      # Entrenamiento RL (SCONE + sconegym)
   conda env create -f instalacion_entornos/mimo.yml       # Simulación con MIMo
   conda env create -f instalacion_entornos/mimo312.yml    # Hardware real (pyCandle)
   conda env create -f instalacion_entornos/ros2rl.yml     # GUI + ROS2 + RL
   ```

3. **Instalar ROS2** (necesario para `interfaz/` y para el modo `--interface ros2` de `inferencia/`):
   ```bash
   ROS_DISTRO=jazzy ./instalacion_entornos/install_ros2.sh
   ```

4. **Instalar SCONE / Hyfydy** siguiendo las instrucciones de [scone.software](https://scone.software) (necesario para `entrenamiento/sconegym`).

5. **Instalar los paquetes propios en modo editable** dentro del entorno correspondiente, por ejemplo:
   ```bash
   conda activate scone
   cd entrenamiento && pip install -r requirements.txt && pip install -e sconegym/

   conda activate mimo
   cd ../mimo && pip install -r requirements.txt && pip install -e .
   ```

6. **Compilar el workspace ROS2** (`interfaz/`):
   ```bash
   conda activate ros2rl
   cd interfaz
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 🚀 Guía rápida de uso

**Entrenar el agente de RL en simulación:**
```bash
conda activate scone
cd entrenamiento
python training.py --total-timesteps 300000 --checkpoint-freq 48000 --save-scone-episodes --progress-bar
```

**Visualizar el seguimiento de trayectoria en simulación (MIMo):**
```bash
conda activate mimo
python mimo/scripts/visualiza_standup_trajectory_tracking.py --reference-csv mimo/scripts/healthy_hip_reference.csv
```

**Lanzar el nodo de motores reales (ROS2 + CANdle):**
```bash
conda activate ros2rl
source interfaz/install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```

**Ejecutar la inferencia híbrida PID + agente SAC sobre los motores reales:**
```bash
conda activate mimo312
python inferencia/inferencia.py \
  --agent-path outputs/entrenamiento/<timestamp>/sac_mimo_final.zip \
  --source-csv mimo/scripts/healthy_hip_reference.csv \
  --interface pycandle
```

**Lanzar la interfaz gráfica de control:**
```bash
cd interfaz
./launch_gui.sh
```

---

## 🦿 Hardware utilizado

- **Motores MD80** de [MAB Robotics](https://www.mabrobotics.pl/), controlados vía bus CAN mediante el adaptador **CANdle** (interfaces `pyCandle` y `candle_ros2`).
- **Dummy robótico de pruebas**: estructura física que reproduce las dimensiones, el peso y los límites articulares de una cadera humana, usada como paso intermedio seguro entre la simulación y un futuro exoesqueleto real con usuario.

## ⚠️ Advertencias de seguridad

Este repositorio incluye scripts que **envían par directamente a motores reales**. Antes de ejecutar cualquier script de `inferencia/` o `mimo/scripts/` sobre hardware:

- Empieza siempre con un *dry run* (sin `--apply-torque`) para comprobar la comunicación y la trayectoria de referencia.
- Usa límites de par bajos (`--safe-torque-limit`) y aumenta progresivamente.
- Ten siempre accesible una parada de emergencia (Ctrl+C o física).
- No reproduzcas rosbags (`ros2 bag play`) con el motor conectado sin supervisión.

---

## 🛠 Tecnologías y librerías principales

| Categoría | Herramientas |
|---|---|
| Simulación musculoesquelética | [SCONE](https://scone.software) / Hyfydy, [sconegym](https://github.com/tgeijten/sconegym) |
| Simulación física / infantil | [MIMo](https://github.com/trieschlab/MIMo) sobre [MuJoCo](https://mujoco.readthedocs.io) |
| Aprendizaje por refuerzo | [Gymnasium](https://github.com/Farama-Foundation/Gymnasium), [Stable-Baselines3](https://stable-baselines3.readthedocs.io) (algoritmo **SAC**), PyTorch |
| Robótica / hardware real | ROS2, MD80 + CANdle (MAB Robotics), pyCandle |
| Interfaz de usuario | PyQt6, Qt Designer |
| Análisis y visualización | pandas, NumPy, Matplotlib, TensorBoard, PlotJuggler |

## 🙏 Créditos y proyectos de terceros

Este trabajo integra y adapta varios proyectos de código abierto, cuya autoría original se reconoce aquí:

- **[MIMo](https://github.com/trieschlab/MIMo)** — Mattern, Schumacher, López, Raabe, Ernst, Aubret y Triesch. Licencia MIT (ver `mimo/LICENSE`).
- **[sconegym](https://github.com/tgeijten/sconegym)** — Pierre Schumacher y Thomas Geijtenbeek. Licencia Apache 2.0.
- **[candle_ros2](https://github.com/mabrobotics)** — MAB Robotics (driver de los motores MD80).
- Creación de modelo en Scone realizada por [Andrés Chavarrías](https://github.com/AndresChS).

El resto del código (entornos `nair_*` de sconegym, scripts de entrenamiento e inferencia, scripts de transferencia sim-to-real de `mimo/scripts/`, GUI del exoesqueleto e integración general de la plataforma) es autoría de **Carlos Eguren Esteban**, desarrollado en el marco de este TFG dentro del grupo **NAIR (CSIC)**.

## 📚 Documentación completa

Para una descripción detallada del estado del arte, la metodología, los resultados experimentales (seguimiento de la marcha, comparación PID vs. PID+RL, validación en el dummy) y las conclusiones, consulta la memoria completa del TFG:

📄 [`TFG-EgurenEsteban-Carlos.pdf`](./TFG-EgurenEsteban-Carlos.pdf)

## 📄 Licencia

Este repositorio combina código propio con librerías de terceros bajo distintas licencias (MIT para MIMo, Apache 2.0 para sconegym, licencias propias de MAB Robotics para `candle_ros2`, etc.). Consulta el archivo `LICENSE` de cada subcarpeta correspondiente antes de reutilizar el código. El código propio del autor se comparte con fines académicos y de investigación.
