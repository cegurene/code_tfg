# Assistive Exoskeleton Workspace - Guía de Uso

## 📋 Contenido
- [Requisitos Previos](#requisitos-previos)
- [Configuración Inicial](#configuración-inicial)
- [Lanzamiento del Sistema](#lanzamiento-del-sistema)
- [Grabación de Datos](#grabación-de-datos)
- [Análisis de Datos](#análisis-de-datos)
- [Solución de Problemas](#solución-de-problemas)

---

## 🔧 Requisitos Previos

### Hardware
- Motor MD80 con driver Candle
- Conexión USB al adaptador Candle

### Software
- ROS2 Jazzy
- Python 3
- PyQt5
- Paquete `candle_ros2`
- Plotjuggler (opcional, para visualización)

```bash
# Instalar Plotjuggler (recomendado para análisis)
sudo apt install ros-jazzy-plotjuggler-ros
```

---

## ⚙️ Configuración Inicial

### 1. Compilar el workspace
```bash
cd /directorio_base
colcon build
source install/setup.bash
```

### 2. Verificar conexión del motor
```bash
# Lanzar nodo candle
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```

```bash
# Ping al motor
mdtool ping
```

---

## 🚀 Lanzamiento del Sistema

### Opción 1: Lanzamiento completo

#### **Terminal 1: Nodo del motor**
```bash
cd /home/carlos/Escritorio/ros2_candle_ws
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```

#### **Terminal 2: Interfaz gráfica**
```bash
cd /home/carlos/Escritorio/assistive_exo_ws
./launch_gui.sh
```

### Opción 2: Lanzamiento sólo del modo impedancia

#### **Terminal 1: Nodo del motor**
```bash
cd /home/carlos/Escritorio/ros2_candle_ws
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```

#### **Terminal 2: Interfaz gráfica**
```bash
cd /home/carlos/Escritorio/assistive_exo_ws
source install/setup.bash
ros2 run gui gui_assistive_exo
```

### Opción 3: Nueva interfaz completa

Si vas a crear una segunda GUI en `interfaz/src/ROS2/gui/gui/interfaz_completa`, el flujo recomendado es:

1. Copiar la base visual de la interfaz actual a una nueva carpeta.
2. Renombrar el archivo `.ui` a algo como `gui_completa.ui`.
3. Editarlo en Qt Designer y regenerar el Python con `pyuic6`.
4. Copiar o adaptar el backend a `gui_completa.py`.
5. Registrar un nuevo entry point en `src/ROS2/gui/setup.py`.
6. Compilar de nuevo el paquete `gui` con `colcon build --packages-select gui`.

Lanzamiento esperado una vez creado el nuevo módulo:

```bash
cd /home/carlos/Escritorio/TFG/code_tfg/interfaz
./launch_gui_completo.sh
```

Y, si prefieres lanzarlo sin script:

```bash
source install/setup.bash
ros2 run gui gui_completa
```

Notas prácticas:
- La UI fuente debe ser la `.ui`; el `.py` generado por `pyuic6` se regenera siempre que cambies el diseño.
- Si el backend nuevo usa imports relativos, mantenlo dentro de la carpeta `interfaz_completa` para no mezclarlo con `gui_assistive_exo`.
- El script nuevo debe apuntar al entry point que registres en `setup.py`.

### ✅ Verificación del Sistema

El sistema está funcionando correctamente si:
1. **Terminal 1** muestra: `Candle initialized successfully`
2. **Terminal 2** abre la interfaz gráfica sin errores
3. Los topics están activos:
```bash
# En una terminal nueva
ros2 topic list
# Deberías ver:
# /md80/impedance_command
# /md80/joint_states
# /md80/motion_command
# /md80/position_pid_command
# /md80/velocity_pid_command
# /parameter_events
# /rosout
```

### 🎮 Uso Básico de la GUI

1. **Añadir motores**: Botón "Add Motors" → espera confirmación
2. **Habilitar motores**: Botón "Enable Motors"
3. **Seleccionar modo de control**:
   - Position PID
   - Velocity PID
   - Impedance Control
4. **Enviar comandos**: Usa los sliders o trayectorias sinusoidales

---

## 📹 Grabación de Datos

### 🎬 Iniciar Grabación

#### **Desde la GUI (Recomendado)**

1. **Configurar nombre del archivo**:
   - Campo "Nombre archivo": `experimento_01`
   - El sistema añadirá automáticamente numeración secuencial (`_001`, `_002`, etc.)

2. **Seleccionar carpeta de guardado** (opcional):
   - Clic en botón "Seleccionar Carpeta"
   - Por defecto usa: `/home/carlos/Escritorio/assistive_exo/src/rosbags/rosbags`

3. **Iniciar grabación**:
   - Clic en botón "Start Recording"
   - Ejecuta tus pruebas/movimientos

4. **Detener grabación**:
   - Clic en botón "Stop Recording"

#### **Desde Terminal (Grabación selectiva)**

```bash
# Terminal 3: Grabación de rosbag
cd /home/carlos/Escritorio/assistive_exo_ws/src/rosbags/rosbags

# Grabar TODOS los topics (igual que la GUI)
ros2 bag record -a -o mi_experimento

# Grabar solo topics específicos (más eficiente)
ros2 bag record \
  /md80/joint_states \
  /md80/motion_command \
  /md80/impedance_command \
  -o mi_experimento
```

### 📊 Qué se Graba

Cuando grabas con `-a` (todos los topics), se captura:

| Topic | Tipo | Contenido | Frecuencia |
|-------|------|-----------|------------|
| `/md80/joint_states` | `sensor_msgs/JointState` | **Posiciones, velocidades, torques reales** (radianes) | ~100 Hz |
| `/md80/joint_states_deg` | `sensor_msgs/JointState` | **Posiciones, velocidades en grados** (generado por converter) | ~100 Hz |
| `/md80/motion_command` | `candle_ros2/MotionCommand` | **Comandos enviados** (setpoints) | Variable |
| `/md80/impedance_command` | `candle_ros2/ImpedanceCommand` | **Parámetros de impedancia** (kp, kd) | Variable |
| `/rosout` | `rcl_interfaces/Log` | Logs del sistema | Variable |

#### **Nota sobre unidades**
- Por defecto, `/md80/joint_states` guarda **posiciones en radianes** y **velocidades en rad/s**
- Se publica automáticamente `/md80/joint_states_deg` con los mismos datos en **grados** y **deg/s**
- Para grabar en grados: lanza el nodo conversor (ver sección "Conversor de Unidades")

### 💾 Organización de Archivos

Estructura recomendada:
```
rosbags/
├── experimento_001/
│   ├── experimento_001_0.mcap
│   └── metadata.yaml
├── experimento_002/
│   ├── experimento_002_0.mcap
│   └── metadata.yaml
└── ...
```

**Buena práctica**: Crear un archivo de texto con cada grabación:
```bash
# En el directorio del rosbag
echo "Fecha: $(date)" > README.txt
echo "Modo: Impedance Control" >> README.txt
echo "Parámetros: kp=50, kd=2.5, offset=1.2" >> README.txt
echo "Descripción: Prueba de seguimiento sinusoidal 0.5Hz" >> README.txt
```

---

## � Conversor de Unidades (Radianes ↔ Grados)

Puedes grabar datos tanto en **radianes** (por defecto) como en **grados**. El conversor genera automáticamente versiones en grados de los topics principales.

### Topics generados por el conversor

| Topic Original | Topic en Grados | Contenido |
|---|---|---|
| `/md80/joint_states` | `/md80/joint_states_deg` | Posiciones y velocidades **actuales** en grados |
| `/md80/motion_command` | `/md80/motion_command_deg` | Posiciones y velocidades **objetivo** (setpoints) en grados |

### Lanzar conversor para grabar en grados

```bash
# Terminal 3: Lanzar conversor (opcional)
cd /home/carlos/Escritorio/assistive_exo_ws
source install/setup.bash
ros2 run joint_states_converter joint_states_converter_node
```

Luego, al grabar rosbag, incluye todos los topics:

```bash
# Grabar incluyendo datos en radianes Y grados
ros2 bag record \
  /md80/joint_states \
  /md80/joint_states_deg \
  /md80/motion_command \
  /md80/motion_command_deg \
  -o mi_experimento
```

### Visualización en Plotjuggler

En Plotjuggler, verás los topics duplicados (radianes y grados):

**Posiciones actuales:**
- `/md80/joint_states/position[0]` → radianes
- `/md80/joint_states_deg/position[0]` → grados

**Setpoints enviados:**
- `/md80/motion_command/target_position[0]` → radianes
- `/md80/motion_command_deg/target_position[0]` → grados

**Ejemplo: comparar seguimiento en grados**
```
Arrastrar al gráfico:
1. /md80/joint_states_deg/position[0]    (posición actual en grados)
2. /md80/motion_command_deg/target_position[0]  (setpoint en grados)
→ Verás cómo sigue el motor el comando
```

---

## �🔍 Análisis de Datos

### 1️⃣ Información Básica del Rosbag

```bash
cd /home/carlos/Escritorio/assistive_exo_ws/src/rosbags/rosbags

# Ver información general
ros2 bag info experimento_001

# Salida mostrará:
# - Duración total
# - Número de mensajes
# - Topics grabados
# - Frecuencia de cada topic
```

### 2️⃣ Visualización Gráfica con Plotjuggler ⭐ (RECOMENDADO)

Nota: si usas VS Code instalado por Snap, lanza Plotjuggler desde una terminal del sistema (fuera de VS Code) para evitar errores de librerias (GLIBC/Snap).

```bash
# Lanzar Plotjuggler
source /opt/ros/jazzy/setup.bash
ros2 run plotjuggler plotjuggler

# Dentro de Plotjuggler:
# 1. File → Load Rosbag...
# 2. Seleccionar: experimento_001/experimento_001_0.mcap
# 3. Arrastrar variables al gráfico:
#    - /md80/joint_states/position[0]  (posición motor izquierdo)
#    - /md80/joint_states/position[1]  (posición motor derecho)
#    - /md80/joint_states/velocity[0]  (velocidad)
#    - /md80/joint_states/effort[0]    (torque)
#    - /md80/motion_command/target_position[0]  (comando enviado)
```

**Gráficas útiles**:
- **Seguimiento**: Superponer `target_position` vs `position` real
- **Error de seguimiento**: Crear plot con diferencia
- **Torque vs tiempo**: Ver esfuerzo del motor
- **Comparación L/R**: Posiciones de ambos motores sincronizadas

### 3️⃣ Reproducción Controlada (⚠️ PRECAUCIÓN)

```bash
# ❌ NO HACER con motor conectado:
ros2 bag play experimento_001

# ✅ HACER: Ver solo estados (sin enviar comandos)
ros2 bag play experimento_001 --topics /md80/joint_states /rosout

# ✅ HACER: Reproducir sin motor físico conectado
# (útil para debugging de la GUI sin hardware)
```

**⚠️ ADVERTENCIA**: `ros2 bag play` con `/md80/motion_command` activo puede provocar movimientos peligrosos si:
- Los parámetros actuales (kp, kd) son diferentes a los de la grabación
- La posición inicial del motor es diferente
- No hay validaciones de seguridad

### 4️⃣ Exportar a CSV para Análisis Externo

#### Método A: Usando `ros2 topic echo`
```bash
# Reproducir el bag en una terminal
ros2 bag play experimento_001 &

# En otra terminal, exportar a CSV
ros2 topic echo /md80/joint_states --csv > joint_states.csv
ros2 topic echo /md80/motion_command --csv > motion_commands.csv

# Detener el bag
killall ros2
```

#### Método B: Script Python (TODO: crear script de análisis)
```bash
# Futuro: script automático de análisis
python3 analyze_rosbag.py experimento_001
```

### 5️⃣ Análisis en Python/Matlab

Una vez exportado a CSV, puedes usar:

**Python (pandas + matplotlib)**:
```python
import pandas as pd
import matplotlib.pyplot as plt

# Cargar datos
joint_states = pd.read_csv('joint_states.csv')

# Graficar posición
plt.plot(joint_states['header.stamp.sec'], joint_states['position[0]'])
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición (rad)')
plt.title('Trayectoria Motor Izquierdo')
plt.show()
```

**Matlab/Octave**:
```matlab
data = readtable('joint_states.csv');
plot(data.header_stamp_sec, data.position_0_);
xlabel('Tiempo (s)');
ylabel('Posición (rad)');
title('Trayectoria Motor Izquierdo');
```

---

## 🛠️ Solución de Problemas

### Problema: Error "Candle not found"
```bash
# Verificar permisos USB
sudo usermod -aG dialout $USER
# Logout/login para aplicar cambios

# Verificar conexión
lsusb | grep -i candle
```

### Problema: GUI no se conecta al motor
```bash
# Verificar que candle_ros2_node esté corriendo
ros2 node list
# Debe aparecer: /candle_ros2_node

# Verificar topics activos
ros2 topic list
ros2 topic hz /md80/joint_states
```

### Problema: Motor se mueve erráticamente
1. **Detener todo**: Ctrl+C en todas las terminales
2. **Desconectar motor físicamente** si es necesario
3. Verificar parámetros kp, kd, offset antes de re-habilitar
4. **Nunca usar `ros2 bag play`** con motor conectado sin supervisión

### Problema: Grabación no funciona
```bash
# Verificar que el directorio existe
mkdir -p /home/carlos/Escritorio/assistive_exo/src/rosbags/rosbags

# Verificar permisos de escritura
ls -ld /home/carlos/Escritorio/assistive_exo/src/rosbags/rosbags
```

### Problema: Rosbag muy grande
```bash
# Ver tamaño del rosbag
du -sh experimento_001/

# Grabar solo topics necesarios (en lugar de -a)
ros2 bag record /md80/joint_states /md80/motion_command -o experimento_limpio

# Convertir a formato comprimido (si es muy grande)
# TODO: investigar compresión MCAP
```

---

## 📚 Referencias

### Topics Principales

```bash
# Comandos de movimiento (publicar)
ros2 topic pub /md80/motion_command candle_ros2/msg/MotionCommand

# Estados de los motores (suscribirse)
ros2 topic echo /md80/joint_states

# Parámetros de impedancia (publicar)
ros2 topic pub /md80/impedance_command candle_ros2/msg/ImpedanceCommand
```

### Servicios Disponibles

```bash
# Listar servicios
ros2 service list | grep md80

# Añadir motores
ros2 service call /md80/add_md80s ...

# Habilitar/deshabilitar
ros2 service call /md80/enable_md80s ...

# Zero motors
ros2 service call /md80/zero_md80s ...

# Cambiar modo de control
ros2 service call /md80/set_control_mode ...
```

---

## 🔐 Seguridad

### ⚠️ Advertencias Importantes

1. **Siempre hacer ZERO** después de habilitar motores antes de enviar comandos
2. **Empezar con parámetros bajos** (kp, velocidad, amplitud) y aumentar gradualmente
3. **Tener acceso al botón de emergencia** o Ctrl+C listo
4. **NO reproducir rosbags** con motor conectado sin supervisión
5. **Verificar límites mecánicos** antes de trayectorias grandes

### ✅ Checklist Pre-Operación

- [ ] Candle conectado correctamente (lsusb)
- [ ] Nodo `candle_ros2_node` corriendo
- [ ] GUI abierta sin errores
- [ ] Motores añadidos y habilitados
- [ ] Zero ejecutado correctamente
- [ ] Parámetros de control configurados
- [ ] Límites mecánicos verificados
- [ ] Emergencia preparada (Ctrl+C o físico)

---

## 📝 Registro de Cambios

- **2026-02-16**: Creación del README con guías de lanzamiento y grabación

---

## 👥 Contacto

Para dudas o problemas, consultar documentación de:
- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- Candle/MD80: Documentación del fabricante
