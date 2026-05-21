# Inferencia

Este directorio contiene el workspace ROS 2 usado para ejecutar la inferencia y la comunicación con el simulador o el motor real.

## Estructura principal

- [src/rl_package/rl_package/](src/rl_package/rl_package/) - paquete ROS2 con los nodos de inferencia, como `rl_node.py`, `rl_node_v2.py`, `myosuite_publisher.py` y `aggregator.py`.
- [src/rl_interfaces/msg/MotionCommand.msg](src/rl_interfaces/msg/MotionCommand.msg) - definición del mensaje usado por el paquete.
- [models/](models/) - modelos y checkpoints usados por los nodos de inferencia.
- [myosuite_local/myosuite/envs/nair/](myosuite_local/myosuite/envs/nair/) - copia vendorizada del entorno NAIR para evitar depender de `site-packages`.
- [test_model_loading.py](test_model_loading.py) - script de comprobación para validar la carga de modelos localmente.

## Requisitos

- ROS 2 instalado y configurado en el sistema.
- Un entorno Python con las dependencias necesarias: `rclpy`, `torch`, `torchrl`, `tensordict`, `gym`/`mujoco`, etc.
- Si usas la copia vendorizada de MyoSuite, asegúrate de exportar `PYTHONPATH` antes de ejecutar los nodos.

## Preparación del entorno

Desde la raíz de [inferencia](.):

```bash
export PYTHONPATH="$PWD/myosuite_local:$PYTHONPATH"
```

Si todavía no has compilado el workspace ROS2:

```bash
colcon build
source install/setup.bash
```

## Orden recomendado en terminal

Si vas a lanzar la inferencia completa, usa este orden:

1. Iniciar el nodo  `aggregator`.

```bash
cd ~/Escritorio/ros2_rl_ws
source install/setup.bash
ros2 run rl_package aggregator
```

2. Levantar simulador o simulador + motor, según el caso.

Sólo simulador
```bash
cd ~/Escritorio/ros2_rl_ws
source install/setup.bash
ros2 run rl_package exo_publisher
```
Simulador+motor real (conectar motor con ordenador, puede variar la ruta o la forma de hacerlo)
```bash
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```
```bash
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 service call /candle_ros2_node/add_md80s candle_ros2/srv/AddMd80s "{drive_ids: [11]}"
ros2 service call /candle_ros2_node/set_mode_md80s candle_ros2/srv/SetModeMd80s "{drive_ids: [11], mode:["RAW_TORQUE"]}"
ros2 service call /candle_ros2_node/enable_md80s candle_ros2/srv/GenericMd80Msg "{drive_ids:[11]}"
```

3. Lanzar el nodo principal de inferencia.

```bash
cd ~/Escritorio/ros2_rl_ws
source install/setup.bash
ros2 run rl_package rl_node_vX
```

4. Lanzar el nodo `myosuite_publisher`

```bash
cd ~/Escritorio/ros2_rl_ws
source install/setup.bash
ros2 run rl_package myosuite_publisher
```

## Verificación opcional

Para comprobar que un modelo local carga correctamente:

```bash
python3 test_model_loading.py
```

## Ejecución

### Nodo principal de inferencia

```bash
ros2 run rl_package rl_node_v2
```

### Publicador de MyoSuite

```bash
ros2 run rl_package myosuite_publisher
```

### Nodo agregador

```bash
ros2 run rl_package aggregator
```

## Ejecución con simulador o motor

### Solo simulador

```bash
cd ~/Escritorio/ros2_rl_ws
source install/setup.bash
ros2 run rl_package exo_publisher
```

### Simulador + motor

Primero inicia el nodo del motor:

```bash
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node USB 1M
```

Después registra y habilita el motor con las llamadas de servicio correspondientes. Ejemplo para el drive 308:

```bash
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 service call /candle_ros2_node/add_md80s candle_ros2/srv/AddMd80s "{drive_ids: [308]}"
ros2 service call /candle_ros2_node/set_mode_md80s candle_ros2/srv/SetModeMd80s "{drive_ids: [308], mode:[\"RAW_TORQUE\"]}"
ros2 service call /candle_ros2_node/enable_md80s candle_ros2/srv/GenericMd80Msg "{drive_ids:[308]}"
```

Si usas el drive 11, repite las llamadas cambiando el `drive_id`:

```bash
cd ~/Escritorio/ros2_candle_ws/
source install/setup.bash
ros2 service call /candle_ros2_node/add_md80s candle_ros2/srv/AddMd80s "{drive_ids: [11]}"
ros2 service call /candle_ros2_node/set_mode_md80s candle_ros2/srv/SetModeMd80s "{drive_ids: [11], mode:[\"RAW_TORQUE\"]}"
ros2 service call /candle_ros2_node/enable_md80s candle_ros2/srv/GenericMd80Msg "{drive_ids:[11]}"
```

## Notas

- Coloca los checkpoints en [models/](models/). Los nodos buscan modelos relativos al `WORKSPACE_ROOT`.
- Si `colcon build` falla por dependencias Python, usa un `venv` o Conda con las librerías requeridas e instala una versión de `rclpy` compatible con tu ROS2.
- Si `colcon build --symlink-install` da el error `option --editable not recognized`, usa `colcon build` sin `--symlink-install`.
- Para pruebas rápidas fuera de ROS2, puedes ejecutar directamente los scripts Python tras exportar `PYTHONPATH`.