# IDs válidos registrados en Gymnasium:
#   'MIMoBench-v0'                             - Muñeco de pie, se cae
#   'MIMoBenchV2-v0'                           - Muñeco de pie, se cae,quitar los 2 objetos
#   'MIMoShowroom-v0'                          - Muñeco en una habitacion
#   'MIMoReach-v0'                             - Muñeco de pie, se mantiene de pie, mueve la cabeza
#   'MIMoStandup-v0'                           - Muñeco sujeto a una valla
#   'MIMoSelfBody-v0'                          - No funciona
#   'MIMoCatch-v0'                             - No funciona
#   'MIMoMuscle-v0'                            - Parecido a MIMoBench-v0
#   'MIMoMuscleStaticTest-v0'                  - Muñeco saltando, no se mueve, se queda volando
#   'MIMoVelocityMuscleTest-v0'                - Muñeco de pie, no se mueve, brazos extendidos, usar este
#   'MIMoMuscleStaticTestV2-v0'                - No funciona
#   'MIMoVelocityMuscleTestV2-v0'              - Muñeco de pie, no se mueve, brazos en cruz
#   'MIMoComplianceTest-v0'                    - No funciona
#   'MIMoComplianceMuscleTest-v0'              - No funciona
#   'MIMoRollOver-v0'                          - No usar, muñeco aparece de repente en el suelo

# Para que funcione gym.make con los entornos personalizados de MIMo, es necesario importar mimoEnv.
import mimoEnv  # Esto registra todos los entornos personalizados

import gymnasium as gym
import numpy as np
import csv
import os
import matplotlib.pyplot as plt
import time
from dataclasses import dataclass
from mimoActuation.muscle import MuscleModel

# === CONFIGURACIÓN PERSONALIZADA ===
ENV_ID = 'MIMoVelocityLowerBody-v0'  # Entorno solo miembros inferiores (puede leerse desde run_config.txt)
CSV_PATH = 'results/hip_motor_real/2026-05-07_16-04-57_id=11/telemetry.csv'  # Ruta al CSV de activaciones
DESFASE_CADERA = 0.5  # Desfase entre caderas (fracción del ciclo, 0.5 = 50% = media oscilación)
TRAJECTORY_PHASE_OFFSET_PCT = None  # Se intenta leer desde run_config.txt; si no, None
CAM_DISTANCIA = 2.5    # Distancia de la cámara al muñeco
# Ajustes de calibración: invertir signo si la trayectoria va al lado contrario
HIP_SIGN_LEFT = 1    # set -1 para invertir la señal de la cadera izquierda
HIP_SIGN_RIGHT = 1   # set -1 para invertir la señal de la cadera derecha
MASS_SCALE = 1.0  # Factor global para escalar masas (1.0 = sin cambio)
FMAX_SCALE = 1.0  # Factor global para escalar fmax/torque de actuadores (1.0 = sin cambio)


@dataclass
class MuscleUnitPattern:
    fmax: float
    vmax: float
    amplitude: float
    frequency_hz: float
    phase_rad: float
    bias: float

# === LOGGING Y PLOTEO ===
from datetime import datetime
now = datetime.now()
run_id = now.strftime('%Y-%m-%d_%H-%M-%S') + '_simulacion'
RESULTS_DIR = os.path.join('results', 'simulation_old', run_id)
os.makedirs(RESULTS_DIR, exist_ok=True)
CSV_LOG_PATH = os.path.join(RESULTS_DIR, 'hip_angles_log.csv')
IMG_PATH = os.path.join(RESULTS_DIR, 'hip_and_activations.png')
FPS = 60  # Asume 60 Hz de simulación
DURACION_GRAFICA = 15  # segundos a mostrar en la imagen
hip_left_log = []
hip_right_log = []
activ_log = []
activ_right_log = []

# === LECTURA DE ACTIVACIONES DESDE TELEMETRY.CSV ===
def cargar_activaciones_csv(path):
    """Lee u_flex_0-2 y u_ext_0-2 del CSV de telemetría (neural drive, no activaciones filtrantes)."""
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        columnas = ['u_flex_0', 'u_flex_1', 'u_flex_2', 'u_ext_0', 'u_ext_1', 'u_ext_2']
        data = []
        for row in reader:
            data.append([float(row[col]) for col in columnas])
    return np.array(data)

activaciones = cargar_activaciones_csv(CSV_PATH)
num_steps, num_act = activaciones.shape
assert num_act == 6, f'El CSV debe tener 6 columnas (u_flex_0-2, u_ext_0-2), tiene {num_act}'
print(f"CSV cargado: {num_steps} muestras de activaciones musculares")

def _parse_muscle_unit_spec(spec):
    """Parse the runner format 'fmax,vmax,amp,freq,phase,bias;...' into MuscleUnitPattern objects."""
    if not spec:
        return []
    parts = [p.strip() for p in str(spec).split(';') if p.strip()]
    units = []
    for part in parts:
        fields = [x.strip() for x in part.split(',')]
        if len(fields) != 6:
            continue
        try:
            units.append(MuscleUnitPattern(
                fmax=float(fields[0]),
                vmax=float(fields[1]),
                amplitude=float(fields[2]),
                frequency_hz=float(fields[3]),
                phase_rad=float(fields[4]),
                bias=float(fields[5]),
            ))
        except Exception:
            continue
    return units


def _calc_desfase_samples():
    """Calcula el desfase como 0.5 * tiempo_pico_a_pico.
    
    Encuentra picos de flex_0 con amplitud > 0.25 y calcula la distancia temporal
    entre picos consecutivos. El período es esa distancia. El desfase es 0.5 * período.
    """
    from scipy import signal
    
    # Extraer flex_0 (columna 0)
    flex_0 = activaciones[:, 0]
    
    # Encontrar picos con umbral > 0.25
    peaks, _ = signal.find_peaks(flex_0, height=0.25, distance=50)  # distance=50 para evitar falsos picos muy cercanos
    
    if len(peaks) < 2:
        print(f"  Advertencia: solo {len(peaks)} picos encontrados. Usando desfase 50%.")
        return int(round(0.5 * num_steps))
    
    # Calcular distancias entre picos consecutivos
    peak_distances = np.diff(peaks)
    
    # Usar la distancia promedio como período
    period_samples = np.mean(peak_distances)
    
    # Desfase = 0.5 * período
    desfase_samples = int(round(0.5 * period_samples))
    
    print(f"  flex_0 picos encontrados: {len(peaks)}")
    print(f"  Período promedio (pico a pico): {period_samples:.1f} muestras")
    print(f"  Desfase (0.5 * período): {desfase_samples} muestras")
    
    return desfase_samples

def _read_runconfig_muscle_blocks(csv_path):
    """Read run_config.txt and return explicit actuator muscle blocks plus legacy fallback values.
    
    Also attempts to extract trajectory_phase_offset_pct if present.
    """
    run_dir = os.path.dirname(os.path.abspath(csv_path))
    cfg_path = os.path.join(run_dir, "run_config.txt")
    print("")
    if not os.path.isfile(cfg_path):
        print(f"Not found run_config.txt at {cfg_path}, proceeding without muscle values.")
        return {}, None, None, None, None, None

    print(f"Found run_config.txt at {cfg_path}, attempting to read explicit actuator muscle specs.")

    blocks = {}
    current_name = None
    current_block = None
    legacy_flex = None
    legacy_ext = None
    trajectory_phase_offset = None
    
    with open(cfg_path, 'r', encoding='utf-8') as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('trajectory_phase_offset_pct:'):
                try:
                    trajectory_phase_offset = float(line.split(':', 1)[1].strip()) / 100.0
                    print(f"Read trajectory_phase_offset_pct: {trajectory_phase_offset * 100}%")
                except Exception:
                    pass
                continue
            if line.startswith('actuator:'):
                current_name = line.split(':', 1)[1].strip()
                current_block = blocks.setdefault(current_name, {})
                continue
            if line.startswith('flex_muscles:'):
                legacy_flex = _parse_muscle_unit_spec(line.split(':', 1)[1].strip())
                continue
            if line.startswith('ext_muscles:'):
                legacy_ext = _parse_muscle_unit_spec(line.split(':', 1)[1].strip())
                continue
            if current_block is None:
                continue
            key, _, value = line.partition(':')
            if not _:
                continue
            key = key.strip().lower()
            value = value.strip()
            if key in {'fmax_neg', 'vmax_neg', 'fmax_pos', 'vmax_pos'}:
                try:
                    current_block[key] = float(value)
                except Exception:
                    current_block[key] = None
            elif key in {'source_units_neg', 'source_units_pos'}:
                current_block[key] = value

    print(f"Parsed explicit actuator blocks: {list(blocks.keys())}")
    return blocks, legacy_flex, legacy_ext, cfg_path, run_dir, trajectory_phase_offset


# LEER run_config.txt PRIMERO para obtener trajectory_phase_offset_pct
runconfig_blocks, legacy_flex_units, legacy_ext_units, cfg_path, run_dir, trajectory_phase_offset_from_cfg = _read_runconfig_muscle_blocks(CSV_PATH)

# Usa el offset leído desde run_config.txt si está disponible
if trajectory_phase_offset_from_cfg is not None:
    TRAJECTORY_PHASE_OFFSET_PCT = trajectory_phase_offset_from_cfg
    print(f"Using trajectory_phase_offset_pct from run_config.txt: {TRAJECTORY_PHASE_OFFSET_PCT * 100}%")
elif TRAJECTORY_PHASE_OFFSET_PCT is not None:
    print(f"Using manually configured TRAJECTORY_PHASE_OFFSET_PCT: {TRAJECTORY_PHASE_OFFSET_PCT * 100}%")
else:
    TRAJECTORY_PHASE_OFFSET_PCT = 0.0
    print(f"No trajectory_phase_offset_pct found, using 0%")

# Calcular offsets en muestras (no aplicarlas a la data, las aplicaremos en el loop)
# NOTA: offset_samples = punto donde comienza el ciclo en el CSV
#       desfase_samples = desfase REAL calculado como 0.5 * período de flex_0
offset_samples = 0
desfase_samples = _calc_desfase_samples()

print(f"CSV has {num_steps} samples")
print(f"  Starting offset: {offset_samples} samples ({offset_samples/num_steps*100:.1f}%)")
print(f"  Left hip will read from CSV index: (step + {offset_samples}) % {num_steps}")
print(f"  Right hip will read from CSV index: (step + {offset_samples} + {desfase_samples}) % {num_steps}")

# === FUNCIONES DE CÁMARA ===
def set_camera_to_com(env, distancia=2.5):
    # Calcula el centro de masa del muñeco (solo X, Y)
    com = env.data.subtree_com[1]  # 1 suele ser el cuerpo principal
    viewer = getattr(env, 'viewer', None)
    if viewer and hasattr(viewer, 'cam'):
        viewer.cam.lookat[0] = com[0]
        viewer.cam.lookat[1] = com[1]
        # Mantén la altura fija (por ejemplo, z=1.0)
        viewer.cam.lookat[2] = 1.0
        viewer.cam.distance = distancia


def scale_masses(env, factor: float):
    """Escala masas del modelo en tiempo de ejecución.

    Intenta ajustar `body_mass` y `geom_mass` cuando estén disponibles.
    """
    if abs(factor - 1.0) < 1e-12:
        return
    print("")
    print(f"Aplicando factor de masa {factor} al modelo (runtime)")
    # Escala masas de bodies si existe el array
    try:
        env.model.body_mass[:] = env.model.body_mass[:] * float(factor)
        print(f"Masas de bodies escaladas por {factor}")
    except Exception:
        pass
    # Escala masas de geoms si existe el array
    try:
        env.model.geom_mass[:] = env.model.geom_mass[:] * float(factor)
        print(f"Masas de geoms escaladas por {factor}")
    except Exception:
        pass
    # Nota: inertias derivadas pueden necesitar recalibración más precisa

env = gym.make(ENV_ID, render_mode='human')
env.unwrapped.model.opt.gravity[:] = np.array([0.0, 0.0, -9.81])
print('body_mass exposed?', hasattr(env.model, 'body_mass'))
print('geom_mass exposed?', hasattr(env.model, 'geom_mass'))

# listar masas por geom (si la API permite acceder a .mass)
for i in range(env.model.ngeom):
    try:
        g = env.model.geom(i)
        print(i, g.name, getattr(g, 'mass', 'no mass attr'))
    except Exception:
        print('geom', i, 'no accesible')

# Aplica escalado de masas en tiempo de ejecución si se ha configurado
try:
    scale_masses_loc = scale_masses
except NameError:
    scale_masses_loc = None
if scale_masses_loc is not None and MASS_SCALE != 1.0:
    scale_masses_loc(env, MASS_SCALE)
obs, info = env.reset()
# print(env.action_space.shape)
# print(getattr(env.actuation_model, 'muscle_names', 'No muscle_names'))
# Construyamos mapeo de actuadores (orden tal como los tiene el modelo)
try:
    mimo_actuator_ids = env.mimo_actuators
except Exception:
    mimo_actuator_ids = getattr(env, 'mimo_actuators', None)
if mimo_actuator_ids is None:
    mimo_actuator_ids = np.arange(env.model.nu)
actuator_names = [env.model.actuator(i).name for i in mimo_actuator_ids]
n_mim_act = len(actuator_names)
"""
print(f"Número de actuadores MIMo: {n_mim_act}")
for i, name in enumerate(actuator_names):
    print(f"{i}: {name}")
"""
# Nuestro CSV tiene 6 canales por cadera (3 flexores + 3 extensores)
# que representan UNA sola DoF de cadera. Combinaremos los 3 flexores
# y los 3 extensores en una única señal por dirección (media simple).
hip_right_dof = 'act:right_hip_flex'
hip_left_dof = 'act:left_hip_flex'

def find_index(name, actuator_names):
    try:
        return actuator_names.index(name)
    except ValueError:
        return None

idx_right = find_index(hip_right_dof, actuator_names)
idx_left = find_index(hip_left_dof, actuator_names)


def _units_to_arrays(units):
    if not units:
        return None, None
    fmax_vals = [float(u.fmax) for u in units]
    vmax_vals = [float(u.vmax) for u in units]
    return fmax_vals, vmax_vals


def _scale_list(values, factor):
    if values is None:
        return None
    return [float(v) * float(factor) for v in values]


if runconfig_blocks:
    print(f"Usando bloques explícitos de run_config.txt: {list(runconfig_blocks.keys())}")
else:
    legacy_flex_fmax, legacy_flex_vmax = _units_to_arrays(legacy_flex_units)
    legacy_ext_fmax, legacy_ext_vmax = _units_to_arrays(legacy_ext_units)
    print(f"Usando fmax flexores desde run_config.txt: {legacy_flex_fmax}")
    print(f"Usando fmax extensores desde run_config.txt: {legacy_ext_fmax}")
    print(f"Usando vmax flexores desde run_config.txt: {legacy_flex_vmax}")
    print(f"Usando vmax extensores desde run_config.txt: {legacy_ext_vmax}")

def _block_to_weights(block):
    source_neg = _parse_muscle_unit_spec(block.get('source_units_neg', ''))
    source_pos = _parse_muscle_unit_spec(block.get('source_units_pos', ''))
    neg_fmax, neg_vmax = _units_to_arrays(source_neg)
    pos_fmax, pos_vmax = _units_to_arrays(source_pos)
    return neg_fmax, neg_vmax, pos_fmax, pos_vmax

runconfig_neg_fmax = None
runconfig_neg_vmax = None
runconfig_pos_fmax = None
runconfig_pos_vmax = None
if runconfig_blocks:
    first_block = next(iter(runconfig_blocks.values()))
    runconfig_neg_fmax, runconfig_neg_vmax, runconfig_pos_fmax, runconfig_pos_vmax = _block_to_weights(first_block)

print("")

print("")

step = 0
PRINT_EVERY = 100  # Frecuencia de impresión de debug info

try:
    # If this env uses MuscleModel and we parsed muscle specs, apply them to the actuation model
    try:
        if isinstance(env.actuation_model, MuscleModel):
            import numpy as _np
            cur_fmax = getattr(env.actuation_model, 'fmax', None)
            cur_vmax = getattr(env.actuation_model, 'vmax', None)

            def _find_actuator_index(name):
                try:
                    return actuator_names.index(name)
                except ValueError:
                    return None

            def _apply_block_for_actuator(act_name, block):
                idx = _find_actuator_index(act_name)
                if idx is None or cur_fmax is None or cur_vmax is None:
                    return False
                neg_fmax = block.get('fmax_neg')
                neg_vmax = block.get('vmax_neg')
                pos_fmax = block.get('fmax_pos')
                pos_vmax = block.get('vmax_pos')
                if neg_fmax is None or neg_vmax is None or pos_fmax is None or pos_vmax is None:
                    return False
                neg_fmax = float(neg_fmax) * float(FMAX_SCALE)
                pos_fmax = float(pos_fmax) * float(FMAX_SCALE)
                neg_vmax = float(neg_vmax)
                pos_vmax = float(pos_vmax)

                new_fmax = _np.asarray(cur_fmax).copy()
                new_vmax = _np.asarray(cur_vmax).copy()
                n_act = len(actuator_names)
                if idx < new_fmax.shape[0]:
                    new_fmax[idx] = neg_fmax
                    new_vmax[idx] = neg_vmax
                if idx + n_act < new_fmax.shape[0]:
                    new_fmax[idx + n_act] = pos_fmax
                    new_vmax[idx + n_act] = pos_vmax
                env.actuation_model.set_fmax(new_fmax)
                env.actuation_model.set_vmax(new_vmax)
                print(f"Applied per-actuator specs to {act_name} (idx={idx}).")
                return True

            applied_any = False
            if runconfig_blocks:
                for act_name, block in runconfig_blocks.items():
                    if _apply_block_for_actuator(act_name, block):
                        applied_any = True
            else:
                # Backward-compatible fallback for old run_config.txt files.
                legacy_flex_fmax, legacy_flex_vmax = _units_to_arrays(legacy_flex_units)
                legacy_ext_fmax, legacy_ext_vmax = _units_to_arrays(legacy_ext_units)
                if legacy_flex_fmax is not None and legacy_ext_fmax is not None and cur_fmax is not None and cur_vmax is not None:
                    hip_names = [name for name in ('act:right_hip_flex', 'act:left_hip_flex') if name in actuator_names]
                    for act_name in hip_names:
                        idx = _find_actuator_index(act_name)
                        if idx is None:
                            continue
                        new_fmax = _np.asarray(cur_fmax).copy()
                        new_vmax = _np.asarray(cur_vmax).copy()
                        n_act = len(actuator_names)
                        new_fmax[idx] = float(np.sum(np.asarray(legacy_flex_fmax, dtype=float))) * float(FMAX_SCALE)
                        new_vmax[idx] = float(np.mean(np.asarray(legacy_flex_vmax, dtype=float)))
                        new_fmax[idx + n_act] = float(np.sum(np.asarray(legacy_ext_fmax, dtype=float))) * float(FMAX_SCALE)
                        new_vmax[idx + n_act] = float(np.mean(np.asarray(legacy_ext_vmax, dtype=float)))
                        env.actuation_model.set_fmax(new_fmax)
                        env.actuation_model.set_vmax(new_vmax)
                        applied_any = True
                        print(f"Applied legacy flex/ext specs to {act_name} (idx={idx}).")

            if not applied_any:
                print("No matching actuator blocks were applied from run_config.txt.")
    except Exception:
        pass

    max_steps = 2 * num_steps  # Limitar a 2 ciclos completos del CSV para terminar rápido
    step_count = 0
    
    while step_count < max_steps:
        # Construye vector completo de acción para el modelo de actuación.
        # Puede ser que el modelo espere 2*n (muscles neg/pos) o ya espere n entradas (net torque/drive).
        action_space_len = env.action_space.shape[0]
        full_action = np.zeros(action_space_len, dtype=np.float32)
        n_mimo_act = len(mimo_actuator_ids)
        # Determina si el action_space está dividido en dos mitades (flex/ext) o es una única señal por actuador
        if action_space_len == 2 * n_mimo_act:
            two_channel = True
            half = action_space_len // 2
        else:
            two_channel = False

        # Reproduce el CSV en bucle (wrap-around) para ejecución indefinida
        # Aplica primero el offset de inicio, luego el desfase entre caderas
        idx_base = (step + offset_samples) % num_steps
        csv_idx_left = idx_base
        csv_idx_right = (idx_base + desfase_samples) % num_steps
        
        # DEBUG: mostrar primeros 5 steps
        if step < 5:
            print(f"step={step}: csv_idx_left={csv_idx_left}, csv_idx_right={csv_idx_right}, desfase_samples={desfase_samples}")
        
        # Cadera izquierda: datos directos del CSV en índice actual
        flex_izq = activaciones[csv_idx_left, :3]
        ext_izq = activaciones[csv_idx_left, 3:]
        
        # Cadera derecha: MISMO CSV pero con índice desplazado 0.5 ciclo (1500 muestras)
        # Sin invertir flexores/extensores - se usan directamente desfasados temporalmente
        flex_der = activaciones[csv_idx_right, :3]
        ext_der = activaciones[csv_idx_right, 3:]

        # Logging de ángulos y activaciones
        if idx_left is not None:
            qadr_left = int(env.model.jnt_qposadr[env.model.actuator_trnid[mimo_actuator_ids[idx_left], 0]])
            hip_left_log.append(float(env.data.qpos[qadr_left]))
        else:
            hip_left_log.append(float('nan'))
        if idx_right is not None:
            qadr_right = int(env.model.jnt_qposadr[env.model.actuator_trnid[mimo_actuator_ids[idx_right], 0]])
            hip_right_log.append(float(env.data.qpos[qadr_right]))
        else:
            hip_right_log.append(float('nan'))
        activ_log.append(list(activaciones[csv_idx_left]))
        activ_right_log.append(list(activaciones[csv_idx_right]))

        # Combina las 3 unidades flex/ext en una sola por cadera usando fmax como pesos
        def _combine(u_flex, u_ext, fmax_flex, fmax_ext):
            # u_flex/ext: arrays length M
            if fmax_flex and len(fmax_flex) == len(u_flex):
                wf = np.asarray(fmax_flex, dtype=np.float32)
                flex_comb = float(np.sum(u_flex * wf) / (np.sum(wf) + 1e-12))
            else:
                flex_comb = float(np.mean(u_flex))
            if fmax_ext and len(fmax_ext) == len(u_ext):
                we = np.asarray(fmax_ext, dtype=np.float32)
                ext_comb = float(np.sum(u_ext * we) / (np.sum(we) + 1e-12))
            else:
                ext_comb = float(np.mean(u_ext))
            return flex_comb, ext_comb

        if runconfig_blocks and runconfig_neg_fmax is not None and runconfig_pos_fmax is not None:
            flex_left_comb, ext_left_comb = _combine(flex_izq, ext_izq, runconfig_neg_fmax, runconfig_pos_fmax)
            flex_right_comb, ext_right_comb = _combine(flex_der, ext_der, runconfig_neg_fmax, runconfig_pos_fmax)
        else:
            flex_left_comb, ext_left_comb = _combine(flex_izq, ext_izq, legacy_flex_fmax, legacy_ext_fmax)
            flex_right_comb, ext_right_comb = _combine(flex_der, ext_der, legacy_flex_fmax, legacy_ext_fmax)

        def _compute_tau_from_units(u_flex, u_ext, fmax_flex, fmax_ext):
            # Devuelve torque objetivo (positiva = extensión según CSV convención asumida)
            if fmax_flex is not None and fmax_ext is not None and len(fmax_flex) == len(u_flex) and len(fmax_ext) == len(u_ext):
                tau_flex = float(np.sum(np.asarray(u_flex) * np.asarray(fmax_flex)))
                tau_ext = float(np.sum(np.asarray(u_ext) * np.asarray(fmax_ext)))
                return tau_flex - tau_ext
            else:
                # Fallback: usar la diferencia normalizada (media) como proxy
                return float(np.mean(u_flex) - np.mean(u_ext))

        # Calcula torque objetivo para cada cadera
        if runconfig_blocks and runconfig_neg_fmax is not None and runconfig_pos_fmax is not None:
            tau_left = _compute_tau_from_units(flex_izq, ext_izq, runconfig_neg_fmax, runconfig_pos_fmax)
            tau_right = _compute_tau_from_units(flex_der, ext_der, runconfig_neg_fmax, runconfig_pos_fmax)
        else:
            tau_left = _compute_tau_from_units(flex_izq, ext_izq, legacy_flex_fmax, legacy_ext_fmax)
            tau_right = _compute_tau_from_units(flex_der, ext_der, legacy_flex_fmax, legacy_ext_fmax)

        # Si el entorno usa MuscleModel, debemos enviar activaciones por músculo (2*n):
        is_muscle_model = isinstance(env.actuation_model, MuscleModel)

        if is_muscle_model:
            # action_space expected: 2 * n_mimo_act (negatives first, positives second)
            if action_space_len == 2 * n_mimo_act:
                if idx_left is not None:
                    full_action[idx_left] = float(HIP_SIGN_LEFT * flex_left_comb)
                    full_action[idx_left + n_mimo_act] = float(HIP_SIGN_LEFT * ext_left_comb)
                if idx_right is not None:
                    full_action[idx_right] = float(HIP_SIGN_RIGHT * flex_right_comb)
                    full_action[idx_right + n_mimo_act] = float(HIP_SIGN_RIGHT * ext_right_comb)
            else:
                # fallback: place net activation in single-channel if unexpectedly shaped
                if idx_left is not None and idx_left < action_space_len:
                    full_action[idx_left] = float(HIP_SIGN_LEFT * (flex_left_comb - ext_left_comb))
                if idx_right is not None and idx_right < action_space_len:
                    full_action[idx_right] = float(HIP_SIGN_RIGHT * (flex_right_comb - ext_right_comb))
        else:
            # Non-muscle actuation (e.g., SpringDamperModel): map torque -> ctrl (tau/gear)
            if two_channel:
                # Cada actuador tiene dos canales (neg/pos) en las dos mitades del vector
                if idx_left is not None and idx_left < half:
                    sim_idx = int(mimo_actuator_ids[idx_left])
                    gear_neg = float(env.model.actuator_gear[sim_idx, 0])
                    try:
                        sim_idx_pos = int(mimo_actuator_ids[idx_left + half])
                    except Exception:
                        sim_idx_pos = None
                    if tau_left >= 0:
                        if sim_idx_pos is not None:
                            gear_pos = float(env.model.actuator_gear[sim_idx_pos, 0])
                            ctrl = np.clip(tau_left / (gear_pos + 1e-12), env.action_space.low[idx_left + half], env.action_space.high[idx_left + half])
                            full_action[idx_left + half] = float(ctrl)
                    else:
                        ctrl = np.clip((-tau_left) / (gear_neg + 1e-12), env.action_space.low[idx_left], env.action_space.high[idx_left])
                        full_action[idx_left] = float(ctrl)
                else:
                    if idx_left is not None:
                        print(f"Warning: left actuator index {idx_left} out of half-range (half={half})")

                if idx_right is not None and idx_right < half:
                    sim_idx = int(mimo_actuator_ids[idx_right])
                    gear_neg = float(env.model.actuator_gear[sim_idx, 0])
                    try:
                        sim_idx_pos = int(mimo_actuator_ids[idx_right + half])
                    except Exception:
                        sim_idx_pos = None
                    if tau_right >= 0:
                        if sim_idx_pos is not None:
                            gear_pos = float(env.model.actuator_gear[sim_idx_pos, 0])
                            ctrl = np.clip(tau_right / (gear_pos + 1e-12), env.action_space.low[idx_right + half], env.action_space.high[idx_right + half])
                            full_action[idx_right + half] = float(ctrl)
                    else:
                        ctrl = np.clip((-tau_right) / (gear_neg + 1e-12), env.action_space.low[idx_right], env.action_space.high[idx_right])
                        full_action[idx_right] = float(ctrl)
                else:
                    if idx_right is not None:
                        print(f"Warning: right actuator index {idx_right} out of half-range (half={half})")
            else:
                # Single-channel actuation: mapa directo de torque objetivo a ctrl usando actuator_gear
                if idx_left is not None and idx_left < action_space_len:
                    sim_idx = int(mimo_actuator_ids[idx_left])
                    gear = float(env.model.actuator_gear[sim_idx, 0])
                    if isinstance(tau_left, float):
                        ctrl = np.clip(tau_left / (gear + 1e-12), env.action_space.low[idx_left], env.action_space.high[idx_left])
                        full_action[idx_left] = float(ctrl)
                    else:
                        full_action[idx_left] = float(flex_left_comb - ext_left_comb)
                if idx_right is not None and idx_right < action_space_len:
                    sim_idx = int(mimo_actuator_ids[idx_right])
                    gear = float(env.model.actuator_gear[sim_idx, 0])
                    if isinstance(tau_right, float):
                        ctrl = np.clip(tau_right / (gear + 1e-12), env.action_space.low[idx_right], env.action_space.high[idx_right])
                        full_action[idx_right] = float(ctrl)
                    else:
                        full_action[idx_right] = float(flex_right_comb - ext_right_comb)

        obs, reward, terminated, truncated, info = env.step(full_action)
        set_camera_to_com(env, CAM_DISTANCIA)
        env.render()

        # Debug prints: mostrar qué actuadores de cadera se usaron y qué valores se envían/aplican
        if step % PRINT_EVERY == 0 or step < 5:
            print(f"step={step}")
            if idx_left is not None:
                print(f" left actuator idx={idx_left} name={actuator_names[idx_left]} sent:(flex={flex_left_comb:.3f}, ext={ext_left_comb:.3f}) ctrl={float(env.data.ctrl[mimo_actuator_ids[idx_left]])} gear={float(env.model.actuator_gear[mimo_actuator_ids[idx_left],0])}")
                # also print counterpart (positive muscle)
                print(f" left positive ctrl={float(env.data.ctrl[mimo_actuator_ids[idx_left]+0 if False else mimo_actuator_ids[idx_left]])} (note: muscle model sets ctrl to ones), gear_pos={float(env.model.actuator_gear[mimo_actuator_ids[idx_left],0])}")
            else:
                print(" left actuator not found")
            if idx_right is not None:
                print(f" right actuator idx={idx_right} name={actuator_names[idx_right]} sent:(flex={flex_right_comb:.3f}, ext={ext_right_comb:.3f}) ctrl={float(env.data.ctrl[mimo_actuator_ids[idx_right]])} gear={float(env.model.actuator_gear[mimo_actuator_ids[idx_right],0])}")
                print(f" right positive ctrl={float(env.data.ctrl[mimo_actuator_ids[idx_right]+0 if False else mimo_actuator_ids[idx_right]])} gear_pos={float(env.model.actuator_gear[mimo_actuator_ids[idx_right],0])}")
            else:
                print(" right actuator not found")
            # Print qpos of hips, computed torque and contact count for diagnosis
            try:
                def _actuator_joint_qpos(act_idx):
                    # Map actuator index (index in mimo_actuator_ids list) -> joint qpos adr
                    sim_act_id = int(mimo_actuator_ids[act_idx])
                    joint_id = int(env.model.actuator_trnid[sim_act_id, 0])
                    qpos_adr = int(env.model.jnt_qposadr[joint_id])
                    return qpos_adr

                left_q = 'n/a'
                right_q = 'n/a'
                left_tau = 'n/a'
                right_tau = 'n/a'
                if idx_left is not None:
                    qadr = _actuator_joint_qpos(idx_left)
                    left_q = float(env.data.qpos[qadr])
                    sim_idx = int(mimo_actuator_ids[idx_left])
                    left_tau = float(env.model.actuator_gear[sim_idx, 0] * env.data.ctrl[sim_idx])
                if idx_right is not None:
                    qadr = _actuator_joint_qpos(idx_right)
                    right_q = float(env.data.qpos[qadr])
                    sim_idx = int(mimo_actuator_ids[idx_right])
                    right_tau = float(env.model.actuator_gear[sim_idx, 0] * env.data.ctrl[sim_idx])

                ncon = int(getattr(env.data, 'ncon', 0))
                # También añadir qpos de rodillas y orientación del torso (z de subtree_com)
                # obtener qpos de rodillas
                def _find_joint_qpos(joint_name):
                    try:
                        jid = env.model.joint(joint_name).id
                        adr = int(env.model.jnt_qposadr[jid])
                        return float(env.data.qpos[adr])
                    except Exception:
                        return 'n/a'

                #right_knee_q = _find_joint_qpos('robot:right_knee')
                #left_knee_q = _find_joint_qpos('robot:left_knee')
                # torso COM z
                torso_z = float(env.data.subtree_com[env.model.body('upper_body').id][2])

                #print(f" qpos_right={right_q} qpos_left={left_q} knee_r={right_knee_q} knee_l={left_knee_q} torque_r={right_tau}Nm torque_l={left_tau}Nm contacts={ncon} torso_z={torso_z}")
            except Exception:
                pass

        # Si el entorno terminó (episodio), resetea y sigue reproduciendo
        if terminated or truncated:
            try:
                obs, info = env.reset()
            except Exception:
                pass

        step += 1
        step_count += 1
except KeyboardInterrupt:
    print("Simulación interrumpida por usuario")
except Exception as e:
    print(f"Simulación finalizada por excepción: {e}")
finally:
    env.close()

    # Guardar CSV de ángulos de cadera
    with open(CSV_LOG_PATH, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['hip_left', 'hip_right'] + [f'activ_{i+1}' for i in range(6)])
        for l, r, a in zip(hip_left_log, hip_right_log, activ_log):
            writer.writerow([l, r] + a)

    # Graficar solo los últimos 20 segundos
    n_plot = min(len(hip_left_log), 20 * FPS)
    start_plot = max(0, len(hip_left_log) - n_plot)
    t = np.arange(n_plot) / FPS
    fig, axs = plt.subplots(3, 1, figsize=(14, 8), sharex=True)

    axs[0].plot(t, hip_left_log[start_plot:start_plot + n_plot], label='Cadera Izquierda', color='#1f5c9a', linewidth=1.8)
    axs[0].plot(t, hip_right_log[start_plot:start_plot + n_plot], label='Cadera Derecha', color='#c43c39', linewidth=1.8, alpha=0.95)
    axs[0].set_ylabel('Ángulo de cadera [rad]')
    axs[0].set_title('Trayectoria de ambas caderas (últimos 20s)')
    axs[0].grid(True, alpha=0.25)
    axs[0].legend(loc='best', frameon=False)
    axs[0].invert_yaxis()  # Invertir eje Y para que flexión hacia adelante sea hacia abajo

    left_activs = np.array(activ_log[start_plot:start_plot + n_plot])
    right_activs = np.array(activ_right_log[start_plot:start_plot + n_plot])
    for i in range(3):
        axs[1].plot(t, left_activs[:, i], label=f'flex_{i}', linewidth=1.1, alpha=0.9)
    for i in range(3):
        axs[1].plot(t, left_activs[:, i + 3], label=f'ext_{i}', linewidth=1.1, alpha=0.9, linestyle='--')
    axs[1].set_ylabel('Activación [-]')
    axs[1].set_title('Activaciones musculares cadera izquierda (últimos 20s)')
    axs[1].set_ylim(-0.05, 1.05)
    axs[1].grid(True, alpha=0.25)
    axs[1].legend(loc='upper right', ncol=2, fontsize=8, frameon=False)

    for i in range(3):
        axs[2].plot(t, right_activs[:, i], label=f'flex_{i}', linewidth=1.1, alpha=0.9)
    for i in range(3):
        axs[2].plot(t, right_activs[:, i + 3], label=f'ext_{i}', linewidth=1.1, alpha=0.9, linestyle='--')
    axs[2].set_ylabel('Activación [-]')
    axs[2].set_xlabel('Tiempo [s]')
    axs[2].set_title('Activaciones musculares cadera derecha (últimos 20s)')
    axs[2].set_ylim(-0.05, 1.05)
    axs[2].grid(True, alpha=0.25)
    axs[2].legend(loc='upper right', ncol=2, fontsize=8, frameon=False)

    plt.tight_layout()
    plt.savefig(IMG_PATH, dpi=220)
    # Guardar metadatos adicionales en un TXT
    META_PATH = os.path.join(RESULTS_DIR, 'simulation_info.txt')

    def _estimate_freq_and_angvel(angle_list, fps):
        arr = np.asarray(angle_list, dtype=float)
        mask = np.isfinite(arr)
        if mask.sum() < 8:
            return None, None
        s = arr[mask]
        N = len(s)
        # angular velocity (mean absolute derivative)
        angvel = float(np.nanmean(np.abs(np.diff(s)) * fps))
        # dominant frequency via FFT
        s = s - np.mean(s)
        yf = np.fft.rfft(s)
        xf = np.fft.rfftfreq(N, 1.0 / fps)
        freq_mask = (xf >= 0.2) & (xf <= 3.0)
        if not np.any(freq_mask):
            return None, angvel
        yf_sel = np.abs(yf[freq_mask])
        idx = int(np.argmax(yf_sel))
        xf_sel = xf[freq_mask]
        return float(xf_sel[idx]), angvel

    left_freq, left_angvel = _estimate_freq_and_angvel(hip_left_log, FPS)
    right_freq, right_angvel = _estimate_freq_and_angvel(hip_right_log, FPS)

    def _find_peaks(angle_list, fps, max_peaks=5):
        arr = np.asarray(angle_list, dtype=float)
        mask = np.isfinite(arr)
        if mask.sum() < 3:
            return {'n_max': 0, 'n_min': 0, 'max_peaks': [], 'min_peaks': []}
        x = arr[mask]
        # local maxima
        imax = np.where((x[1:-1] > x[:-2]) & (x[1:-1] > x[2:]))[0] + 1
        imin = np.where((x[1:-1] < x[:-2]) & (x[1:-1] < x[2:]))[0] + 1
        max_vals = x[imax] if imax.size else np.array([])
        min_vals = x[imin] if imin.size else np.array([])
        max_times = imax / float(fps) if imax.size else np.array([])
        min_times = imin / float(fps) if imin.size else np.array([])
        # select strongest peaks
        def select(pevals, ptimes):
            if len(pevals) == 0:
                return []
            order = np.argsort(-np.abs(pevals))
            sel = order[:max_peaks]
            return [(float(ptimes[i]), float(pevals[i])) for i in sel]

        return {
            'n_max': int(len(max_vals)),
            'n_min': int(len(min_vals)),
            'max_peaks': select(max_vals, max_times),
            'min_peaks': select(min_vals, min_times)
        }

    left_peaks = _find_peaks(hip_left_log, FPS, max_peaks=5)
    right_peaks = _find_peaks(hip_right_log, FPS, max_peaks=5)

    with open(META_PATH, 'w', encoding='utf-8') as mf:
        mf.write(f"run_id: {run_id}\n")
        mf.write(f"env_id: {ENV_ID}\n")
        mf.write(f"csv_source: {CSV_PATH}\n")
        mf.write(f"mass_scale: {MASS_SCALE}\n")
        mf.write(f"fps: {FPS}\n")
        mf.write(f"n_timesteps_recorded: {len(hip_left_log)}\n")
        duration_s = len(hip_left_log) / (FPS if FPS > 0 else 1)
        mf.write(f"duration_s: {duration_s:.3f}\n")
        mf.write('\n')
        mf.write('gait_interpretation:\n')
        mf.write('  one_hip_cycle: return of the same hip to the same phase/position\n')
        mf.write('  steps_per_stride: 2\n')
        mf.write('  note: if the right hip completes one cycle, the left leg has also taken one step, so that equals 2 steps total\n')
        mf.write('\n')
        mf.write('hip_left:\n')
        mf.write(f"  dominant_freq_hz: {left_freq if left_freq is not None else 'n/a'}\n")
        mf.write(f"  mean_abs_angvel_rad_s: {left_angvel if left_angvel is not None else 'n/a'}\n")
        # pasos por segundo estimados: (1) a partir de la freq dominante (asumiendo 1 ciclo = 1 stride = 2 pasos)
        try:
            steps_per_s_freq_left = float(left_freq) * 2 if left_freq is not None else None
        except Exception:
            steps_per_s_freq_left = None
        total_extrema_left = left_peaks['n_max'] + left_peaks['n_min']
        steps_per_s_peaks_left = float(total_extrema_left) / duration_s if duration_s > 0 else None
        mf.write(f"  steps_per_s_from_freq: {steps_per_s_freq_left if steps_per_s_freq_left is not None else 'n/a'}\n")
        mf.write(f"  steps_per_s_from_peaks: {steps_per_s_peaks_left if steps_per_s_peaks_left is not None else 'n/a'}\n")
        mf.write(f"  n_peaks_max: {left_peaks['n_max']}\n")
        mf.write(f"  n_peaks_min: {left_peaks['n_min']}\n")
        mf.write('  peaks_max (time_s, value):\n')
        for t, v in left_peaks['max_peaks']:
            mf.write(f"    - {t:.3f}s: {v:.6f}\n")
        mf.write('  peaks_min (time_s, value):\n')
        for t, v in left_peaks['min_peaks']:
            mf.write(f"    - {t:.3f}s: {v:.6f}\n")
        mf.write('hip_right:\n')
        mf.write(f"  dominant_freq_hz: {right_freq if right_freq is not None else 'n/a'}\n")
        mf.write(f"  mean_abs_angvel_rad_s: {right_angvel if right_angvel is not None else 'n/a'}\n")
        try:
            steps_per_s_freq_right = float(right_freq) * 2 if right_freq is not None else None
        except Exception:
            steps_per_s_freq_right = None
        total_extrema_right = right_peaks['n_max'] + right_peaks['n_min']
        steps_per_s_peaks_right = float(total_extrema_right) / duration_s if duration_s > 0 else None
        mf.write(f"  steps_per_s_from_freq: {steps_per_s_freq_right if steps_per_s_freq_right is not None else 'n/a'}\n")
        mf.write(f"  steps_per_s_from_peaks: {steps_per_s_peaks_right if steps_per_s_peaks_right is not None else 'n/a'}\n")
        mf.write(f"  n_peaks_max: {right_peaks['n_max']}\n")
        mf.write(f"  n_peaks_min: {right_peaks['n_min']}\n")
        mf.write('  peaks_max (time_s, value):\n')
        for t, v in right_peaks['max_peaks']:
            mf.write(f"    - {t:.3f}s: {v:.6f}\n")
        mf.write('  peaks_min (time_s, value):\n')
        for t, v in right_peaks['min_peaks']:
            mf.write(f"    - {t:.3f}s: {v:.6f}\n")
        mf.write('\n')
        mf.write('actuators:\n')
        try:
            mf.write(f"  n_mimo_actuators: {n_mim_act}\n")
            mf.write(f"  left_actuator_index: {idx_left}\n")
            mf.write(f"  right_actuator_index: {idx_right}\n")
            mf.write('  actuator_names:\n')
            for i, nm in enumerate(actuator_names):
                mf.write(f"    - {i}: {nm}\n")
        except Exception:
            pass
        mf.write('\n')
        mf.write('run_config_muscle_blocks:\n')
        if runconfig_blocks:
            for act_name, block in runconfig_blocks.items():
                mf.write(f"  {act_name}:\n")
                mf.write(f"    fmax_neg: {block.get('fmax_neg')}\n")
                mf.write(f"    vmax_neg: {block.get('vmax_neg')}\n")
                mf.write(f"    fmax_pos: {block.get('fmax_pos')}\n")
                mf.write(f"    vmax_pos: {block.get('vmax_pos')}\n")
        else:
            mf.write('  legacy: true\n')
            mf.write(f"  flex_fmax: {legacy_flex_fmax}\n")
            mf.write(f"  flex_vmax: {legacy_flex_vmax}\n")
            mf.write(f"  ext_fmax: {legacy_ext_fmax}\n")
            mf.write(f"  ext_vmax: {legacy_ext_vmax}\n")

    print(f"CSV de ángulos guardado en {CSV_LOG_PATH}")
    print(f"Imagen de gráficas guardada en {IMG_PATH}")
    print(f"Metadatos guardados en {META_PATH}")