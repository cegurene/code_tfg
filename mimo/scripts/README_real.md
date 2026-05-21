# Hip/Knee Motor Control Scripts

This document describes scripts for controlling and validating hip and knee motors with MIMo dynamics, both in simulation and on real hardware via ROS2.

---

## Control Theory: PID Torque Command

All control modes (both `pid` and `muscle`) internally use a **PID controller** to track a reference trajectory and produce a raw torque command:

```
torque_raw = kp * (q_ref - q) + ki * integral_error + kd * (qd_ref - qd)
```

### Component Breakdown

| Term | Meaning | Effect |
|------|---------|--------|
| **`kp * (q_ref - q)`** | Proportional | Pushes motor toward desired angle. Higher `kp` = faster response but more overshoot. |
| **`ki * integral_error`** | Integral | Corrects steady-state bias (friction, gravity). Accumulates position error over time. |
| **`kd * (qd_ref - qd)`** | Derivative | Damping term; smooths motion by opposing velocity mismatches. Higher `kd` = less overshoot but sluggish. |

**Practical tuning:**
- Start with defaults: `kp=1.8, ki=0.15, kd=0.08`
- Increase `kp` if tracking is too slow
- Increase `ki` if final position drifts or doesn't overcome friction
- Increase `kd` if motion is jerky or oscillating

---

## 1) Simulation Validation Script

File: `mimo/scripts/visualiza_standup_trajectory_tracking.py`

Purpose:
- Validate knee tracking in simulation.
- Generate reference trajectories from virtual actuator components.
- Compute tracking metrics and export time-series CSV.

Typical commands:

```bash
# Default validation
python3 visualiza_standup_trajectory_tracking.py --reference-csv healthy_hip_reference.csv

# Faster headless run
python3 visualiza_standup_trajectory_tracking.py --reference-csv healthy_hip_reference.csv --no-render

# Custom PID and reference components
python3 visualiza_standup_trajectory_tracking.py \
  --reference-csv healthy_hip_reference.csv \
  --kp 2.0 --ki 0.2 --kd 0.1 \
Safety behavior in real mode:
```

Key flags:
- `--env`: Gym environment id.
- `--steps`: number of control steps.
- `--warmup-steps`: initial zero-action settling phase.
- `--virtual-actuators`: reference definition `amp,freq,phase;...`.
- `--ref-offset`: reference angle offset in rad.
- `--kp`, `--ki`, `--kd`: PID gains.
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque
```

Purpose:
- Run in simulation (`--mode sim`) or real motor via ROS2 (`--mode real`).
- **Muscle mode** (recommended): Multi-muscle activation model (flexors/extensors) generates torque from reference trajectory tracking.
- **PID mode** (legacy): Direct PID tracking without muscle simulation.
- Automatic telemetry recording (CSV, plots, metrics).

Safety behavior in real mode:
- Torque is **disabled by default** (use `--apply-torque` to enable).
- Watchdog forces zero torque when joint state is stale (`--watchdog-timeout-s`, default 0.25s).
- Torque limiting is optional (`--safe-torque-limit` + `--torque-limit-mode`).

---

### Quick Start: Real Motor with Muscle Control

**Safety-first dry run (no torque sent):**
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque
```

**Full run with torque enabled and adaptive scaling:**
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque \
  --safe-torque-limit 0.5
```

**Asymmetric raw-torque scaling (more authority in extension):**
**Auto-generated folder structure** (when `--run-dir` is not provided):
```
code_tfg/outputs/mimo/
├── new_workflow/
│   └── simulator/
│       └── <timestamp>_trajectory_tracking/
│           ├── hip_tracking_telemetry.csv
│           ├── hip_tracking_summary.png
│           └── simulation_summary.txt
└── runner/
    └── <timestamp>_id=<motor_id>/
        ├── telemetry.csv
        ├── timeseries.png
        ├── run_config.txt
        └── simulation_info.txt
```

Workflow:
1. Trajectory generates reference angle `q_ref` and velocity `qd_ref`
2. PID controller computes `torque_raw` from tracking error
3. Muscle model **synthesizes** flexor/extensor activations from `torque_raw`
4. Final torque sent to motor

Advantages:
- Biologically-inspired muscle activations for logging/visualization
- Inherits trajectory robustness
- Smooth transitions and activation dynamics
- Default flow: `--muscle-flow trajectory_to_torque`

Key muscle flags:
- `--flex-muscles`: flexor units spec `fmax,vmax,amp,freq,phase,bias;...`
- `--ext-muscles`: extensor units spec `fmax,vmax,amp,freq,phase,bias;...`
- `--q-neutral`, `--q-min`, `--q-max`: motor geometry
- `--muscle-tau`: activation time constant (default 0.02s)
- `--lce-min`, `--lce-max`: virtual muscle length range
- `--lmin`, `--lmax`, `--fvmax`, `--fpmax`: force-length/force-velocity curve parameters

Example with custom flexor/extensor definition:
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque
```

#### **PID Mode** (direct trajectory tracking)

Simpler alternative: Pure PID without muscle synthesis.

```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque
```

---

### Reference Trajectory

Supported sources:

**Virtual sine actuators (default):**
```bash
--virtual-actuators "amp1,freq1,phase1;amp2,freq2,phase2;..."
```
- Default: `"0.575,1,1.5707963268"` (single sine, ~33° amplitude, starts at flexion)
- Combines multiple sine components; phase in radians

**CSV trajectory (periodic gait or custom motion):**
```bash
--trajectory-csv path/to/gait.csv \
--trajectory-angle-unit deg \
--trajectory-period-s 1.063 \
--trajectory-phase-offset-pct 0.0 \
--auto-phase-align
```
- Auto-detects columns (`phase_pct` or similar, `hip_angle` or similar)
- `--auto-phase-align`: aligns initial phase to current motor position
- `--trajectory-scale`: amplitude multiplier

Example:
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 \
  --motor-id 11 \
  --apply-torque
```

---

### Torque Limiting Strategies

**Option 1: Hard clipping** (`clip` mode)
- Simple saturation: `torque = clip(torque_raw, -limit, +limit)`
- Use when: Simplicity is more important than smoothness

```bash
--safe-torque-limit 0.5 --torque-limit-mode clip
```

**Option 2: Adaptive scaling** (`scale` mode, recommended)
- Tracks envelope of raw torque and scales down smoothly
- Preserves signal shape; applies soft compression via tanh near limit
- Recovers gain after transients fade

```bash
--safe-torque-limit 0.5 \
--torque-limit-mode scale \
--torque-scale-decay-s 0.6 \
--torque-scale-softness 1.8
```

Tuning adaptive scaling:
- `--torque-scale-decay-s`: envelope recovery time (larger = slower recovery, smoother but less responsive)
- `--torque-scale-softness`: compression strength near limit (larger = more compression)

**Option 3: No limiting**
- Omit both flags for unrestricted (use with caution on real hardware)

```bash
# No --safe-torque-limit or --torque-limit-mode specified
```

---

### Raw Torque Scaling

When `--torque-output-source raw_scaled`, the command is `torque_cmd = torque_raw * scale`:

**Uniform scaling (e.g., send 1/3 of raw torque):**
```bash
--torque-output-source raw_scaled \
--torque-raw-scale 0.3333
```

**Asymmetric scaling (direction-dependent):**
```bash
--torque-output-source raw_scaled \
--torque-raw-scale-pos 0.33 \
--torque-raw-scale-neg 0.45
```
- Useful if motor has different impedance in flexion vs. extension
- `torque_raw_scale_pos`: applied when `torque_raw ≥ 0` (flexion)
- `torque_raw_scale_neg`: applied when `torque_raw < 0` (extension)

Example (pyCandle):
```bash
python3 hip_motor_torque_replay_runner_pycandle.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface pycandle \
  --motor-id 11 \
  --apply-torque
```

---

### Telemetry & Output Files

Flags:
- `--save-csv / --no-save-csv`: time-series CSV (default: true)
- `--csv-path`: custom CSV output path
- `--save-plot / --no-save-plot`: standard figure (default: true)

## Nuevo flujo: replay desde simulación

Los nuevos runners de replay toman el torque exportado por la simulación y lo aplican al motor real. Por defecto los resultados de estas ejecuciones se almacenan en:

- `code_tfg/outputs/mimo/runner/<timestamp>_id=<motor_id>/` — contiene `telemetry.csv`, `timeseries.png`, `run_config.txt` y `simulation_info.txt`.

Ejemplo (ROS2):
```bash
python3 hip_motor_torque_replay_runner.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface ros2 --motor-id 308 --apply-torque
```

Ejemplo (pyCandle):
```bash
python3 hip_motor_torque_replay_runner_pycandle.py \
  --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
  --interface pycandle --motor-id 308 --apply-torque
```

Recuerda que puedes forzar otra carpeta de salida con `--run-dir` y que `--csv-path`/`--plot-path` permiten personalizar ficheros individuales.
- `--plot-path`: custom plot path
- `--save-paper-plot / --no-save-paper-plot`: paper-style summary with metrics (default: true)
- `--paper-plot-path`: custom paper-plot path
- `--show-plot / --no-show-plot`: display window at end
- `--run-dir`: custom per-run output folder

**Auto-generated folder structure** (when `--run-dir` is not provided):
```
code_tfg/outputs/mimo/
├── new_workflow/
│   └── simulator/
│       └── <timestamp>_trajectory_tracking/
│           ├── hip_tracking_telemetry.csv
│           ├── hip_tracking_summary.png
│           └── simulation_summary.txt
└── runner/
  └── <timestamp>_id=<motor_id>/
    ├── telemetry.csv
    ├── timeseries.png
    ├── run_config.txt
    └── simulation_info.txt
```

**CSV columns:**
- `time_s`, `q_rad`, `qd_rad_s`, `q_des_rad`
- `torque_raw_nm`, `torque_clip_nm`, `torque_cmd_nm`, `torque_scale_gain`
- `u_flex_*`, `u_ext_*`: muscle unit activations (if muscle mode)

---

## Troubleshooting & Best Practices

### Hardware Testing Workflow

1. **Dry run (no torque):**
   ```bash
   python3 hip_motor_torque_replay_runner.py \
     --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
     --interface ros2 --motor-id 11
   ```
   ✓ Verify ROS2 topics are live  
   ✓ Check reference trajectory makes sense  
   ✓ Inspect CSV and plots for signal quality  

2. **Light torque run (adaptive scaling):**
   ```bash
   python3 hip_motor_torque_replay_runner.py \
     --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
     --interface ros2 --motor-id 11 \
     --safe-torque-limit 0.2 --apply-torque
   ```
   ✓ Test motor response with limited authority  
   ✓ Verify muscle activations look reasonable  

3. **Full run:**
   ```bash
   python3 hip_motor_torque_replay_runner.py \
     --source-csv code_tfg/outputs/mimo/new_workflow/simulator/<run_tag>/hip_tracking_telemetry.csv \
     --interface ros2 --motor-id 11 \
     --safe-torque-limit 0.5 --apply-torque --save-csv --save-plot
   ```

### Common Issues

| Issue | Solution |
|-------|----------|
| **No joint state received** | Check ROS2 topics (`ros2 topic list`) and motor connection |
| **Motor doesn't move** | Verify `--apply-torque` is set; check if torque limit is too low |
| **Jerky/oscillating motion** | Reduce `kp`, increase `kd`, or enable `--torque-limit-mode scale` |
| **Tracking lag** | Increase `kp` or `ki`; check if motor has high friction |
| **Saturation/clipping warnings** | Increase `--safe-torque-limit` or reduce `kp` |

### Notes

- **Muscle mode default:** Although the code default is `pid`, **`--control-mode muscle` is the primary mode used in practice**. It provides smooth muscle-inspired activations and better trajectory tracking.
- **Angle alignment visualization:** `--plot-angle-align` only affects plot display; raw control and metrics use unaligned signals.
- **Legacy mode:** Use `--muscle-flow actuators_to_torque` to recover older behavior where muscle activations directly drive torque (not recommended).
- **ROS2 requirement:** Real mode depends on working ROS2 setup and `candle_ros2` or `rl_interfaces` message types.
- **Control loop timing:** The control loop respects `--rate-hz` via wall-clock sleep; jitter depends on system load.
- **CSV data:** Always saved before plots; safe for interrupted runs.

---

## File Locations

- **Trajectory tracking script:** `mimo/scripts/visualiza_standup_trajectory_tracking.py`
- **Hip motor replay scripts:** `mimo/scripts/hip_motor_torque_replay_runner.py` and `mimo/scripts/hip_motor_torque_replay_runner_pycandle.py`
- **Output folder:** `code_tfg/outputs/mimo/` (timestamped per run)
- **This README:** `mimo/scripts/README_real.md`
