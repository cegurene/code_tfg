"""RL_NODE_V7 - Reinforcement Learning Agent with Adjustable Torque Scaling

CRITICAL FIX: Scales model actions to match real motor torque range.

The model was trained in simulation with gear=100 (torque range ~[-100, 100] Nm).
Real MD80 motor has max_torque=9.0 Nm → Need torque scaling.

QUICK ADJUSTMENT:
  Edit line ~40 in this file:
    TORQUE_SCALE = 1.0  # Change this value
  
  Suggested values to try:
    0.09 = Theoretical (too low, motor won't move)
    1.0  = Direct predictions (~0.9 Nm typical)
    5.0  = Medium force (~4.5 Nm typical)
    9.0  = Use full motor range (safe maximum)
  
  After changing, rebuild:
    cd ~/Escritorio/ros2_rl_ws
    colcon build --packages-select rl_package
    source install/setup.bash

Torque flow:
  1. Model outputs: action_normalized in [-1, 1]
  2. Scaled torque: action_normalized × TORQUE_SCALE
  3. Clamped to motor limit: clip to [-9, 9] Nm
  4. Published to motor
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rl_interfaces.msg import MotionCommand as SimMotionCommand
try:
    from candle_ros2.msg import MotionCommand as CandleMotionCommand
except Exception:
    CandleMotionCommand = None

import numpy
import os
import sys
import time
import json
from collections import deque
from pathlib import Path

import torch

WORKSPACE_ROOT = Path(__file__).resolve().parents[4]

MODELS_DIR = WORKSPACE_ROOT / 'outputs'
ENV_MODELS_DIR = MODELS_DIR / 'PPO' / 'nair_walk_h0918KneeExoRS2-v0'

TIME_MODELS_DIR = ENV_MODELS_DIR / '2026-05-19' / '11-48-17'

best_model_path = TIME_MODELS_DIR / 'checkpoint_1060864.pt'

class MinimalPublisher(Node):

    # ============================================================================
    # CONFIGURACIÓN DE ESCALADO DE TORQUE - MODIFICAR AQUÍ Y RECOMPILAR
    # ============================================================================
    # Ajusta TORQUE_SCALE hasta encontrar el valor que mueve el motor correctamente
    # 
    # Valores sugeridos para probar (de menor a mayor fuerza):
    #   TORQUE_SCALE = 0.09   # Teórico (9/100) - DEMASIADO BAJO, motor no se mueve
    #   TORQUE_SCALE = 1.0    # Usa predicciones directas (~0.9 Nm, como v4)
    #   TORQUE_SCALE = 5.0    # Fuerza media (~4.5 Nm con predicciones típicas)
    #   TORQUE_SCALE = 9.0    # Usa rango completo del motor (máximo seguro)
    #
    # Después de cambiar, ejecutar:
    #   cd ~/Escritorio/ros2_rl_ws
    #   colcon build --packages-select rl_package
    #   source install/setup.bash
    # ============================================================================
    
    TORQUE_SCALE =  1 # <-- CAMBIAR ESTE VALOR Y RECOMPILAR
    
    MAX_TORQUE = 9.0  # Nm (límite máximo del motor MD80, NO modificar)
    # SIM2REAL alignment: Sign to apply to torque when sending to real motor
    # If motor range is inverted (exo_sign = -1), flip torque direction
    EXO_SIGN = 1.0  # Default: 1.0. Set to -1.0 if motor angle range is inverted
    SAFE_TORQUE_LIMIT = 2.0  # Nm hard cap for real hardware (safe default)
    STARTUP_RAMP_SECONDS = 3.0  # seconds to ramp from 0 to SAFE_TORQUE_LIMIT
    OBS_TIMEOUT_SEC = 0.20  # watchdog: if no observations, force zero torque
    APPLY_TORQUE = False  # False = inference only (safe), True = actuate hardware
    ENABLE_SAFETY_LIMITS = True  # True = apply range/velocity E-STOP checks
    SELF_TEST_TORQUE = 0.4  # Nm (safe default)
    SELF_TEST_DURATION = 0.25  # s por pulso (safe default)
    LIMIT_MARGIN_RAD = 0.10  # extra margin outside calibrated limits
    LIMIT_SOFT_ZONE_RAD = 0.03  # proactive zone near limits to block outward torque
    LIMIT_HOLD_KP = 8.0  # Nm/rad proportional hold near limit
    LIMIT_HOLD_KD = 0.25  # Nms/rad damping near limit
    LIMIT_HOLD_MAX_TORQUE = 0.8  # Nm max hold torque command at limit
    MAX_SAFE_VEL_RAD_S = 6.0  # hard emergency velocity limit
    ABS_POS_HARD_LIMIT_RAD = 10.0  # hard absolute position limit (encoder runaway guard)

    @staticmethod
    def _env_bool(name, default):
        value = os.getenv(name)
        if value is None:
            return default
        return value.strip().lower() in ('1', 'true', 'yes', 'y', 'on')

    def _configure_runtime_mode(self):
        """Configure torque application mode at runtime (interactive + env overrides)."""
        # Optional environment overrides
        self.APPLY_TORQUE = self._env_bool('RL_APPLY_TORQUE', self.APPLY_TORQUE)
        self.ENABLE_SAFETY_LIMITS = self._env_bool('RL_ENABLE_SAFETY_LIMITS', self.ENABLE_SAFETY_LIMITS)

        env_safe_limit = os.getenv('RL_SAFE_TORQUE_LIMIT')
        if env_safe_limit is not None:
            try:
                self.SAFE_TORQUE_LIMIT = max(0.1, min(self.MAX_TORQUE, float(env_safe_limit)))
            except ValueError:
                self.get_logger().warn(f"Invalid RL_SAFE_TORQUE_LIMIT='{env_safe_limit}', using default.")

        env_scale = os.getenv('RL_TORQUE_SCALE')
        if env_scale is not None:
            try:
                self.TORQUE_SCALE = max(0.01, min(self.MAX_TORQUE, float(env_scale)))
            except ValueError:
                self.get_logger().warn(f"Invalid RL_TORQUE_SCALE='{env_scale}', using default.")

        # Interactive override (safest for manual testing)
        answer = input("Enable REAL torque output to motor now? [y/N]: ").strip().lower()
        self.APPLY_TORQUE = answer in ('y', 'yes')

        limits_answer = input("Enable safety limits (range/velocity E-STOP)? [y/N]: ").strip().lower()
        self.ENABLE_SAFETY_LIMITS = limits_answer in ('y', 'yes')

        if self.APPLY_TORQUE:
            default_limit = min(self.SAFE_TORQUE_LIMIT, 0.5)
            raw_limit = input(f"Safe torque limit in Nm for this run (default {default_limit:.2f}): ").strip()
            if raw_limit:
                try:
                    self.SAFE_TORQUE_LIMIT = max(0.1, min(self.MAX_TORQUE, float(raw_limit)))
                except ValueError:
                    self.get_logger().warn(f"Invalid torque limit '{raw_limit}', using {default_limit:.2f} Nm.")
                    self.SAFE_TORQUE_LIMIT = default_limit
            else:
                self.SAFE_TORQUE_LIMIT = default_limit

            self.get_logger().warn(
                f"REAL TORQUE ENABLED for this run. SAFE_TORQUE_LIMIT={self.SAFE_TORQUE_LIMIT:.2f} Nm"
            )
        else:
            self.get_logger().info("Inference-only mode enabled (Torque_sent=0.0 Nm).")

        self.get_logger().warn(
            f"Safety limits enabled: {self.ENABLE_SAFETY_LIMITS}"
        )

    def __init__(self):
        """Initialize RL agent with torque scaling for real motor."""
        super().__init__('rl_node_v7')
        
        # Select environment mode: 0 = simulator only, 1 = simulator + hardware
        print("\nRL Environment Selection:")
        print("=" * 70)
        print("0: MyoSuite simulator only (no motor hardware required)")
        print("1: MyoSuite simulator + MD80 motor (requires hardware connection)")
        print("=" * 70)
        self.control_type = int(input("Select RL environment [0 or 1] (default=1): ") or "1")
        
        if self.control_type not in (0, 1):
            raise ValueError(f"Invalid environment selection: {self.control_type}")
        
        # Publishers
        self.hardware_publisher_ = None
        if CandleMotionCommand is not None and self.control_type == 1:
            try:
                self.hardware_publisher_ = self.create_publisher(
                    CandleMotionCommand, '/md80/motion_command', 10
                )
            except Exception as e:
                self.hardware_publisher_ = None
                self.get_logger().error(
                    f"No se pudo crear publisher de hardware candle_ros2: {e}"
                )
                self.get_logger().error(
                    "Revisa candle_ros2 (typesupport roto o workspace mal sourced). "
                    "Se continuará SIN publicar a hardware."
                )

        sim_topic = '/rl/motion_command' if CandleMotionCommand is not None else '/md80/motion_command'
        self.sim_publisher_ = self.create_publisher(SimMotionCommand, sim_topic, 10)
        
        # Current motor state
        self.current_pos = None
        self.current_vel = None
        self.last_obs_time = None
        self._startup_time = time.time()
        self._e_stop_latched = False
        self._last_watchdog_warn_time = 0.0
        self._sat_count = 0
        self._sample_count = 0
        self._diag_window = deque(maxlen=200)
        self._obs_norm_enabled = False
        self._obs_norm_mean = None
        self._obs_norm_std = None
        self._obs_norm_clip = 5.0
        
        # SIM2REAL alignment: Read exo_sign from environment (matches aggregator)
        exo_sign_raw = os.getenv('RL_EXO_SIGN', '1')
        self.EXO_SIGN = -1.0 if exo_sign_raw.strip() in ('-1', '-1.0') else 1.0
        
        # Subscribe to joint states only if hardware mode
        if self.control_type == 1:
            self.joint_subscription = self.create_subscription(
                JointState,
                '/md80/joint_states',
                self.joint_state_callback,
                10
            )
            self.joint_subscription
        else:
            self.joint_subscription = None
        
        # Motor ID input - only needed for hardware mode
        if self.control_type == 1:
            self.motor_id = int(input("Enter the motor ID to control: "))
        else:
            self.motor_id = None
        
        # Wait for motor connection only in hardware mode
        if self.control_type == 1:
            print("\n⏳ Waiting for joint state data from motor...")
            wait_time = 0.0
            max_wait = 5.0
            timeout_per_spin = 0.2
            
            while self.current_pos is None and wait_time < max_wait:
                rclpy.spin_once(self, timeout_sec=timeout_per_spin)
                wait_time += timeout_per_spin
            
            if self.current_pos is None:
                self.get_logger().error("ERROR: No joint state received after 5 seconds.")
                raise RuntimeError("Cannot connect to motor")
            
            print(f"✓ Motor connected. Initial position: {self.current_pos:.4f} rad\n")
        else:
            print("\n✓ Simulator-only mode. No hardware connection required.\n")

        self._configure_runtime_mode()
        
        # Display torque scaling info
        print("="*70)
        print("TORQUE SCALING CONFIGURATION")
        print("="*70)
        print(f"Torque scale factor:    {self.TORQUE_SCALE:.2f}")
        print(f"Motor max torque:       {self.MAX_TORQUE:.1f} Nm")
        print(f"SAFE torque limit:      {self.SAFE_TORQUE_LIMIT:.2f} Nm")
        # SIM2REAL sign will be displayed after angle calibration
        print(f"Startup ramp:           {self.STARTUP_RAMP_SECONDS:.1f} s")
        print(f"Obs watchdog timeout:   {self.OBS_TIMEOUT_SEC:.2f} s")
        print(f"Apply torque to motor:  {self.APPLY_TORQUE}")
        print(f"Safety limits enabled:  {self.ENABLE_SAFETY_LIMITS}")
        print(f"Limit soft zone:        {self.LIMIT_SOFT_ZONE_RAD:.3f} rad")
        print(f"Limit hold gains:       Kp={self.LIMIT_HOLD_KP:.2f}, Kd={self.LIMIT_HOLD_KD:.2f}")
        print(f"Limit hold max torque:  {self.LIMIT_HOLD_MAX_TORQUE:.2f} Nm")
        print(f"Effective range model:  [{-self.TORQUE_SCALE:.2f}, {self.TORQUE_SCALE:.2f}] Nm")
        print(f"Effective range motor:  [{-self.MAX_TORQUE:.1f}, {self.MAX_TORQUE:.1f}] Nm (hard limit)")
        print("")
        print("Para cambiar el escalado: Editar rl_node_v7.py línea ~40")
        print("Valores sugeridos: 0.09 (muy bajo), 1.0 (medio), 5.0 (alto), 9.0 (máximo)")
        print("="*70 + "\n")
        
        # Perform angle calibration only in hardware mode to auto-detect motor sign (EXO_SIGN)
        if self.control_type == 1:
            self._calibrate_angle_limits()
            
            # User can still disable safety limit enforcement separately
            if not self.ENABLE_SAFETY_LIMITS:
                self.get_logger().warn("Safety limit ENFORCEMENT disabled, but angle range auto-detected for sign correction.")
        else:
            print("[SIMULATOR MODE] Skipping angle calibration. Using EXO_SIGN=1.0 (can be overridden with RL_EXO_SIGN)\n")
        
        # Subscribe to observations
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/rl/observations',
            self.listener_callback,
            10)
        self.subscription

        self.get_logger().info('rl_node_v7 iniciado (con escalado de torque)')
        self._warned_no_model = False
        self.model = self._load_model(best_model_path)
        if self.model is None:
            self.get_logger().warn('Modelo no cargado. Se enviará torque 0.0.')
        else:
            self.get_logger().info('Modelo cargado correctamente.')

        # Optional actuator self-test to verify hardware path independent of inference (hardware mode only)
        if self.control_type == 1:
            self._maybe_run_actuator_self_test()

    def _calibrate_angle_limits(self):
        """Interactive angle limit calibration for SIM2REAL sign detection."""
        import math
        import threading
        
        print("\n" + "="*70)
        print("INTERACTIVE ANGLE CALIBRATION FOR MOTOR RANGE DETECTION")
        print("="*70)
        
        # Step 1: FLEXED position (knee bent - typically higher encoder value)
        print("\n📍 STEP 1: FLEXED KNEE POSITION")
        print("-" * 70)
        print("1. Manually move the motor to the FLEXED position (doblado / rodilla doblada)")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at FLEXED position\n")
        
        self._wait_for_user_input_and_capture("min")  # Named min but semantically FLEXED
        print(f"✓ FLEXED position captured: {self.angle_min:.4f} rad ({math.degrees(self.angle_min):.2f}°)")
        
        # Step 2: EXTENDED position (knee straight - typically lower encoder value)
        print("\n📍 STEP 2: EXTENDED KNEE POSITION")
        print("-" * 70)
        print("1. Manually move the motor to the EXTENDED position (estirado / rodilla estirada)")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at EXTENDED position\n")
        
        self._wait_for_user_input_and_capture("max")  # Named max but semantically EXTENDED
        print(f"✓ EXTENDED position captured: {self.angle_max:.4f} rad ({math.degrees(self.angle_max):.2f}°)")
        
        # Auto-detect if captured values are inverted
        # (flexed > extended numerically = motor range is inverted)
        if self.angle_min > self.angle_max:
            # Don't swap - keep semantic order (flexed first, extended second)
            # Instead, auto-detect inverted range and adjust exo_sign
            self.EXO_SIGN = -1.0
            self.get_logger().warn(
                f"Motor range detected as INVERTED (flexed={self.angle_min:.4f} > extended={self.angle_max:.4f}). "
                f"Auto-adjusting: EXO_SIGN = {self.EXO_SIGN:+.0f}"
            )
        
        # Summary
        angle_range = self.angle_max - self.angle_min
        print("\n" + "="*70)
        print("✓ ANGLE LIMIT CALIBRATION COMPLETE")
        print("="*70)
        print(f"Minimum angle: {self.angle_min:.4f} rad ({math.degrees(self.angle_min):.2f}°)")
        print(f"Maximum angle: {self.angle_max:.4f} rad ({math.degrees(self.angle_max):.2f}°)")
        print(f"Safe range:    {angle_range:.4f} rad ({math.degrees(angle_range):.2f}°)")
        print(f"Motor ID:      {self.motor_id}")
        print(f"SIM2REAL sign (exo): {self.EXO_SIGN:+.0f}")
        print("="*70 + "\n")
        
        self.get_logger().info(
            f"Limits: min={self.angle_min:.4f} rad, max={self.angle_max:.4f} rad"
        )

    def _wait_for_user_input_and_capture(self, limit_type):
        """Wait for ENTER while displaying current position."""
        import math
        import threading
        
        user_ready = threading.Event()
        
        def wait_for_input():
            input()
            user_ready.set()
        
        input_thread = threading.Thread(target=wait_for_input, daemon=True)
        input_thread.start()
        
        while not user_ready.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pos is not None:
                print(f"\rCurrent position: {self.current_pos:.4f} rad ({math.degrees(self.current_pos):.2f}°)  ", end="", flush=True)
        
        if limit_type == "min":
            self.angle_min = self.current_pos
        elif limit_type == "max":
            self.angle_max = self.current_pos
        
        input_thread.join(timeout=1.0)

    def _load_model(self, model_path):
        """Load TorchRL PPO model."""
        try:
            checkpoint = torch.load(model_path, map_location='cpu')
            
            if 'obs_dim' not in checkpoint or 'act_dim' not in checkpoint:
                raise RuntimeError("Checkpoint sin obs_dim/act_dim")
            
            obs_dim = checkpoint['obs_dim']
            act_dim = checkpoint['act_dim']

            env_id = checkpoint.get('env_id', None)
            if env_id is not None:
                self.get_logger().info(f"Checkpoint env_id={env_id}")

            self._obs_norm_enabled = bool(checkpoint.get('obs_norm_enabled', False))
            self._obs_norm_clip = float(checkpoint.get('obs_norm_clip', 5.0))
            obs_norm_mean = checkpoint.get('obs_norm_mean', None)
            obs_norm_std = checkpoint.get('obs_norm_std', None)

            if self._obs_norm_enabled and obs_norm_mean is not None and obs_norm_std is not None:
                mean_tensor = torch.as_tensor(obs_norm_mean, dtype=torch.float32).reshape(-1)
                std_tensor = torch.as_tensor(obs_norm_std, dtype=torch.float32).reshape(-1)

                if mean_tensor.numel() == obs_dim and std_tensor.numel() == obs_dim:
                    self._obs_norm_mean = mean_tensor
                    self._obs_norm_std = torch.clamp(std_tensor, min=1e-6)
                    self.get_logger().info(
                        f"Observation normalization loaded from checkpoint (clip=±{self._obs_norm_clip:.2f})."
                    )
                else:
                    self._obs_norm_enabled = False
                    self._obs_norm_mean = None
                    self._obs_norm_std = None
                    self.get_logger().warn(
                        "obs_norm tensors incompatible with obs_dim. Disabling normalization."
                    )
            else:
                self._obs_norm_enabled = False
                self._obs_norm_mean = None
                self._obs_norm_std = None
                self.get_logger().info("Checkpoint without obs normalization. Using raw observations.")

            metadata_sidecar = model_path + '.meta.json'
            if os.path.exists(metadata_sidecar):
                try:
                    with open(metadata_sidecar, 'r', encoding='utf-8') as metadata_file:
                        metadata = json.load(metadata_file)
                    self.get_logger().info(
                        f"Checkpoint metadata: env_id={metadata.get('env_id', 'unknown')} "
                        f"obs_dim={metadata.get('obs_dim', '?')} act_dim={metadata.get('act_dim', '?')}"
                    )
                except Exception as sidecar_error:
                    self.get_logger().warn(f"Could not parse metadata sidecar: {sidecar_error}")
            
            self.get_logger().info(f"Loading: obs_dim={obs_dim}, act_dim={act_dim}")
            
            from torchrl.modules import ProbabilisticActor
            from torchrl.modules.distributions.continuous import TanhNormal
            from tensordict.nn import TensorDictModule
            import torch.nn as nn
            
            class ActorNetwork(nn.Module):
                def __init__(self, obs_dim, act_dim):
                    super().__init__()
                    self.net = nn.Sequential(
                        nn.Linear(obs_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, act_dim),
                    )
                    self.log_std = nn.Parameter(torch.zeros(act_dim))
                
                def forward(self, obs):
                    mean = self.net(obs)
                    std = torch.exp(self.log_std).expand_as(mean)
                    return mean, std
            
            actor_net = ActorNetwork(obs_dim, act_dim)
            
            policy_module = TensorDictModule(
                actor_net,
                in_keys=["observation"],
                out_keys=["loc", "scale"]
            )
            
            policy = ProbabilisticActor(
                module=policy_module,
                in_keys=["loc", "scale"],
                distribution_class=TanhNormal,
                distribution_kwargs={"low": -1.0, "high": 1.0},
                return_log_prob=False,
            )
            
            policy.load_state_dict(checkpoint['actor'], strict=True)
            policy.eval()
            
            self.get_logger().info(f"✓ Model loaded from step {checkpoint.get('step', '?')}")
            return policy
            
        except FileNotFoundError:
            raise RuntimeError(f"Model not found: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def _publish_torque(self, torque_value):
        """Publish torque command to hardware and sim topics."""
        torque_clamped = float(numpy.clip(torque_value, -self.MAX_TORQUE, self.MAX_TORQUE))

        if self.hardware_publisher_ is not None and self.motor_id is not None:
            msg_hw = CandleMotionCommand()
            msg_hw.drive_ids = [self.motor_id]
            msg_hw.target_position = [0.0]
            msg_hw.target_velocity = [0.0]
            # Apply EXO_SIGN to match aggregator's observation direction
            # If motor range is inverted, flip torque direction accordingly
            torque_for_hw = torque_clamped * self.EXO_SIGN
            msg_hw.target_torque = [torque_for_hw]
            self.hardware_publisher_.publish(msg_hw)

        # Publish to simulator (works in both modes)
        msg_sim = SimMotionCommand()
        msg_sim.drive_ids = [self.motor_id if self.motor_id is not None else 1]
        msg_sim.target_position = [0.0]
        msg_sim.target_velocity = [0.0]
        msg_sim.target_torque = [torque_clamped]
        self.sim_publisher_.publish(msg_sim)

    def _maybe_run_actuator_self_test(self):
        """Run optional +torque/-torque pulse test to validate MD80 actuation path."""
        try:
            answer = input(
                f"\nRun actuator self-test (+/-{self.SELF_TEST_TORQUE:.1f} Nm, {self.SELF_TEST_DURATION:.2f}s each, SAFE mode)? [y/N]: "
            ).strip().lower()
        except Exception:
            answer = "n"

        if answer not in ("y", "yes"):
            self.get_logger().info("Self-test skipped.")
            return

        self.get_logger().warn(
            "SELF-TEST enabled. Keep the exoskeleton supported and be ready to stop if needed."
        )

        if self.current_pos is None:
            self.get_logger().warn("Self-test skipped: no joint state available.")
            return

        start_pos = float(self.current_pos)
        self.get_logger().info(
            f"SELF-TEST start | pos={start_pos:.3f} rad | applying +{self.SELF_TEST_TORQUE:.1f} Nm"
        )

        end_time = time.time() + self.SELF_TEST_DURATION
        while time.time() < end_time:
            self._publish_torque(self.SELF_TEST_TORQUE)
            rclpy.spin_once(self, timeout_sec=0.01)

        mid_pos = float(self.current_pos) if self.current_pos is not None else start_pos
        self.get_logger().info(
            f"SELF-TEST mid | pos={mid_pos:.3f} rad | applying -{self.SELF_TEST_TORQUE:.1f} Nm"
        )

        end_time = time.time() + self.SELF_TEST_DURATION
        while time.time() < end_time:
            self._publish_torque(-self.SELF_TEST_TORQUE)
            rclpy.spin_once(self, timeout_sec=0.01)

        end_pos = float(self.current_pos) if self.current_pos is not None else mid_pos
        self._publish_torque(0.0)

        delta_positive = mid_pos - start_pos
        delta_negative = end_pos - mid_pos
        self.get_logger().info(
            f"SELF-TEST result | Δpos(+pulse)={delta_positive:.4f} rad | Δpos(-pulse)={delta_negative:.4f} rad"
        )

        if abs(delta_positive) < 0.01 and abs(delta_negative) < 0.01:
            self.get_logger().error(
                "SELF-TEST: No observable movement. Commands are published, but actuator may not be in torque mode or not applying torque."
            )
        else:
            self.get_logger().info("SELF-TEST: Actuator responds to torque commands.")

    def listener_callback(self, msg):
        """Process observations with torque scaling."""
        observations = msg.data
        self.last_obs_time = time.time()
        
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        self._callback_count += 1
        
        # Log observations every 4 callbacks (1/4 frequency)
        if self._callback_count % 60 == 0:
            # Show first 3 values + critical exo values (pos/vel/acc at indices 7/23/39)
            exo_pos = observations[7] if len(observations) > 7 else 0.0
            exo_vel = observations[23] if len(observations) > 23 else 0.0
            exo_acc = observations[39] if len(observations) > 39 else 0.0
            print()
            self.get_logger().info(
                f'📥 Obs[{len(observations)}]: [time={observations[0]:.2f}] | '
                f'EXO: pos={exo_pos:.3f} vel={exo_vel:.3f} acc={exo_acc:.3f}'
            )

        if self.model is None:
            act_normalized = 0.0
            if not self._warned_no_model:
                self.get_logger().warn("Model not loaded. Zero torque.")
                self._warned_no_model = True
        else:
            from tensordict import TensorDict
            obs_tensor = torch.tensor(observations, dtype=torch.float32).unsqueeze(0)

            if self._obs_norm_enabled and self._obs_norm_mean is not None and self._obs_norm_std is not None:
                obs_tensor = (obs_tensor - self._obs_norm_mean.unsqueeze(0)) / self._obs_norm_std.unsqueeze(0)
                obs_tensor = torch.nan_to_num(obs_tensor, nan=0.0, posinf=self._obs_norm_clip, neginf=-self._obs_norm_clip)
                obs_tensor = torch.clamp(obs_tensor, -self._obs_norm_clip, self._obs_norm_clip)
            
            tensordict_obs = TensorDict(
                {"observation": obs_tensor},
                batch_size=[1],
            )
            
            with torch.no_grad():
                action_dict = self.model(tensordict_obs)
                action_tensor = action_dict["action"].squeeze(0)
            
            # Normalized action in [-1, 1]
            act_normalized = action_tensor[0].item() if action_tensor.numel() > 0 else 0.0
        
        # TORQUE SCALING: Convert normalized action to real motor torque
        # Note: EXO_SIGN correction is applied in _publish_torque() before sending to hardware
        raw_torque = -act_normalized * self.TORQUE_SCALE
        act_scaled = raw_torque
        
        # Log model output every 100 callbacks
        if self._callback_count % 60 == 0 and self.model is not None:
            self.get_logger().info(
                f'🤖 Model: {act_normalized:.3f} (norm) → {act_scaled:.4f} Nm (scaled by {self.TORQUE_SCALE:.3f})'
            )
        
        # SAFETY LIMITER with scaled torque
        # ⚠️ TEMPORARILY DISABLED FOR TESTING - UNCOMMENT TO RE-ENABLE
        safety_override = False
        """
        if self.current_pos is not None:
            limit_kp = 2.0   # Reduced gain for scaled torques
            limit_kd = 0.1
            max_return_torque = self.MAX_TORQUE * 0.5  # 50% of max for safety return
            
            vel = self.current_vel if self.current_vel is not None else 0.0
            
            if self.current_pos < self.angle_min:
                error = self.angle_min - self.current_pos
                return_torque = (limit_kp * error) - (limit_kd * vel)
                return_torque = min(max_return_torque, max(0.0, return_torque))
                self.get_logger().warn(
                    f"⚠️  LIMIT LOW: pos={self.current_pos:.3f} < min={self.angle_min:.3f}. "
                    f"Override: {act_scaled:.3f} → +{return_torque:.3f} Nm"
                )
                act_scaled = return_torque
                safety_override = True
                
            elif self.current_pos > self.angle_max:
                error = self.current_pos - self.angle_max
                return_torque = -((limit_kp * error) + (limit_kd * vel))
                return_torque = max(-max_return_torque, min(0.0, return_torque))
                self.get_logger().warn(
                    f"⚠️  LIMIT HIGH: pos={self.current_pos:.3f} > max={self.angle_max:.3f}. "
                    f"Override: {act_scaled:.3f} → {return_torque:.3f} Nm"
                )
                act_scaled = return_torque
                safety_override = True
        """
        
        # Clamp to motor limits + additional safe cap and startup ramp
        after_motor_clip = float(numpy.clip(act_scaled, -self.MAX_TORQUE, self.MAX_TORQUE))

        ramp_elapsed = time.time() - self._startup_time
        ramp_factor = min(1.0, max(0.0, ramp_elapsed / self.STARTUP_RAMP_SECONDS))
        dynamic_safe_limit = self.SAFE_TORQUE_LIMIT * ramp_factor
        act_final = float(numpy.clip(after_motor_clip, -dynamic_safe_limit, dynamic_safe_limit))

        # Angle-limit guard (non-latching):
        # If policy pushes outward near a limit, replace command with a bounded
        # PD hold torque around the boundary (instead of forcing 0 Nm).
        # This keeps position near the limit while preventing overshoot.
        if self.ENABLE_SAFETY_LIMITS and self.current_pos is not None:
            pos_now = float(self.current_pos)
            vel_now = float(self.current_vel) if self.current_vel is not None else 0.0
            soft_min = self.angle_min + self.LIMIT_SOFT_ZONE_RAD
            soft_max = self.angle_max - self.LIMIT_SOFT_ZONE_RAD

            hold_active = False
            hold_side = None
            hold_torque = 0.0

            if pos_now >= soft_max and act_final > 0.0:
                target_pos = self.angle_max
                hold_torque = (self.LIMIT_HOLD_KP * (target_pos - pos_now)) - (self.LIMIT_HOLD_KD * vel_now)
                hold_torque = float(numpy.clip(
                    hold_torque,
                    -self.LIMIT_HOLD_MAX_TORQUE,
                    self.LIMIT_HOLD_MAX_TORQUE,
                ))
                if pos_now >= self.angle_max and hold_torque > 0.0:
                    hold_torque = 0.0

                act_final = float(numpy.clip(hold_torque, -dynamic_safe_limit, dynamic_safe_limit))
                hold_active = True
                hold_side = "HIGH"
            elif pos_now <= soft_min and act_final < 0.0:
                target_pos = self.angle_min
                hold_torque = (self.LIMIT_HOLD_KP * (target_pos - pos_now)) - (self.LIMIT_HOLD_KD * vel_now)
                hold_torque = float(numpy.clip(
                    hold_torque,
                    -self.LIMIT_HOLD_MAX_TORQUE,
                    self.LIMIT_HOLD_MAX_TORQUE,
                ))
                if pos_now <= self.angle_min and hold_torque < 0.0:
                    hold_torque = 0.0

                act_final = float(numpy.clip(hold_torque, -dynamic_safe_limit, dynamic_safe_limit))
                hold_active = True
                hold_side = "LOW"

            if hold_active:
                safety_override = True
                # Log limit warnings every 4 callbacks
                if self._callback_count % 60 == 0:
                    self.get_logger().warn(
                        f"LIMIT-{hold_side}: hold torque near boundary | "
                        f"torque_hold={act_final:.3f} Nm "
                        f"pos={pos_now:.3f} rad range=[{self.angle_min:.3f},{self.angle_max:.3f}] "
                        f"soft_zone={self.LIMIT_SOFT_ZONE_RAD:.3f} rad"
                    )

        # Saturation metrics
        self._sample_count += 1
        if abs(raw_torque - act_final) > 1e-6:
            self._sat_count += 1

        # Rolling diagnostics window
        self._diag_window.append((
            float(observations[7]) if len(observations) > 7 else 0.0,
            float(observations[23]) if len(observations) > 23 else 0.0,
            float(observations[39]) if len(observations) > 39 else 0.0,
            float(raw_torque),
        ))

        # HARD SAFETY / E-STOP (latched until node restart)
        # Only for severe conditions. Angle-limit handling above is non-latching.
        if self.ENABLE_SAFETY_LIMITS and self.APPLY_TORQUE and not self._e_stop_latched and self.current_pos is not None:
            pos_now = float(self.current_pos)
            vel_now = float(self.current_vel) if self.current_vel is not None else 0.0
            over_velocity = abs(vel_now) > self.MAX_SAFE_VEL_RAD_S
            abs_pos_overflow = abs(pos_now) > self.ABS_POS_HARD_LIMIT_RAD

            if over_velocity or abs_pos_overflow:
                self._e_stop_latched = True
                self.APPLY_TORQUE = False
                act_final = 0.0
                self._publish_torque(0.0)
                self.get_logger().error(
                    "E-STOP LATCHED: torque disabled due to safety condition | "
                    f"pos={pos_now:.3f} rad vel={vel_now:.3f} rad/s "
                    f"max_vel={self.MAX_SAFE_VEL_RAD_S:.3f} abs_pos_limit={self.ABS_POS_HARD_LIMIT_RAD:.3f}"
                )
        
        # DEBUG: Log actual command being sent (every 4 callbacks)
        if self._callback_count % 60 == 0:
            sat_pct = (100.0 * self._sat_count / self._sample_count) if self._sample_count > 0 else 0.0
            self.get_logger().info(
                f'💾 COMMAND: raw={raw_torque:.4f} Nm | clipped={act_final:.4f} Nm | safe_limit={dynamic_safe_limit:.3f} Nm | ramp={ramp_factor:.2f} | sat={sat_pct:.1f}%'
            )

        # Variability diagnostics (every 4 callbacks, only if window has data)
        if self._callback_count % 60 == 0 and len(self._diag_window) >= 20:
            window_array = numpy.array(self._diag_window, dtype=float)
            std_pos = float(numpy.std(window_array[:, 0]))
            std_vel = float(numpy.std(window_array[:, 1]))
            std_acc = float(numpy.std(window_array[:, 2]))
            std_torque = float(numpy.std(window_array[:, 3]))
            self.get_logger().info(
                f'📊 DIAG(win={len(self._diag_window)}): std_pos={std_pos:.5f}, std_vel={std_vel:.5f}, std_acc={std_acc:.5f}, std_torque_raw={std_torque:.5f}'
            )
        
        # Publish
        if self.APPLY_TORQUE:
            self._publish_torque(act_final)
            torque_sent = act_final
            mode_label = "✓ MODEL"
        else:
            self._publish_torque(0.0)
            torque_sent = 0.0
            mode_label = "🧪 INFERENCE_ONLY"

        # Log safety/status messages (every 4 callbacks, or always on safety override)
        if self._callback_count % 60 == 0 or safety_override:
            status = "⚠️ SAFETY" if safety_override else mode_label
            vel = self.current_vel if self.current_vel is not None else 0.0
            pos = self.current_pos if self.current_pos is not None else 0.0
            self.get_logger().info(
                f'{status} | Torque_raw={raw_torque:.3f} Nm | Torque_cmd={act_final:.3f} Nm | Torque_sent={torque_sent:.3f} Nm | Pos={pos:.3f} rad | Vel={vel:.3f} rad/s'
            )

    def joint_state_callback(self, msg):
        """Track motor state."""
        if not msg.name:
            return
        
        target_name = f"Joint {self.motor_id}"
        for i, name in enumerate(msg.name):
            if name == target_name:
                if i < len(msg.position):
                    self.current_pos = msg.position[i]
                if i < len(msg.velocity):
                    self.current_vel = msg.velocity[i]
                return

    def _watchdog_timer_callback(self):
        """Force zero torque if observations stop arriving."""
        if self.last_obs_time is None:
            return

        dt = time.time() - self.last_obs_time
        if dt <= self.OBS_TIMEOUT_SEC:
            return

        self._publish_torque(0.0)
        now = time.time()
        if now - self._last_watchdog_warn_time > 1.0:
            self._last_watchdog_warn_time = now
            self.get_logger().warn(
                f"OBS watchdog triggered (dt={dt:.3f}s). Publishing 0.0 Nm."
            )


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_publisher.create_timer(0.05, minimal_publisher._watchdog_timer_callback)

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            minimal_publisher._publish_torque(0.0)
        except Exception:
            pass
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
