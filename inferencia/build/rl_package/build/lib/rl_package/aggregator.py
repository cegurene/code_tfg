"""AGGREGATOR NODE - Data Fusion for RL Inference

This node aggregates observations from multiple sources and publishes them to the RL agent.
It supports two modes:
  - Mode 0: Only MyoSuite simulator (observations pass-through)
  - Mode 1: MyoSuite simulator + Real MD80 motor (data fusion)

Key Features:
  - Synchronization with flags to ensure both data sources are updated
  - Automatic acceleration calculation from velocity differences
  - Safety checks to prevent IndexError on observation arrays
  - Motor ID filtering for multi-motor setups

Changes History:
  - Fixed logical operators: Replaced bitwise '&' with 'and' for boolean logic
  - Added acceleration calculation: acc = (vel_current - vel_prev) / (1/100Hz)
  - Fixed IndexError: Changed msgMd80ExoAcc from list to float
  - Added bounds checking: Validates observation array length before writing
  - Improved logging: Added warnings when observation length is insufficient
"""

import rclpy
from rclpy.node import Node

# Note: Float32MultiArray may not work on ROS2 Humble, as ROS2 developers don't like
# the use of generic message types and is deprecated in Foxy.
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

import numpy
import os
import time
import threading

class MinimalPublisher(Node):

    def __init__(self, control_type):
        """Initialize the Aggregator node.
        
        Args:
            control_type (int): 0 for simulator only, 1 for simulator + real motor
        """
        super().__init__('aggregator')
        
        # Publisher: Sends fused observations to RL agent
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rl/observations', 10)

        self.control_type = control_type

        # Synchronization flags for Mode 1: Only publish when both sources have updated
        self.flagMyosuite = False   # True when new MyoSuite observation received
        self.flagMd80Exo = False    # True when new MD80 state received
        
        # Debug counter
        self.publish_count = 0
        self._rate_limited_skips = 0

        # Log sampling: print 1 out of every N published messages
        log_every_raw = os.getenv("RL_AGG_LOG_EVERY_N", "25")
        try:
            self.agg_log_every_n = max(1, int(log_every_raw))
        except ValueError:
            self.agg_log_every_n = 25
            self.get_logger().warn(
                f"Invalid RL_AGG_LOG_EVERY_N='{log_every_raw}'. Using 25."
            )

        # Output publish rate limiter (applies to /rl/observations)
        # Set RL_AGG_MAX_PUBLISH_HZ <= 0 to disable limiting.
        rate_raw = os.getenv("RL_AGG_MAX_PUBLISH_HZ", "0.0")
        try:
            self.agg_max_publish_hz = float(rate_raw)
        except ValueError:
            self.agg_max_publish_hz = 0.0
            self.get_logger().warn(
                f"Invalid RL_AGG_MAX_PUBLISH_HZ='{rate_raw}'. Using 0.0 (disabled)."
            )

        self._publish_min_dt = (1.0 / self.agg_max_publish_hz) if self.agg_max_publish_hz > 0.0 else 0.0
        self._last_publish_ts = 0.0
        
        # Cached messages from both sources
        self.msgMyosuite = []       # MyoSuite observation array [91 floats]
        self.msgMd80ExoPos = [0]    # Motor position from encoder [rad]
        self.msgMd80ExoVel = [0]    # Motor velocity [rad/s]
        self.msgMd80ExoAcc = 0.0    # Motor acceleration [rad/s²] - CALCULATED (not from motor)
        self.msgMd80ExoEffort = [0] # Motor torque/effort [N·m]
        self._have_real_joint_state = False
        self._latest_real_pos = None

        # Sim2real alignment for knee state substitution (mode 1)
        # The policy is trained with simulator angle reference. Real encoder may start at
        # a different absolute zero, which can push observations out-of-distribution.
        exo_sign_raw = os.getenv("RL_EXO_SIGN", "1")
        self.exo_sign = -1.0 if exo_sign_raw.strip() in ("-1", "-1.0") else 1.0

        self._exo_auto_offset = os.getenv("RL_EXO_AUTO_OFFSET", "1").strip().lower() in (
            "1", "true", "yes", "y", "on"
        )
        self._exo_offset_calibrated = False
        self.exo_pos_offset = 0.0
        self.exo_pos_gain = 1.0

        # Optional range mapping real->sim.
        # Default simulator knee range (as reported): [-105°, +3°].
        # If real range is provided, a linear map is applied:
        #   pos_sim = gain * (sign * pos_real) + offset
        #   gain = (sim_max - sim_min) / (real_max - real_min)
        # Vel/acc are multiplied by gain as well.
        self.sim_pos_min = numpy.deg2rad(float(os.getenv("RL_EXO_SIM_MIN_DEG", "-105.0")))
        self.sim_pos_max = numpy.deg2rad(float(os.getenv("RL_EXO_SIM_MAX_DEG", "3.0")))
        self._clip_to_sim_range = os.getenv("RL_EXO_CLIP_TO_SIM_RANGE", "1").strip().lower() in (
            "1", "true", "yes", "y", "on"
        )

        real_min_raw = os.getenv("RL_EXO_REAL_MIN_DEG")
        real_max_raw = os.getenv("RL_EXO_REAL_MAX_DEG")
        if real_min_raw is not None and real_max_raw is not None:
            try:
                real_min = numpy.deg2rad(float(real_min_raw))
                real_max = numpy.deg2rad(float(real_max_raw))
                real_span = real_max - real_min
                sim_span = self.sim_pos_max - self.sim_pos_min
                if abs(real_span) > 1e-6:
                    self.exo_pos_gain = float(sim_span / real_span)
                else:
                    self.get_logger().warn(
                        "RL_EXO_REAL_MIN_DEG and RL_EXO_REAL_MAX_DEG are too close. Using gain=1.0."
                    )
            except ValueError:
                self.get_logger().warn(
                    f"Invalid real range envs RL_EXO_REAL_MIN_DEG='{real_min_raw}', RL_EXO_REAL_MAX_DEG='{real_max_raw}'. Using gain=1.0."
                )

        gain_raw = os.getenv("RL_EXO_RANGE_GAIN")
        if gain_raw is not None and gain_raw.strip() != "":
            try:
                self.exo_pos_gain = float(gain_raw)
            except ValueError:
                self.get_logger().warn(
                    f"Invalid RL_EXO_RANGE_GAIN='{gain_raw}'. Keeping gain={self.exo_pos_gain:.4f}."
                )

        manual_offset_raw = os.getenv("RL_EXO_POS_OFFSET")
        if manual_offset_raw is not None and manual_offset_raw.strip() != "":
            try:
                self.exo_pos_offset = float(manual_offset_raw)
                self._exo_offset_calibrated = True
                self._exo_auto_offset = False
            except ValueError:
                self.get_logger().warn(
                    f"Invalid RL_EXO_POS_OFFSET='{manual_offset_raw}'. Using automatic calibration."
                )

        # Frequency for acceleration calculation (100 Hz assumed from motor update rate)
        self.msgMd80ExoHz = 100

        self.id = input('Enter the motor ID as a number: ')

        if (control_type == 0):
            # This control_type uses only the virtual MyoSuite environment, receiving the messages and directly sending them to the RL node
            print('Environment selected: MyoSuite simulator only')

            # We create a subscriber to listen to the MyoSuite environment observations
            self.subscription = self.create_subscription(
                Float32MultiArray,              # Message type used by the publisher
                '/env/myosuite_obs',            # topic name
                self.listener_0_callback,       # Method called each time a message is received
                10)                             # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

        elif (control_type == 1):
            # This control type uses both the MyoSuite environment and the MD80 exoskeleton
            # Once it has received a new message from both sources, it will combine them and publish the result to the RL node
            print('Environment selected: MyoSuite simulator and MD80 exoskeleton')

            # We create a subscriber to listen to the MyoSuite environment observations
            self.subscription = self.create_subscription(
                Float32MultiArray,                  # Message type used by the publisher
                '/env/myosuite_obs',                # topic name
                self.listener_1_myosuite_callback,  # Method called each time a message is received
                10)                                 # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

            # We create a subscriber to listen to the MD80 motor
            self.subscription = self.create_subscription(
                JointState,                         # Message type used by the publisher
                '/md80/joint_states',               # topic name
                self.listener_1_exo_callback,       # Method called each time a message is received
                10)                                 # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

        else:
            print('The environment selected is not valid, please restart the node')

        self.get_logger().info(
            f"[SIM2REAL ALIGN] sign={self.exo_sign:+.0f}, auto_offset={self._exo_auto_offset}, "
            f"offset_init={self.exo_pos_offset:.4f} rad, gain={self.exo_pos_gain:.4f}, "
            f"sim_range=[{numpy.rad2deg(self.sim_pos_min):.1f}°, {numpy.rad2deg(self.sim_pos_max):.1f}°], "
            f"clip={self._clip_to_sim_range}"
        )
        self.get_logger().info(
            f"[AGG RATE] max_publish_hz={self.agg_max_publish_hz:.2f}"
            if self.agg_max_publish_hz > 0.0
            else "[AGG RATE] limiter disabled (publishing as fast as data arrives)"
        )
        self.get_logger().info(
            f"[AGG LOG] printing 1 every {self.agg_log_every_n} published messages"
        )

        if self.control_type == 1:
            self._maybe_interactive_real_range_calibration()

    def _wait_for_first_joint_state(self, timeout_sec=5.0):
        """Spin until at least one real joint state has been received."""
        start = time.time()
        while (time.time() - start) < timeout_sec:
            if self._have_real_joint_state and self._latest_real_pos is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._have_real_joint_state and self._latest_real_pos is not None

    def _wait_for_enter_and_capture_signed_pos(self, step_label):
        """Keep spinning and showing current encoder position until ENTER is pressed."""
        user_ready = threading.Event()

        def _wait_input():
            try:
                input()
            except Exception:
                pass
            user_ready.set()

        input_thread = threading.Thread(target=_wait_input, daemon=True)
        input_thread.start()

        while not user_ready.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_real_pos is not None:
                signed_pos = self.exo_sign * float(self._latest_real_pos)
                # Use same format as rl_node_v7: short message to fit in one terminal line
                print(
                    f"\rCurrent position: {signed_pos:.4f} rad ({numpy.rad2deg(signed_pos):.2f}°)  ",
                    end="",
                    flush=True,
                )

        print("")
        result = self.exo_sign * float(self._latest_real_pos) if self._latest_real_pos is not None else None
        input_thread.join(timeout=1.0)
        return result

    def _maybe_interactive_real_range_calibration(self):
        """Optional interactive calibration for real encoder flexed/extended range."""
        env_interactive = os.getenv("RL_EXO_INTERACTIVE_RANGE")
        if env_interactive is not None:
            interactive_enabled = env_interactive.strip().lower() in ("1", "true", "yes", "y", "on")
        else:
            answer = input("Run interactive REAL range calibration (flexed/extended) now? [y/N]: ").strip().lower()
            interactive_enabled = answer in ("y", "yes")

        if not interactive_enabled:
            return

        print("\n" + "=" * 70)
        print("INTERACTIVE REAL RANGE CALIBRATION")
        print("This calibrates encoder range mapping to simulator range.")
        print(
            f"Simulator target range: [{numpy.rad2deg(self.sim_pos_min):.1f}°, {numpy.rad2deg(self.sim_pos_max):.1f}°]"
        )
        print("=" * 70)

        if not self._wait_for_first_joint_state(timeout_sec=6.0):
            self.get_logger().warn(
                "Interactive range calibration skipped: no /md80/joint_states received."
            )
            return

        # STEP 1: Minimum (FLEXED)
        print("\n📍 STEP 1/2: FLEXED KNEE POSITION")
        print("-" * 70)
        print("1. Manually move the motor to the FLEXED knee position (doblado)")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at FLEXED position\n")
        
        real_flexed_signed = self._wait_for_enter_and_capture_signed_pos(
            "STEP 1/2: Move to FLEXED knee position (doblado) and press ENTER"
        )
        if real_flexed_signed is not None:
            print(f"✓ FLEXED position captured: {numpy.rad2deg(real_flexed_signed):.1f}°")
        
        # STEP 2: Maximum (EXTENDED)
        print("\n📍 STEP 2/2: EXTENDED KNEE POSITION")
        print("-" * 70)
        print("1. Manually move the motor to the EXTENDED knee position (estirado)")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at EXTENDED position\n")
        
        real_extended_signed = self._wait_for_enter_and_capture_signed_pos(
            "STEP 2/2: Move to EXTENDED knee position (estirado) and press ENTER"
        )
        if real_extended_signed is not None:
            print(f"✓ EXTENDED position captured: {numpy.rad2deg(real_extended_signed):.1f}°")

        if real_flexed_signed is None or real_extended_signed is None:
            self.get_logger().warn("Interactive range calibration aborted: missing captured position.")
            return

        # IMPORTANT: Check if captured values are inverted (FLEXED > EXTENDED numerically)
        # This indicates motor range is inverted, so adjust exo_sign automatically
        # BUT: Keep semantic order (flexed first, extended second as captured by user)
        if real_flexed_signed > real_extended_signed:
            self.exo_sign = -1.0
            self.get_logger().warn(
                f"Motor range detected as INVERTED (flexed={real_flexed_signed:.4f} > extended={real_extended_signed:.4f}). "
                f"Auto-adjusting: exo_sign = {self.exo_sign:+.0f}"
            )

        # IMPORTANT: Do not reorder captured values.
        # We preserve biomechanics semantics (flexed -> extended), even if numerically
        # flexed angle is greater than extended angle. This may produce negative gain,
        # which is valid and encodes axis direction.
        real_span = real_extended_signed - real_flexed_signed
        sim_span = self.sim_pos_max - self.sim_pos_min
        if abs(real_span) < 1e-6:
            self.get_logger().warn(
                "Interactive range calibration invalid: flexed and extended are too close. Keeping previous gain."
            )
            return

        self.exo_pos_gain = float(sim_span / real_span)

        # Calculate offset so that real_extended maps to sim_min:
        # sim_min = exo_sign * real_extended * exo_pos_gain + exo_pos_offset
        # exo_pos_offset = sim_min - exo_sign * real_extended * exo_pos_gain
        if self._exo_auto_offset:
            self.exo_pos_offset = self.sim_pos_min - (self.exo_sign * real_extended_signed * self.exo_pos_gain)
            self._exo_offset_calibrated = True
            self.get_logger().info(
                f"[SIM2REAL ALIGN] Offset computed from calibration: {self.exo_pos_offset:+.4f} rad "
                f"(maps real_extended={real_extended_signed:+.4f} to sim_min={self.sim_pos_min:+.4f})"
            )

        print("\n" + "=" * 70)
        print("✓ REAL RANGE CALIBRATION COMPLETE")
        print(
            f"Real FLEXED (signed):   {numpy.rad2deg(real_flexed_signed):.1f}°"
        )
        print(
            f"Real EXTENDED (signed): {numpy.rad2deg(real_extended_signed):.1f}°"
        )
        print(
            f"Sim target range:  [{numpy.rad2deg(self.sim_pos_min):.1f}°, {numpy.rad2deg(self.sim_pos_max):.1f}°]"
        )
        print(f"Computed gain:     {self.exo_pos_gain:.4f}")
        print(f"EXO sign:          {self.exo_sign:+.0f}")
        print("=" * 70 + "\n")

        self.get_logger().info(
            f"[SIM2REAL ALIGN] Interactive range calibrated: gain={self.exo_pos_gain:.4f} exo_sign={self.exo_sign:+.0f} "
            f"from real_flexed={real_flexed_signed:+.4f} rad, real_extended={real_extended_signed:+.4f} rad"
        )

    def listener_0_callback(self, msg):
        # Log the received message to the console
        #self.get_logger().info(f'Myosuite: "{msg.data}"')
        # Directly try to publish the message received by the MyoSuite enviroment, as they have the same structure
        self.publish(msg)

    def _rate_limit_allows_publish(self):
        """Return True when enough time elapsed since last publication."""
        if self.agg_max_publish_hz <= 0.0:
            return True

        now = time.time()
        if (now - self._last_publish_ts) < self._publish_min_dt:
            self._rate_limited_skips += 1
            if self._rate_limited_skips % 200 == 1:
                self.get_logger().info(
                    f"[AGG RATE] Skipping publish to hold {self.agg_max_publish_hz:.2f} Hz max "
                    f"(skipped={self._rate_limited_skips})",
                    throttle_duration_sec=2.0
                )
            return False

        self._last_publish_ts = now
        return True

    def _should_log_this_publish(self):
        """Return True if this published sample should be logged (1 of every N)."""
        return (self.publish_count % self.agg_log_every_n) == 0

    def listener_1_myosuite_callback(self, msg):
        # Log the received message to the console
        #self.get_logger().info(f'Myosuite data: "{msg.data}"')

        self.flagMyosuite = True    # Alert that we have received an update from the MyoSuite environment
        #self.get_logger().info('FlagMyosuite to True')
        self.msgMyosuite = msg.data     # Save the array from the message
        self.publish(msg)   # Try to send the observations to the RL node 

    def listener_1_exo_callback(self, msg):
        """Callback for MD80 motor joint states (Mode 1 only).
        
        Receives JointState messages from the real MD80 motor via Candle ROS2 node.
        Filters by motor ID to handle multi-motor systems.
        Calculates acceleration from velocity differences (numerical differentiation).
        
        Args:
            msg (JointState): Contains name[], position[], velocity[], effort[]
        """
        #self.get_logger().info(f'Exo data: "{msg.position}"')
        #self.get_logger().info(f'Exo name: "{msg.name}"')

        # Filter: Only process messages from our target motor ID
        if(msg.name[0] == ('Joint '+self.id)):
            #self.get_logger().info(f'Message from joint id: '+ self.id)
            self.flagMd80Exo = True  # Signal that fresh motor data is available
            #self.get_logger().info('FlagExo to True')
            
            # Store motor state
            self.msgMd80ExoPos = msg.position  # Current position [rad]
            if len(msg.position) > 0:
                self._latest_real_pos = float(msg.position[0])
                self._have_real_joint_state = True
            
            # Calculate acceleration via numerical differentiation:
            # acc = Δvel / Δt, where Δt = 1/frequency
            self.msgMd80ExoAcc = (msg.velocity[0] - self.msgMd80ExoVel[0]) / (1/self.msgMd80ExoHz)
            
            self.msgMd80ExoVel = msg.velocity  # Update velocity for next iteration
            self.msgMd80ExoEffort = msg.effort # Torque feedback
            
            self.publish(msg)  # Attempt fusion and publish to RL

    def publish(self, msg):
        
        if (self.control_type == 0):
            if not self._rate_limit_allows_publish():
                return

            # As the message structure is the same for the MyoSuite environment and the RL node, send this message directly
            msg_pub = Float32MultiArray() # Create a message as a float array
            msg_pub.data = msg.data

            self.publish_count += 1
            self.publisher_.publish(msg_pub)
            if self._should_log_this_publish():
                self.get_logger().info(
                    f'[MODE 0] Publishing WITHOUT fusion [#{self.publish_count}]'
                )

        elif (self.control_type == 1):
            if not (self.flagMyosuite and self.flagMd80Exo):
                # Skip publishing if not both sources are ready
                # self.get_logger().debug(f'Skipping publish: flagMyosuite={self.flagMyosuite}, flagMd80Exo={self.flagMd80Exo}')
                return

            if not self._rate_limit_allows_publish():
                return

            self.publish_count += 1
            log_this_publish = self._should_log_this_publish()
            
            # MODE 1: Data fusion - Replace simulated motor values with real hardware feedback
            # Only publish when BOTH sources have fresh data (synchronized update)
            
            msg_pub = Float32MultiArray()
            observations = list(self.msgMyosuite)  # Copy MyoSuite observations (avoid reference issues)
            obs_len = len(observations)
            
            if log_this_publish:
                self.get_logger().info(
                    f'[FUSION] Before [#{self.publish_count}]: obs[7]={observations[7]:.3f}, MD80 pos={self.msgMd80ExoPos[0]:.3f}'
                )

            raw_real_pos = self.msgMd80ExoPos[0]
            raw_real_vel = self.msgMd80ExoVel[0]
            raw_real_acc = self.msgMd80ExoAcc

            real_pos_aligned = self.exo_sign * raw_real_pos * self.exo_pos_gain
            real_vel_aligned = self.exo_sign * raw_real_vel * self.exo_pos_gain
            real_acc_aligned = self.exo_sign * raw_real_acc * self.exo_pos_gain

            # NOTE: offset is now computed during interactive calibration only.
            # Do NOT auto-calibrate offset from observations[7] as that gives incorrect values
            # if simulator is not properly synchronized.

            real_pos_aligned += self.exo_pos_offset
            if self._clip_to_sim_range:
                real_pos_aligned = float(
                    numpy.clip(real_pos_aligned, self.sim_pos_min, self.sim_pos_max)
                )
            
            # SAFETY: Check array bounds before writing to prevent IndexError
            # The observation vector structure for M1640KneeExoSim2Real-v0 (52 total):
            #   obs[0]:     Time
            #   obs[1-16]:  Joint positions (qpos) - 16 values
            #   obs[17-32]: Joint velocities (qvel) - 16 values
            #   obs[33-48]: Joint accelerations (qacc) - 16 values
            #   obs[49]:    Exoskeleton action/effort (act)
            #   obs[50]:    Pose error
            #   obs[51]:    Interaction force
            
            # Replace index 7: Exoskeleton knee joint POSITION (real hardware)
            if obs_len > 7:
                observations[7] = real_pos_aligned
            else:
                self.get_logger().warn(f'Observations length {obs_len} < 8. Skipping position update.')

            # Replace index 23: Exoskeleton knee joint VELOCITY (real hardware)
            if obs_len > 23:
                observations[23] = real_vel_aligned
            else:
                self.get_logger().warn(f'Observations length {obs_len} < 24. Skipping velocity update.')

            # Replace index 39: Exoskeleton knee joint ACCELERATION (calculated from velocity)
            if obs_len > 39:
                observations[39] = real_acc_aligned  # Note: This is a float, not array
            else:
                self.get_logger().warn(f'Observations length {obs_len} < 40. Skipping acceleration update.')

            # Replace index 49: Exoskeleton ACTION/TORQUE (real hardware)
            # For M1640KneeExoSim2Real-v0 (52 obs): act[0] is at global index 49
            if obs_len > 49:
                observations[49] = self.msgMd80ExoEffort[0]
            else:
                self.get_logger().warn(f'Observations length {obs_len} < 50. Skipping effort update.')
            
            if log_this_publish:
                self.get_logger().info(
                    f'[FUSION] After [#{self.publish_count}]: obs[7]={observations[7]:.3f}, obs[23]={observations[23]:.3f}, '
                    f'obs[39]={observations[39]:.3f} | offset={self.exo_pos_offset:+.3f}, '
                    f'sign={self.exo_sign:+.0f}, gain={self.exo_pos_gain:.3f}'
                )
            
            msg_pub.data = observations # Save the array into the message

            # Log what we're about to publish
            if log_this_publish:
                self.get_logger().info(
                    f'[PUBLISH #{self.publish_count}] pos={observations[7]:.3f}, vel={observations[23]:.3f}'
                )
            
            self.publisher_.publish(msg_pub)    # Publish the observation message to the RL node
            #self.get_logger().info('Publishing RL observation: "%s"' % msg_pub.data)

            # Update the flags to wait for a new set of observations to come
            self.flagMd80Exo = False
            self.flagMyosuite = False
            #self.get_logger().info('Flags to false')
            

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    print('Introduce an int to select the desired environment for the RL: \n 0: MyoSuite simulator only\n 1: MyoSuite simulator and md80 exoskeleton motor')
    control_type = int(input('Select RL environment: '))
    # Create an instance of the MinimalPublisher node
    minimal_pubsub = MinimalPublisher(control_type)

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_pubsub)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        # Destroy the node and shutdown the ROS2 Python client library
        minimal_pubsub.destroy_node()
        rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
