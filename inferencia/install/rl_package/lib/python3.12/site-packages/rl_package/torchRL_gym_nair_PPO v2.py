"""
    Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), Jorge Gómez, David Rodriguez, Pablo Lanillos 
    source	:: https://github.com/AndresChS/NAIR_Code
"""

import sys
import os
import shutil
from myosuite.utils import gym
import hydra
import time
from datetime import datetime
import torch
import torch.nn as nn
import torch.optim as optim
from torchrl.envs import GymWrapper, TransformedEnv
from torchrl.collectors import SyncDataCollector
from torchrl.data.replay_buffers import LazyTensorStorage, ReplayBuffer
from torchrl.objectives import ClipPPOLoss
from torchrl.objectives.value import GAE
from torchrl.modules.distributions.continuous import TanhNormal
from torchrl.modules import ProbabilisticActor, ValueOperator
from tensordict.nn import TensorDictModule, TensorDictSequential
from omegaconf import DictConfig
from torchrl.data.replay_buffers.samplers import SamplerWithoutReplacement

import warnings
warnings.filterwarnings("ignore")
from torch import multiprocessing


from collections import defaultdict

import matplotlib.pyplot as plt
import torch
from tensordict.nn import TensorDictModule
from tensordict.nn.distributions import NormalParamExtractor
from torch import nn
from torchrl.collectors import SyncDataCollector
from torchrl.data.replay_buffers import ReplayBuffer
from torchrl.data.replay_buffers.samplers import SamplerWithoutReplacement
from torchrl.data.replay_buffers.storages import LazyTensorStorage
from torchrl.envs import Compose, DoubleToFloat, ObservationNorm, StepCounter, TransformedEnv
from torchrl.envs.utils import check_env_specs, ExplorationType, set_exploration_type
from torchrl.modules import ProbabilisticActor, TanhNormal, ValueOperator
from torchrl.objectives import ClipPPOLoss
from torchrl.objectives.value import GAE
from tqdm import tqdm

# Importing ROS packages
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# Note: the Float32MultiArray may not work on ROS2 Humble, as ROS2 developers don't like the use of generic messages types and is supposed to be depreciated in Foxy.
from std_msgs.msg import Float32MultiArray
from rl_interfaces.msg import MotionCommand
from sensor_msgs.msg import JointState


# Add scone gym
sconegym_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../NAIR_envs')) #CHANGE
#sys.path.append(sconegym_path) #CHANGE
sys.path.append('/home/nair-group/jorge_gomez/ros2_rl_ws/src/rl_package/rl_package')
#import sconegym # type: ignore #CHANGE

# Import algorithms models from nair PPO, SAC, DDPG
from nair_agents import get_models
init_dir = os.getcwd()
init_dir = '/home/nair-group/jorge_gomez/ros2_rl_ws/src/rl_package/rl_package'

# =====================================================================
class RLNode(Node):
    def __init__(self):
        # We create the publisher for the RL actions and the subscriber for the RL observations
        super().__init__('rl_training_node')
        # The actions are the torque for the motors, published in the structure accepted by the MD80 motor.
        self.publisher_ = self.create_publisher(MotionCommand, '/md80/motion_command', 10)
        self.motor_id = int(input("Enter the motor ID to control: "))  # Get motor ID from user input

            # We create a subscriber to listen to the MD80 motor
        self.subscription = self.create_subscription(
            JointState,                         # Message type used by the publisher
            '/md80/joint_states',               # topic name
            self.listener_callback,             # Method called each time a message is received
            10)                                 # Queue size for incoming messages
        self.subscription  # Prevent unused variable warning
        self.msgMd80ExoPos = [0]
        self.msgMd80ExoVel = [0]
        self.msgMd80ExoAcc = [0]
        self.msgMd80ExoEffort = [0]

    def listener_callback(self, msg):
                # Log the received message to the console
        # JointState has a string[] name, double[] position, double[] velocity and double[] effort
        self.get_logger().info(f'Exo data: "{msg.position}"')
        self.get_logger().info(f'Exo name: "{msg.name}"')

        if(msg.name[0] == ('Joint '+self.id)):
            self.get_logger().info(f'Message from joint id: '+ self.id)
            self.msgMd80ExoPos = msg.position  # Save the position value (float[]) from the message
            self.msgMd80ExoAcc = (msg.velocity[0] - self.msgMd80ExoVel[0])/(1/self.msgMd80ExoHz)
            self.msgMd80ExoVel = msg.velocity
            self.msgMd80ExoEffort = msg.effort

rclpy.init()
rl_node = RLNode()
# =====================================================================
def evaluate_agent(policy, env, num_episodes=5, device="cpu"):
    policy.eval()
    total_reward = 0.0
    for _ in range(num_episodes):
        obs = env.reset()
        done = False
        ep_reward = 0.0
        while not done:
            with torch.no_grad():
                action = policy(obs.to(device)).sample()

                print(action, type(action), getattr(action, 'shape', None))

            motor_obs = [rl_node.msgMd80ExoPos[0], rl_node.msgMd80ExoVel[0], rl_node.msgMd80ExoAcc[0], rl_node.msgMd80ExoEffort[0]] #May cause problems

            obs, reward, done, _ = env.step(action.cpu(), motor_obs=motor_obs)
            ep_reward += reward
        total_reward += ep_reward
    policy.train()
    return total_reward / num_episodes

@hydra.main(config_path='/home/nair-group/jorge_gomez/ros2_rl_ws/src/rl_package/rl_package', config_name="config_PPO", version_base="1.1")
def main(cfg_hydra: DictConfig):
    start_time = time.time()
    today = datetime.now().strftime('%Y-%m-%d')
    time_now = datetime.now().strftime('%H-%M-%S')
    #shutil.copy(os.path.dirname(__file__) + "../../../../NAIR_envs/sconegym/nair_gaitgym.py", ".")
    shutil.copy(init_dir+"/config_PPO.yaml", ".")
    shutil.copy(init_dir+"/torchRL_gym_nair_PPO.py", ".")
    output_dir = os.path.join(os.getcwd(), f"outputs/checkpoints")
    os.makedirs(output_dir, exist_ok=True)

    print("Launching RL node")

    rl_node = RLNode()
    executor = SingleThreadedExecutor()
    executor.add_node(rl_node)

    executor.spin_once(timeout_sec=1.0)  # Run the node for a short time to ensure it initializes properly

    #rclpy.spin(rl_node) # Llamada bloqueante, es necesario?

    print("RL node launched")

    #env_id = cfg_hydra.env.env_name
    #env_id = 'ExoLegSpasticityFlexoExtEnv-v0'
    #num_cpu = cfg_hydra.env.num_cpu
    #delays = cfg_hydra.env.use_delayed_sensors

    try:
        #env = gym.vector.make(env_id, use_delayed_sensors=delays, num_envs=num_cpu, asynchronous=False)
        env = gym.make('M1640KneeExoSim2Real-v0')
        print("observation space env: ", env.observation_space)
        print("action space env: ", env.action_space)
    except gym.error.DeprecatedEnv as e:
        env_id = [spec.id for spec in gym.envs.registry.all() if spec.id.startswith("nair")][0]
        print("sconewalk_h0918_osim-v1 not found. Trying {}".format(env_id))

    obs = env.reset()
    print(obs, type(obs), getattr(obs, 'shape', None))
    print(type(env.reset()))

    base_env = GymWrapper(env)
    env = TransformedEnv(base_env)
    #env.append_transform(ObservationNorm(in_keys=["observation"]))
    
    obs_dim = env.observation_spec["observation"].shape[-1]
    act_dim = env.action_spec.shape[-1]
    agent = "PPO"  # o "PPO", "DDPG"

    actor, critic = get_models(agent, obs_dim, act_dim)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    env = env.to(device)
    actor.to(device)
    critic.to(device)
    print("obs_dim:", obs_dim, "act_dim:", act_dim)
    print("observation_spec:", env.observation_spec)
    print("action_spec:", env.action_spec)

    policy = TensorDictModule(
        actor, in_keys=["observation"], out_keys=["loc", "scale"]
    )
    policy = ProbabilisticActor(
        module=policy,
        spec=env.action_spec,
        in_keys=["loc", "scale"],
        distribution_class=TanhNormal,
        distribution_kwargs={
            "low": env.action_spec.space.low,
            "high": env.action_spec.space.high,
        },
        return_log_prob=True,
        # we'll need the log-prob for the numerator of the importance weights
    )
    value = nn.Sequential(
        nn.LazyLinear(256, device=device),
        nn.Tanh(),
        nn.LazyLinear(256, device=device),
        nn.Tanh(),
        nn.LazyLinear(256, device=device),
        nn.Tanh(),
        nn.LazyLinear(1, device=device),
    )

    value = ValueOperator(
        module=value,
        in_keys=["observation"],
    )
    print("action_spec:", env.action_spec)
    print("Running policy:", policy(env.reset()))
    print("Running value:", value(env.reset()))

    collector = SyncDataCollector(
        env,
        policy,
        frames_per_batch=cfg_hydra.collector.frames_per_batch,
        total_frames=cfg_hydra.collector.total_frames,
        split_trajs=False,
        device=device,
    )
    replay_buffer = ReplayBuffer(
        storage=LazyTensorStorage(max_size=cfg_hydra.collector.frames_per_batch),
        sampler=SamplerWithoutReplacement(),
    )

    advantage_module = GAE(
        gamma=cfg_hydra.hiperparameters.gamma,    
        lmbda=cfg_hydra.hiperparameters.lmbda, 
        value_network=value, 
        average_gae=True, 
        device=device,
    )

    loss_module = ClipPPOLoss(
        actor_network=policy,
        critic_network=value,
        clip_epsilon=cfg_hydra.hiperparameters.clip_epsilon,
        entropy_bonus=bool(cfg_hydra.hiperparameters.entropy_coef),
        entropy_coef=cfg_hydra.hiperparameters.entropy_coef,
        # these keys match by default but we set this for completeness
        critic_coef=cfg_hydra.hiperparameters.critic_coef,
        loss_critic_type="smooth_l1",
    )

    optim = torch.optim.Adam(loss_module.parameters(), cfg_hydra.optim.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optim, cfg_hydra.collector.total_frames // cfg_hydra.collector.frames_per_batch, 0.0
    )

    logs = defaultdict(list)
    pbar = tqdm(total=cfg_hydra.collector.total_frames)
    eval_str = ""
    best_reward = -float("inf")  # Para guardar el mejor agente
    best_checkpoint_path = None
    global_step = 0  # Contador de steps
    for i, tensordict_data in enumerate(collector):
        batch_size = tensordict_data.numel()
        global_step += batch_size
        pbar.update(batch_size)

        for _ in range(cfg_hydra.collector.learning_epochs):
            advantage_module(tensordict_data)
            data_view = tensordict_data.reshape(-1)
            replay_buffer.extend(data_view.cpu())

            for _ in range(cfg_hydra.collector.frames_per_batch // cfg_hydra.hiperparameters.mini_batch_size):
                subdata = replay_buffer.sample(cfg_hydra.hiperparameters.mini_batch_size).to(device)

                if subdata.batch_size != torch.Size([]):
                    subdata.batch_size = []

                loss_vals = loss_module(subdata)
                loss_value = (
                    loss_vals.get("loss_objective", 0.0)
                    + loss_vals.get("loss_critic", 0.0)
                    + loss_vals.get("loss_entropy", 0.0)
                )
                loss_value.backward()
                torch.nn.utils.clip_grad_norm_(loss_module.parameters(), cfg_hydra.hiperparameters.max_grad_norm)
                optim.step()
                optim.zero_grad()

        # Guardar checkpoint cada 10,000 pasos
        if global_step % 10_000 < batch_size:
            checkpoint_path = os.path.join(output_dir, f"checkpoint_{global_step}.pt")
            torch.save({
                "step": global_step,
                "actor": policy.state_dict(),
                "critic": value.state_dict() if 'critic_network' in locals() else None,
                "optimizer": optim.state_dict(),
            }, checkpoint_path)
            """
            current_reward = evaluate_agent(policy, env)  # <- necesitas implementar esta función

            # Si la recompensa mejora, guarda como mejor agente
            if current_reward > best_reward:
                best_reward = current_reward
                best_checkpoint_path = os.path.join(output_dir, "best_agent.pt")
                torch.save({
                    "step": global_step,
                    "actor": policy.state_dict(),
                    "critic": value.state_dict() if 'critic_network' in locals() else None,
                    "optimizer": optim.state_dict(),
                    "best_reward": best_reward,
                }, best_checkpoint_path)
                print(f"Saving new best agent con recompensa {best_reward:.2f}")
            """
if __name__ == "__main__":
    main()

