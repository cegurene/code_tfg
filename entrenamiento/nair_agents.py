import torch
import torch.nn as nn

# === PPO ===
class PPOActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, act_dim)
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, x):
        mean = self.net(x)
        std = torch.exp(self.log_std).expand_as(mean)
        return mean, std

class PPOCritic(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.net(x)
    
# === MPO ===
class MPOActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, act_dim)
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim))  

    def forward(self, x):
        mean = self.net(x)
        std = torch.exp(self.log_std).expand_as(mean)
        return mean, std


class MPOCritic(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, x):
        return self.net(x)

# === TRPO ===
class TRPOActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, act_dim)
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim)) 

    def forward(self, x):
        mean = self.net(x)
        std = torch.exp(self.log_std).expand_as(mean)
        return mean, std


class TRPOCritic(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.net(x)

# === DDPG ===
class DDPGActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 400),
            nn.ReLU(),
            nn.Linear(400, 300),
            nn.ReLU(),
            nn.Linear(300, act_dim),
            nn.Tanh()
        )

    def forward(self, x):
        return self.net(x)

class DDPGCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + act_dim, 400),
            nn.ReLU(),
            nn.Linear(400, 300),
            nn.ReLU(),
            nn.Linear(300, 1)
        )

    def forward(self, x, a):
        return self.net(torch.cat([x, a], dim=-1))

# === SAC ===
class SACActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, act_dim)
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, x):
        mean = self.net(x)
        std = torch.exp(self.log_std)
        return mean, std

class SACCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + act_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, x, a):
        return self.net(torch.cat([x, a], dim=-1))

# === Utilidad para cargar dinámicamente ===
def get_models(agent, obs_dim, act_dim):
    if agent == "PPO":
        return PPOActor(obs_dim, act_dim), PPOCritic(obs_dim)
    elif agent == "TRPO":
        return TRPOActor(obs_dim, act_dim), TRPOCritic(obs_dim, act_dim)
    elif agent == "MPO":
        return MPOActor(obs_dim, act_dim), MPOCritic(obs_dim, act_dim)
    elif agent == "DDPG":
        return DDPGActor(obs_dim, act_dim), DDPGCritic(obs_dim, act_dim)
    elif agent == "SAC":
        return SACActor(obs_dim, act_dim), SACCritic(obs_dim, act_dim)
    else:
        raise ValueError(f"Algoritmo no soportado: {agent}")
