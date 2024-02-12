from lrhc_control.envs.lrhc_training_env import LRhcTrainingEnv
from SharsorIPCpp.PySharsorIPC import VLevel

from stable_baselines3 import PPO

namespace = "kyon0"
env = LRhcTrainingEnv(namespace=namespace,
                verbose=True,
                vlevel=VLevel.V2)

while True:

    env.step(action=None)

model = PPO(
    "MlpPolicy",
    env,
    n_steps=1000,
    batch_size=1000,
    n_epochs=20,
    learning_rate=0.001,
    gamma=0.99,
    device="cuda:0",
    ent_coef=0.0,
    vf_coef=0.5,
    max_grad_norm=1.0,
    verbose=1,
    tensorboard_log="./lrhc_tensorboard",
)
model.learn(total_timesteps=100000)
model.save("lrhc_kyon")

env.close()