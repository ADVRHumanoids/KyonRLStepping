from lrhc_control.envs.lrhc_training_env import LRhcTrainingVecEnv
from SharsorIPCpp.PySharsorIPC import VLevel

from stable_baselines3 import PPO

import os, argparse

# Function to set CPU affinity
def set_affinity(cores):
    try:
        os.sched_setaffinity(0, cores)
        print(f"Set CPU affinity to cores: {cores}")
    except Exception as e:
        print(f"Error setting CPU affinity: {e}")

if __name__ == "__main__":  

    # Parse command line arguments for CPU affinity
    parser = argparse.ArgumentParser(description="Set CPU affinity for the script.")
    parser.add_argument('--cores', nargs='+', type=int, help='List of CPU cores to set affinity to')
    
    args = parser.parse_args()
    
    # Set CPU affinity if cores are provided
    if args.cores:
        set_affinity(args.cores)
        
    namespace = "kyon0"
    env = LRhcTrainingVecEnv(namespace=namespace,
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