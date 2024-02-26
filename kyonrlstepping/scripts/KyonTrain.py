from lrhc_control.envs.heightchange_env import LRhcHeightChange
from lrhc_control.training_algs.ppo import CleanPPO

from SharsorIPCpp.PySharsorIPC import VLevel

from stable_baselines3 import PPO

import os, argparse

from datetime import datetime

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
    env = LRhcHeightChange(namespace=namespace,
                    verbose=True,
                    vlevel=VLevel.V2)

    ppo = CleanPPO(env=env)
    run_name = datetime.now().strftime('%Y%m%d%H%M%S')
    ppo.setup(run_name="MorningTest", verbose=True)
    
    while not ppo.is_done():
        
        ppo.learn()