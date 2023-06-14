import numpy as np

from kyonrlstepping.gym.omni_vect_env.vec_envs import KyonVecEnv

#from stable_baselines3 import PPO

env = KyonVecEnv(headless=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask

task = KyonRlSteppingTask(name="KyonRLStepping", 
                        num_envs = 5, 
                        robot_offset = np.array([0.0, 0.0, 0.8])) # create task

sim_params = {}
sim_params["integration_dt"] = 1.0/60.0
sim_params["rendering_dt"] = 1.0/60.0
sim_params["substeps"] = 1
sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
sim_params["enable_scene_query_support"] = True
sim_params["replicate_physics"] = True
sim_params["use_flatcache"] = True
sim_params["disable_contact_processing"] = False
sim_params["use_gpu_pipeline"] = True
sim_params["device"] = "cuda"

env.set_task(task, 
        backend="torch", 
        sim_params = sim_params) # add the task to the environment 
# (includes spawning robots)

# Run inference on the trained policy
#model = PPO.load("ppo_cartpole")
env._world.reset()
obs = env.reset()
env._world.pause()

while env._simulation_app.is_running():
    # action, _states = model.predict(obs)
    action = None
    obs, rewards, dones, info = env.step(action)

env.close()