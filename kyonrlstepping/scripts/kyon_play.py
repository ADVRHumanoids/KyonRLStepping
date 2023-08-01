import numpy as np

from omni_custom_gym.gym.omni_vect_env.vec_envs import RobotVecEnv

#from stable_baselines3 import PPO

env = RobotVecEnv(headless=False, 
                enable_livestream=False, 
                enable_viewport=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

num_envs = 3 # 9, 3, 5
sim_params = {}
sim_params["use_gpu_pipeline"] = True
sim_params["integration_dt"] = 1.0/100.0
sim_params["rendering_dt"] = 1.0/50.0
sim_params["substeps"] = 1
sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
sim_params["enable_scene_query_support"] = True
sim_params["replicate_physics"] = True
sim_params["use_flatcache"] = True
sim_params["disable_contact_processing"] = False
if sim_params["use_gpu_pipeline"]:
    sim_params["device"] = "cuda"
else:
    sim_params["device"] = "cpu"

device = sim_params["device"]

task = KyonRlSteppingTask(num_envs = num_envs, 
                        cloning_offset = np.array([0.0, 0.0, 2.0]), 
                        device = device) # create task

env.set_task(task, 
        backend="torch", 
        sim_params = sim_params) # add the task to the environment 
# (includes spawning robots)

# Run inference on the trained policy
#model = PPO.load("ppo_cartpole")
env._world.reset()
obs = env.reset()
# env._world.pause()

n_jnts = env._task._robot_n_dofs

control_clust_dt = sim_params["integration_dt"] * 2
cluster_client = KyonRHClusterClient(cluster_size=num_envs, 
                                    device=device, 
                                    cluster_dt=control_clust_dt, 
                                    control_dt=sim_params["integration_dt"])

import time
rt_time_reset = 100
rt_factor = 1.0
real_time = 0.0
sim_time = 0.0
i = 0
start_time = time.time()
start_time_loop = 0

while env._simulation_app.is_running():
    
    start_time_loop = time.time()

    # if (i >= rt_time_reset):

    #     real_time = 0.0
    #     sim_time = 0.0

    # action, _states = model.predict(obs)
    

    # rhc_cmds = rhc_get_cmds_fromjoy() or from agent

    if cluster_client.is_cluster_instant(i):

        cluster_client.solve()

        print("[main][info]: cumulative cluster solution time:-> " + str(cluster_client.solution_time))

    obs, rewards, dones, info = env.step() 
    
    now = time.time()
    real_time = now - start_time
    sim_time += sim_params["integration_dt"]
    rt_factor = sim_time / real_time
    
    i+=1 # updating simulation iteration number

    print("[main][info]: current RT factor-> " + str(rt_factor))
    print("[main][info]: current training RT factor-> " + str(rt_factor * num_envs))
    print("[main][info]: real_time-> " + str(real_time))
    print("[main][info]: sim_time-> " + str(sim_time))
    print("[main][info]: loop execution time-> " + str(now - start_time_loop))

print("[main][info]: closing environment and simulation")
cluster_client.close()
env.close()