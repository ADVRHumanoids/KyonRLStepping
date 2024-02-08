import numpy as np

from omni_robo_gym.envs.isaac_env import IsaacSimEnv

env = IsaacSimEnv(headless=False, 
                enable_livestream=False, 
                enable_viewport=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

num_envs = 3
sim_params = {}
sim_params["use_gpu_pipeline"] = True
sim_params["physics_dt"] = 1.0/100.0
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

control_clust_dt = sim_params["physics_dt"] * 2
cluster_client = KyonRHClusterClient(cluster_size=num_envs, 
                                    device=device, 
                                    cluster_dt=control_clust_dt, 
                                    control_dt=sim_params["physics_dt"])

import time
rt_time_reset = 100
rt_factor = 1.0
real_time = 0.0
sim_time = 0.0
i = 0
start_time = time.perf_counter()
start_time_loop = 0

while env._simulation_app.is_running():
    
    start_time_loop = time.perf_counter()

    # if (i >= rt_time_reset):

    #     real_time = 0.0
    #     sim_time = 0.0

    # action, _states = model.predict(obs)
    

    # rhc_cmds = rhc_get_cmds_fromjoy() or from agent

    if cluster_client.is_cluster_instant(i):

        cluster_client.solve()

        print("[main][info]: cumulative cluster solution time:-> " + str(cluster_client.solution_time))

    obs, rewards, dones, info = env.step() 
    
    now = time.perf_counter()
    real_time = now - start_time
    sim_time += sim_params["physics_dt"]
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


env = VecEnvBase(headless=True)

# create task and register task
from cartpole_task import CartpoleTask

task = CartpoleTask(name="Cartpole")
env.set_task(task, backend="torch")

# create agent from stable baselines
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
    tensorboard_log="./cartpole_tensorboard",
)
model.learn(total_timesteps=100000)
model.save("ppo_cartpole")

env.close()