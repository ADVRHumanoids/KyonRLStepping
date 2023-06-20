import numpy as np

from kyonrlstepping.gym.omni_vect_env.vec_envs import RobotVecEnv
from kyon_rhc.kyonrhc import KyonRHCluster, KyonRHC, KyonClusterCmd
from kyon_rhc.interface.control_cluster import RobotClusterState

#from stable_baselines3 import PPO

env = RobotVecEnv(headless=True, 
                enable_livestream=False, 
                enable_viewport=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask

num_envs = 5
task = KyonRlSteppingTask(name="KyonRLStepping", 
                        num_envs = num_envs, 
                        robot_offset = np.array([0.0, 0.0, 2.0])) # create task

device = "cuda"

sim_params = {}
sim_params["integration_dt"] = 1.0/100.0
sim_params["rendering_dt"] = 1.0/60.0
sim_params["substeps"] = 1
sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
sim_params["enable_scene_query_support"] = True
sim_params["replicate_physics"] = True
sim_params["use_flatcache"] = True
sim_params["disable_contact_processing"] = False
sim_params["use_gpu_pipeline"] = True
sim_params["device"] = device

env.set_task(task, 
        backend="torch", 
        sim_params = sim_params) # add the task to the environment 
# (includes spawning robots)

# Run inference on the trained policy
#model = PPO.load("ppo_cartpole")
env._world.reset()
obs = env.reset()
# env._world.pause()

cmd_size = 2
n_jnts = env._task._robot_n_dofs
cluster_cmds = KyonClusterCmd(cluster_size = num_envs, 
                            cmd_size = cmd_size, 
                            device = device)

cluster_state = RobotClusterState(n_dofs = n_jnts, 
                            cluster_size = num_envs, 
                            device = device)

control_cluster = KyonRHCluster(cluster_size = num_envs, 
                            n_dofs = n_jnts,
                            cmd_size = cmd_size, 
                            device = device)
for i in range(0, num_envs):

    result = control_cluster.add_controller(KyonRHC(urdf_path = "", 
                                        config_path = "", 
                                        trigger_pipe = control_cluster.trigger_pipes[i][0],
                                        success_pipe = control_cluster.success_pipes[i][1]))
control_cluster.setup()
control_cluster.update(cluster_state) # we set the initial control cluster state
# using the observations from the first reset call

import torch
import time

rt_time_reset = 100
rt_factor = 1.0
real_time = 0.0
sim_time = 0.0
i = 0
while env._simulation_app.is_running():

    if (i >= rt_time_reset):

        real_time = 0.0
        sim_time = 0.0

    start_time = time.time()
    # action, _states = model.predict(obs)

    # rhc_cmds = rhc_get_cmds_fromjoy() or from agent

    control_cluster.set_commands(cluster_cmds)

    control_cluster.solve()

    # print("Cluster solution time: " + str(control_cluster.solution_time))

    obs, rewards, dones, info = env.step() 
    # control_cluster.update() # open loop update of the internal control cluster
    # control_cluster.update(cluster_state) # closed loop update of the internal control cluster
    
    loop_exec_time = time.time() - start_time

    real_time = real_time + loop_exec_time
    sim_time = sim_time + sim_params["integration_dt"]

    rt_factor = sim_time / real_time
    
    i=+1

    print("\nCurrent RT factor: " + str(rt_factor))
    print("\nreal_time: " + str(real_time))
    print("\nsim_time: " + str(sim_time))

control_cluster.terminate() # closes all processes
env.close()