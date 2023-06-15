import numpy as np

from kyonrlstepping.gym.omni_vect_env.vec_envs import RobotVecEnv
from kyon_rhc.kyonrhc import KyonRHCluster, KyonRHC, KyonClusterCmd
from kyon_rhc.interface.control_cluster import RobotClusterState

#from stable_baselines3 import PPO

env = RobotVecEnv(headless=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask

num_envs = 5
task = KyonRlSteppingTask(name="KyonRLStepping", 
                        num_envs = num_envs, 
                        robot_offset = np.array([0.0, 0.0, 0.8])) # create task

device = "cuda"

sim_params = {}
sim_params["integration_dt"] = 1.0/200.0
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
env._world.pause()

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

control_cluster.update(cluster_state)

while env._simulation_app.is_running():
    # action, _states = model.predict(obs)

    # rhc_cmds = rhc_get_cmds_fromjoy() or from agent

    control_cluster.set_commands(cluster_cmds)
    control_cluster.solve()

    obs, rewards, dones, info = env.step(control_cluster.get()) 

    control_cluster.update() # open loop update of the internal control cluster
    # control_cluster.update(cluster_state) # closed loop update of the internal control cluster


env.close()