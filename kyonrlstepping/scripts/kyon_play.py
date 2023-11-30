import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

import numpy as np
import torch

#from stable_baselines3 import PPO
from kyonrlstepping.envs.kyonenv import KyonEnv 

env = KyonEnv(headless=False, 
        enable_livestream=False, 
        enable_viewport=False) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonRlSteppingTask

from omni_robo_gym.utils.shared_sim_info import SharedSimInfo

print_sim_info = False

num_envs = 3 # 9, 3, 5
sim_params = {}
sim_params["use_gpu_pipeline"] = False
sim_params["integration_dt"] = 1.0/100.0
sim_params["rendering_dt"] = sim_params["integration_dt"]
sim_params["substeps"] = 1
sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
sim_params["enable_scene_query_support"] = False
sim_params["use_fabric"] = True # Enable/disable reading of physics buffers directly. Default is True.
sim_params["replicate_physics"] = True
sim_params["enable_stabilization"] = True
sim_params["disable_contact_processing"] = True
if sim_params["use_gpu_pipeline"]:
    sim_params["device"] = "cuda"
else:
    sim_params["device"] = "cpu"

device = sim_params["device"]

control_clust_dt = sim_params["integration_dt"] * 2
integration_dt = sim_params["integration_dt"]

dtype = "float32" # Isaac requires data to be float32, so this should not be touched
if dtype == "float64":
    dtype_np = np.float64 
    dtype_torch = torch.float64
if dtype == "float32":
    dtype_np = np.float32
    dtype_torch = torch.float32
# this has to be the same wrt the cluster server, otherwise
# messages are not read/written properly

# create task
robot_names = ["kyon0"] # robot names
robot_pkg_names = ["kyon"] # robot type

contact_prims = {} # contact sensors to be added
contact_prims["kyon0"] = ["wheel_1", "wheel_2", "wheel_3", "wheel_4"] # foot contact sensors

contact_offsets = {}
contact_offsets["kyon0"] = {}
for i in range(0, len(contact_prims["kyon0"])):
    
    contact_offsets["kyon0"][contact_prims["kyon0"][i]] = \
        np.array([0.0, 0.0, 0.0])
    
sensor_radii = {}
sensor_radii["kyon0"] = {}
for i in range(0, len(contact_prims["kyon0"])):
    
    sensor_radii["kyon0"][contact_prims["kyon0"][i]] = 0.124
                            
task = KyonRlSteppingTask(cluster_dt = control_clust_dt, 
                    integration_dt = integration_dt,
                    num_envs = num_envs, 
                    cloning_offset = np.array([[0.0, 0.0, 1.3]] * num_envs), 
                    env_spacing=6,
                    spawning_radius=1.0, 
                    use_flat_ground=True, 
                    pos_iter_increase_factor = 1, # 1 means to default
                    vel_iter_increase_factor = 1,
                    default_jnt_stiffness=200.0, 
                    default_jnt_damping=50.0, 
                    default_wheel_stiffness = 0.0,
                    default_wheel_damping=30.0,
                    startup_jnt_stiffness = 100,
                    startup_jnt_damping = 10,
                    startup_wheel_stiffness = 0.0,
                    startup_wheel_damping=100.0,
                    robot_names = robot_names,
                    robot_pkg_names = robot_pkg_names,
                    contact_prims = contact_prims,
                    contact_offsets = contact_offsets,
                    sensor_radii = sensor_radii,
                    override_art_controller=False,
                    device = device, 
                    use_diff_velocities = True,
                    dtype=dtype_torch) # create task

env.set_task(task, 
        backend="torch", 
        sim_params = sim_params, 
        np_array_dtype = dtype_np, 
        verbose=True, 
        debug=True) # add the task to the environment 
# (includes spawning robots and launching the cluster client for the controllers)

# Run inference on the trained policy
#model = PPO.load("ppo_cartpole")
# env._world.reset()
obs = env.reset()
# env._world.pause()

import time
rt_time_reset = 100
rt_factor = 1.0
real_time = 0.0
sim_time = 0.0
i = 0
start_time = time.perf_counter()
start_time_loop = 0
rt_factor_reset_n = 100 
rt_factor_counter = 0

shared_sim_info = SharedSimInfo() # sim. info to be broadcasted
shared_sim_info.start(gpu_pipeline_active=sim_params["use_gpu_pipeline"], 
                    integration_dt=integration_dt,
                    rendering_dt=sim_params["rendering_dt"], 
                    cluster_dt=control_clust_dt)

while env._simulation_app.is_running():
    
    if ((i + 1) % rt_factor_reset_n) == 0:

        rt_factor_counter = 0

        start_time = time.perf_counter()

        sim_time = 0

    start_time_step = time.perf_counter()

    obs, rewards, dones, info = env.step(index=i) 
    
    now = time.perf_counter()

    real_time = now - start_time
    sim_time += sim_params["integration_dt"]
    rt_factor = sim_time / real_time
    
    shared_sim_info.update(sim_rt_factor=rt_factor, 
                        cumulative_rt_factor=rt_factor * num_envs, 
                        time_for_sim_stepping=now - start_time_step)
    
    i+=1 # updating simulation iteration number
    rt_factor_counter = rt_factor_counter + 1

    if print_sim_info:
        
        print(f"[{script_name}]" + "[info]: current RT factor-> " + str(rt_factor))
        print(f"[{script_name}]" + "[info]: current training RT factor-> " + str(rt_factor * num_envs))
        print(f"[{script_name}]" + "[info]: real_time-> " + str(real_time))
        print(f"[{script_name}]" + "[info]: sim_time-> " + str(sim_time))
        print(f"[{script_name}]" + "[info]: time to step full env.-> " + str(now - start_time_step))

    # contact_report = task.omni_contact_sensors["kyon0"].contact_sensors[0][0].get_current_frame() 

    # print("#########")
    # print(contact_report)

    # print(task.omni_contact_sensors["kyon0"].contact_geom_prim_views[0].get_net_contact_forces(clone = False, 
                                                                                            # dt = sim_params["integration_dt"]))
    
    # print("Detailed:")
    # print(task.omni_contact_sensors["kyon0"].contact_geom_prim_views[0].get_contact_force_data(clone = False,
    #                                                                                         dt = sim_params["integration_dt"]))
    # # print("Normal:")
    # # print(contact_report['contacts'])
    # # print(contact_report['normal'].device)
    # print("In contact:")
    # print(contact_report['in_contact'])
    # print("Force:")
    # print(contact_report['force'])
    # print("Number of contacts:")
    # print(contact_report['number_of_contacts'])


print("[main][info]: closing environment and simulation")

shared_sim_info.terminate()

env.close()