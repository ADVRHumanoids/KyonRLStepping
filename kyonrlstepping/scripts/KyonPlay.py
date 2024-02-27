import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

import numpy as np
import torch

#from stable_baselines3 import PPO
from kyonrlstepping.envs.kyon_sim_env import KyonLRhcIsaacSimEnv 

from control_cluster_bridge.utilities.shared_data.sim_data import SharedSimInfo

from omni_robo_gym.utils.rt_factor import RtFactor

from SharsorIPCpp.PySharsorIPC import VLevel

num_envs = 24

# simulation parameters
sim_params = {}
# device settings
sim_params["use_gpu_pipeline"] = False # disabling gpu pipeline is necessary to be able
# to retrieve some quantities from the simulator which, otherwise, would have random values
sim_params["use_gpu"] = True # does this actually do anything?
if sim_params["use_gpu_pipeline"]:
    sim_params["device"] = "cuda"
else:
    sim_params["device"] = "cpu"
device = sim_params["device"]

# sim_params["dt"] = 1.0/100.0 # physics_dt?
sim_params["physics_dt"] = 1.0/400.0 # physics_dt?
sim_params["rendering_dt"] = sim_params["physics_dt"]
# sim_params["substeps"] = 1 # number of physics steps to be taken for for each rendering step
sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
sim_params["enable_scene_query_support"] = False
sim_params["use_fabric"] = True # Enable/disable reading of physics buffers directly. Default is True.
sim_params["replicate_physics"] = True
# sim_params["worker_thread_count"] = 4
sim_params["solver_type"] =  1 # 0: PGS, 1:TGS, defaults to TGS. PGS faster but TGS more stable
sim_params["enable_stabilization"] = True
# sim_params["bounce_threshold_velocity"] = 0.2
# sim_params["friction_offset_threshold"] = 0.04
# sim_params["friction_correlation_distance"] = 0.025
# sim_params["enable_sleeping"] = True
# Per-actor settings ( can override in actor_options )
sim_params["solver_position_iteration_count"] = 2 # defaults to 4
sim_params["solver_velocity_iteration_count"] = 1 # defaults to 1
sim_params["sleep_threshold"] = 0.0 # Mass-normalized kinetic energy threshold below which an actor may go to sleep.
# Allowed range [0, max_float).
sim_params["stabilization_threshold"] = 1e-5
# Per-body settings ( can override in actor_options )
# sim_params["enable_gyroscopic_forces"] = True
# sim_params["density"] = 1000 # density to be used for bodies that do not specify mass or density
# sim_params["max_depenetration_velocity"] = 100.0
# sim_params["solver_velocity_iteration_count"] = 1

# GPU buffers settings
# sim_params["gpu_max_rigid_contact_count"] = 512 * 1024
# sim_params["gpu_max_rigid_patch_count"] = 80 * 1024
# sim_params["gpu_found_lost_pairs_capacity"] = 1024
# sim_params["gpu_found_lost_aggregate_pairs_capacity"] = 1024
# sim_params["gpu_total_aggregate_pairs_capacity"] = 1024
# sim_params["gpu_max_soft_body_contacts"] = 1024 * 1024
# sim_params["gpu_max_particle_contacts"] = 1024 * 1024
# sim_params["gpu_heap_capacity"] = 64 * 1024 * 1024
# sim_params["gpu_temp_buffer_capacity"] = 16 * 1024 * 1024
# sim_params["gpu_max_num_partitions"] = 8

integration_dt = sim_params["physics_dt"]
control_clust_dt = 0.03 # [s]

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

headless = True
enable_livestream = False
enable_viewport = False
env_debug = True
    
env = KyonLRhcIsaacSimEnv(headless=headless,
        sim_device = 0,
        enable_livestream=enable_livestream, 
        enable_viewport=enable_viewport,
        debug = env_debug) # create environment

# now we can import the task (not before, since Omni plugins are loaded 
# upon environment initialization)
from kyonrlstepping.tasks.kyon_rlstepping_task import KyonLRHcIsaacTask
                            
task = KyonLRHcIsaacTask(integration_dt = integration_dt,
        num_envs = num_envs, 
        cloning_offset = np.array([[0.0, 0.0, 0.8]] * num_envs), 
        env_spacing=6,
        spawning_radius=1.0, 
        use_flat_ground=True, 
        solver_position_iteration_count = sim_params["solver_position_iteration_count"], # applies this to all articulations
        solver_velocity_iteration_count = sim_params["solver_velocity_iteration_count"],
        solver_stabilization_thresh = sim_params["sleep_threshold"],
        default_jnt_stiffness=200.0, 
        default_jnt_damping=50.0, 
        default_wheel_stiffness = 0.0,
        default_wheel_damping=10.0,
        startup_jnt_stiffness = 100,
        startup_jnt_damping = 20,
        startup_wheel_stiffness = 0.0,
        startup_wheel_damping=10.0,
        contact_prims = contact_prims,
        contact_offsets = contact_offsets,
        sensor_radii = sensor_radii,
        override_art_controller=True, # uses handmade EXPLICIT controller. This will usually be unstable for relatively high int. dts
        device = device, 
        use_diff_velocities = False, # whether to differentiate velocities numerically
        dtype=dtype_torch,
        debug_mode_jnt_imp = True) # writes jnt imp. controller info on shared mem (overhead)
        # and profiles it

env.set_task(task, 
        cluster_dt = [control_clust_dt],
        backend="torch", 
        is_training = [True],
        n_pre_training_steps = 10, # n of env steps before connecting to training client
        sim_params = sim_params, 
        cluster_client_verbose=True, 
        cluster_client_debug=True) # add the task to the environment 
# (includes spawning robots and launching the cluster client for the controllers)

# Run inference on the trained policy
#model = PPO.load("ppo_cartpole")
obs = env.reset(reset_world=True)

# sim info to be broadcasted 
# adding some data to dict for debugging
sim_params["n_envs"] = num_envs 
sim_params["control_clust_dt"] = control_clust_dt
sim_params["headless"] = headless
sim_params["enable_livestream"] = enable_livestream
sim_params["enable_viewport"] = enable_viewport
sim_params["env_debug"] = env_debug

shared_sim_infos = []
for i in range(len(robot_names)):
    shared_sim_infos.append(SharedSimInfo(
                            namespace=robot_names[i],
                            is_server=True, 
                            sim_params_dict=sim_params,
                            verbose=True,
                            vlevel=VLevel.V2,
                            force_reconnection=True) )

    shared_sim_infos[i].run()

rt_factor = RtFactor(dt_nom=sim_params["physics_dt"],
            window_size=50000)

while env._simulation_app.is_running():
    
    try:

        if rt_factor.reset_due():

            rt_factor.reset()
            
        env.step() 
        
        rt_factor.update()

        for i in range(len(robot_names)):

            shared_sim_infos[i].write(dyn_info_name=["sim_rt_factor", 
                                                "total_rt_factor", 
                                                "env_stepping_dt",
                                                "world_stepping_dt",
                                                "time_to_get_states_from_sim",
                                                "cluster_state_update_dt",
                                                "cluster_sol_time"
                                                ],
                                val=[rt_factor.get(), 
                                    rt_factor.get() * num_envs,
                                    rt_factor.get_avrg_step_time(),
                                    env.debug_data["time_to_step_world"],
                                    env.debug_data["time_to_get_states_from_sim"],
                                    env.debug_data["cluster_state_update_dt"][robot_names[i]],
                                    env.debug_data["cluster_sol_time"][robot_names[i]]])

    except Exception as e:
        
        print(f"An exception occurred: {e}")
        
        for i in range(len(robot_names)):

            shared_sim_infos[i].close()

        env.close()

        break

