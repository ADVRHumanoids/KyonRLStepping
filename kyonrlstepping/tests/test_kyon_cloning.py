import os
from omni.isaac.kit import SimulationApp 

headless = False 
simulation_app = SimulationApp( {"headless": headless} )

import omni.kit.commands 
from omni.kit.viewport.utility.camera_state import ViewportCameraState

from abc import abstractmethod
# from omni.isaac.core.tasks import BaseTask
# from omni.isaac.core.prims import RigidPrimView, RigidPrim, XFormPrim
from omni.isaac.core import World
from omni.isaac.core.utils.prims import move_prim
# from omni.isaac.core.utils.nucleus import find_nucleus_server
# from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
# from omni.isaac.core.materials import PreviewSurface
# from omni.isaac.cloner import GridCloner
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.rotations import euler_angles_to_quat 
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView

from omni.isaac.cloner import GridCloner
import omni.isaac.core.utils.prims as prim_utils

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.urdf._urdf import UrdfJointTargetType

import numpy as np
import torch 
cuda0 = torch.device('cuda:0')

from omni.isaac.core.controllers import ArticulationController

from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools
from omni.isaac.urdf import _urdf

import os
script_path = os.path.abspath(__file__)
package_root_dir = os.path.dirname(os.path.dirname(script_path))

# we generate the URDF where the Kyon description package is located
import rospkg
rospackage = rospkg.RosPack()
kyon_descr_path = rospackage.get_path("kyon_urdf")
kyon_urdf_path = kyon_descr_path + "/urdf"
kyon_xacro_name = "kyon"
kyon_xacro_path = kyon_urdf_path + "/" + kyon_xacro_name + ".urdf.xacro"
kyon_urdf_path = kyon_urdf_path + "/" + kyon_xacro_name + ".urdf"
import subprocess
add_wheels = "wheels:=" + "true"
add_arms = "upper_body:=" + "false"
remove_floating_joint = "floating_joint:=" + "false"
remove_sensors = "sensors:=" + "false"
try:

    xacro_gen = subprocess.check_call(["xacro",\
                                    kyon_xacro_path, \
                                    add_wheels, \
                                    add_arms, \
                                    remove_floating_joint, \
                                    remove_sensors, \
                                    "-o", 
                                    kyon_urdf_path])

except:

    raise Exception('\nFAILED TO GENERATE KYON\'S URDF!!!.\n')
        
if __name__ == "__main__":
    
    num_envs = 11
    env_spacing = 5
    env_ns = "/World/envs"
    template_env_ns = env_ns + "/env_0"

    cloner = GridCloner(spacing=env_spacing)
    cloner.define_base_env(env_ns)

    prim_utils.define_prim(template_env_ns)

    envs_prim_paths = cloner.generate_paths(env_ns + "/env", num_envs)

    rendering_freq = 60.0 # [Hz]
    integration_freq = 60.0 # [Hz]

    device = "cuda" # either "cpu" or "cuda"

    sim_params = {}
    sim_params["dt"] = 1.0/integration_freq
    sim_params["substeps"] = 1
    sim_params["gravity"] = np.array([0.0, 0.0, -9.81])
    sim_params["enable_scene_query_support"] = True
    sim_params["replicate_physics"] = True
    sim_params["use_flatcache"] = True
    sim_params["disable_contact_processing"] = False
    sim_params["use_gpu_pipeline"] = True
    sim_params["device"] = "cuda:0"

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0/integration_freq, 
        rendering_dt=1.0/rendering_freq,
        backend="torch", # "torch" or "numpy"
        device=device,
        physics_prim_path="/physicsScene", 
        sim_params=sim_params
    )
    
    # Physics context settings
    physics_context = world.get_physics_context()
    
    # GPU buffers
    gpu_max_rigid_contact_count = physics_context.get_gpu_max_rigid_contact_count()
    gpu_max_rigid_patch_count = physics_context.get_gpu_max_rigid_patch_count()
    gpu_found_lost_pairs_capacity = physics_context.get_gpu_found_lost_pairs_capacity()
    gpu_found_lost_aggregate_pairs_capacity = physics_context.get_gpu_found_lost_aggregate_pairs_capacity()
    gpu_total_aggregate_pairs_capacity = physics_context.get_gpu_total_aggregate_pairs_capacity()
    gpu_max_soft_body_contacts = physics_context.get_gpu_max_soft_body_contacts()
    gpu_max_particle_contacts = physics_context.get_gpu_max_particle_contacts()
    gpu_heap_capacity = physics_context.get_gpu_heap_capacity()
    gpu_temp_buffer_capacity = physics_context.get_gpu_temp_buffer_capacity()
    # gpu_max_num_partitions = physics_context.get_gpu_max_num_partitions()

    print("PHYSICS CONTEXT INFO: ")
    print("gpu_max_rigid_contact_count: " + str(gpu_max_rigid_contact_count))
    print("gpu_max_rigid_patch_count: " + str(gpu_max_rigid_patch_count))
    print("gpu_found_lost_pairs_capacity: " + str(gpu_found_lost_pairs_capacity))
    print("gpu_found_lost_aggregate_pairs_capacity: " + str(gpu_found_lost_aggregate_pairs_capacity))
    print("gpu_total_aggregate_pairs_capacity: " + str(gpu_total_aggregate_pairs_capacity))
    print("gpu_max_soft_body_contacts: " + str(gpu_max_soft_body_contacts))
    print("gpu_max_particle_contacts: " + str(gpu_max_particle_contacts))
    print("gpu_heap_capacity: " + str(gpu_heap_capacity))
    print("gpu_temp_buffer_capacity: " + str(gpu_temp_buffer_capacity))
    # print(gpu_max_num_partitions)

    physics_context.enable_gpu_dynamics(True)
    physics_context.enable_stablization(True)

    gpu_max_rigid_contact_count_setting = 1000000
    gpu_max_rigid_patch_count_setting = gpu_max_rigid_patch_count
    gpu_found_lost_pairs_capacity_setting = gpu_found_lost_pairs_capacity
    gpu_found_lost_aggregate_pairs_capacity_setting = gpu_found_lost_aggregate_pairs_capacity
    gpu_total_aggregate_pairs_capacity_setting = gpu_total_aggregate_pairs_capacity
    gpu_max_soft_body_contacts_setting = gpu_max_soft_body_contacts
    gpu_max_particle_contacts_setting = gpu_max_particle_contacts
    gpu_heap_capacity_setting = gpu_heap_capacity
    gpu_temp_buffer_capacity_setting = gpu_temp_buffer_capacity
    gpu_max_num_partitions = 8

    physics_context.set_gpu_max_rigid_contact_count(gpu_max_rigid_contact_count_setting)
    physics_context.set_gpu_max_rigid_patch_count(gpu_max_rigid_patch_count_setting)
    physics_context.set_gpu_found_lost_pairs_capacity(gpu_found_lost_pairs_capacity_setting)
    physics_context.set_gpu_found_lost_aggregate_pairs_capacity(gpu_found_lost_aggregate_pairs_capacity_setting)
    physics_context.set_gpu_total_aggregate_pairs_capacity(gpu_total_aggregate_pairs_capacity_setting)
    physics_context.set_gpu_max_soft_body_contacts(gpu_max_soft_body_contacts_setting)
    physics_context.set_gpu_max_particle_contacts(gpu_max_particle_contacts_setting)
    physics_context.set_gpu_heap_capacity(gpu_heap_capacity_setting)
    physics_context.set_gpu_temp_buffer_capacity(gpu_temp_buffer_capacity_setting)
    physics_context.set_gpu_max_num_partitions(gpu_max_num_partitions)

    physics_scene_prim = physics_context.get_current_physics_scene_prim()
    solver_type = physics_context.get_solver_type()
    # solver = "TGS" # TGS (default) or PGS (more accurate?)
    # physics_context.set_solver_type(solver)

    # get world stage
    scene = world.scene
    # get stage handle
    stage = omni.usd.get_context().get_stage()
        
    # add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

    # a ground plane common to all envs
    ground_plane_prim_path = "/World/ground_plane"
    ground_plane = scene.add_default_ground_plane(z_position=0, name="ground_plane", 
                            prim_path= ground_plane_prim_path, 
                            static_friction=0.5, 
                            dynamic_friction=0.5, 
                            restitution=0.8)
    ground_plane_path = ground_plane.prim_path

    # we load the urdf
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()

    import_config.merge_fixed_joints = True
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = True
    import_config.create_physics_scene = True
    import_config.default_drive_strength = 10.0 # strange behavior: not parsed, unless == 0
    import_config.default_position_drive_damping = 1.0 # strange behavior: not parsed, unless == 0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION # JOINT_DRIVE_POSITION, JOINT_DRIVE_VELOCITY, JOINT_DRIVE_NONE
    import_config.distance_scale = 1
    # import_config.density = 0.0

    # import URDF
    success, kyon_prim_path_default = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=kyon_urdf_path,
        import_config=import_config, 
    )
    kyon_prim_path = template_env_ns + "/Kyon"
    move_prim(kyon_prim_path_default, kyon_prim_path)

    pos_offsets = np.zeros((num_envs, 3))
    for i in range(0, num_envs):
        pos_offsets[i, :] = np.array([0, 0, 1.5])
    envs_positions = cloner.clone(
        source_prim_path=template_env_ns,
        prim_paths=envs_prim_paths,
        replicate_physics=sim_params["replicate_physics"],
        position_offsets = pos_offsets
    ) # kyon is now at the default env prim --> we can clone the environment

    kyons = ArticulationView(env_ns + "/env*"+ "/Kyon", reset_xform_properties=False)

    physics_scene_path = world.get_physics_context().prim_path
    cloner.filter_collisions(
        physics_scene_path, 
        "/World/collisions", 
        prim_paths=envs_prim_paths, 
        global_paths=[ground_plane_prim_path] # can collide with these prims
    )

    scene.add(kyons)
    n_prims = kyons.count
    enable_self_collisions = True
    kyons.set_enabled_self_collisions(np.array([enable_self_collisions] * n_prims))

    hip_roll_default_config = torch.tensor([0.3, -0.3, 0.3, -0.3])
    hip_pitch_default_config = torch.tensor([-0.3, -0.3, 0.3, 0.3])
    knee_pitch_default_config = torch.tensor([0.3, 0.3, -0.3, -0.3])
    wheels_default_config = torch.tensor([0.0, 0.0, 0.0, 0.0])
    kyon_default_joint_positions = torch.cat((hip_roll_default_config, 
                                            hip_pitch_default_config, 
                                            knee_pitch_default_config, 
                                            wheels_default_config),
                                            0)

    world.reset(soft=False) 

    kyons_default_joint_positions = torch.zeros((num_envs, kyons.num_dof))
    for i in range(0,  num_envs):
        kyons_default_joint_positions[i, :] = kyon_default_joint_positions

    hip_roll_default_vel = torch.tensor([0.0, 0.0, 0.0, 0.0])
    hip_pitch_default_vel = torch.tensor([0.0, 0.0, 0.0, 0.0])
    knee_pitch_default_vel = torch.tensor([0.0, 0.0, 0.0, 0.0])
    # wheels_default_vel =  torch.tensor([0.5, -0.5, 0.5, -0.5])
    wheels_default_vel =  torch.tensor([0.0, -0.0, 0.0, -0.0])
    kyon_default_joint_velocities = torch.cat((hip_roll_default_vel, 
                                                hip_pitch_default_vel, 
                                                knee_pitch_default_vel, 
                                                wheels_default_vel),
                                                0)
    kyons_default_joint_velocities = torch.zeros((num_envs, kyons.num_dof))
    for i in range(0, num_envs):
        kyons_default_joint_velocities[i, :] = kyon_default_joint_velocities

    kyons.set_joints_default_state(positions= kyons_default_joint_positions, 
                                   velocities= kyons_default_joint_velocities)

    hip_roll_kp = 200.0
    hip_roll_kd = 10.0
    hip_pitch_kp = 200.0
    hip_pitch_kd = 10.0
    knee_pitch_kp = 200.0
    knee_pitch_kd = 10.0 
    wheels_kp = 0.0
    wheels_kd = 100.0
    hip_roll_kps = torch.tensor([hip_roll_kp, hip_roll_kp, hip_roll_kp, hip_roll_kp])
    hip_roll_kds = torch.tensor([hip_roll_kd, hip_roll_kd, hip_roll_kd, hip_roll_kd])
    hip_pitch_kps = torch.tensor([hip_pitch_kp, hip_pitch_kp, hip_pitch_kp, hip_pitch_kp])
    hip_pitch_kds = torch.tensor([hip_pitch_kd, hip_pitch_kd, hip_pitch_kd, hip_pitch_kd])
    knee_pitch_kps = torch.tensor([knee_pitch_kp, knee_pitch_kp, knee_pitch_kp, knee_pitch_kp])
    knee_pitch_kds = torch.tensor([knee_pitch_kd, knee_pitch_kd, knee_pitch_kd, knee_pitch_kd])
    wheels_kps = torch.tensor([wheels_kp, wheels_kp, wheels_kp, wheels_kp])
    wheels_kds = torch.tensor([wheels_kd, wheels_kd, wheels_kd, wheels_kd])
    
    kyon_joint_kps = torch.cat((hip_roll_kps, 
                                hip_pitch_kps, 
                                knee_pitch_kps, 
                                wheels_kps),
                                0)
    kyon_joint_kds = torch.cat((hip_roll_kds, 
                                hip_pitch_kds, 
                                knee_pitch_kds, 
                                wheels_kds),
                                0)   
    kyons_joint_kps = torch.zeros((num_envs, kyons.num_dof))
    kyons_joint_kds = torch.zeros((num_envs, kyons.num_dof)) 
    for i in range(0, num_envs):
        kyons_joint_kps[i, :] = kyon_joint_kps
        kyons_joint_kds[i, :] = kyon_joint_kds

    kyons.set_gains(kps= kyons_joint_kps, kds= kyons_joint_kds, 
                    # indices= 
                    )

    kyons.initialize()

    print("Env. bodies: " + str(kyons.body_names))
    print("n. prims: " + str(n_prims))
    print("prims names: " + str(kyons.prim_paths))
    print("n. bodies: " + str(kyons.num_bodies))
    print("n. dofs: " + str(kyons.num_dof))
    print("dof names: " + str(kyons.dof_names))
    print("dof limits: " + str(kyons.get_dof_limits()))
    print("effort modes: " + str(kyons.get_effort_modes()))
    print("dof gains: " + str(kyons.get_gains()))
    print("dof max efforts: " + str(kyons.get_max_efforts()))
    print("dof gains: " + str(kyons.get_gains()))
    print("physics handle valid: " + str(kyons.is_physics_handle_valid()))

    world.reset(soft=False) 

    world.pause()

    if(headless):
        world.play()
    while simulation_app.is_running():
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset(soft=True)
            world.step(render=not headless)
        else:
            world.step(render=not headless)

    simulation_app.close()