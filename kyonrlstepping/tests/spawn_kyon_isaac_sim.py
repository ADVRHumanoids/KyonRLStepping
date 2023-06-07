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

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.urdf._urdf import UrdfJointTargetType

import numpy as np
import torch 
cuda0 = torch.device('cuda:0')

from omni.isaac.core.controllers import ArticulationController

from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools
# from omni.isaac.core.utils.extensions import get_extension_path_from_name
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

    print('\nFAILED TO GENERATE KYON\'S URDF!!!.\n')
        
if __name__ == "__main__":
    
    rendering_freq = 60.0 # [Hz]
    integration_freq = 100.0 # [Hz]

    device = "cpu" # either "cpu" or "cuda"
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0/integration_freq, 
        rendering_dt=1.0/rendering_freq,
        backend="torch", # "torch" or "numpy"
        device=device,
        physics_prim_path="/physicsScene"
    )
    
    # get world stage
    scene = world.scene
    # get stage handle
    stage = omni.usd.get_context().get_stage()

    # add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

    # scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))

    physics_context = world.get_physics_context()
    physics_context.enable_gpu_dynamics(True)
    physics_context.enable_stablization(True)
    physics_scene_prim = physics_context.get_current_physics_scene_prim()
    solver_type = physics_context.get_solver_type()
    # solver = "TGS" # TGS (default) or PGS (more accurate?)
    # physics_context.set_solver_type(solver)

    # adding ground plane
    # result, plane_path = omni.kit.commands.execute(
    #     "AddGroundPlaneCommand",
    #     stage=stage,
    #     planePath="/groundPlane",
    #     axis="Z",
    #     size=1500.0,
    #     position=Gf.Vec3f(0, 0, 0.0),
    #     color=Gf.Vec3f(0.5),
    # )
    # make sure the ground plane is under root prim and not robot
    # omni.kit.commands.execute(
    #     "MovePrimCommand", path_from=plane_path, path_to="/groundPlane", keep_world_transform=True
    # )
    # ground_plane = scene.add_ground_plane(z_position=0, name="ground_plane", 
    #                         prim_path= "/World/ground_plane", 
    #                         static_friction=0.5, 
    #                         dynamic_friction=0.5, 
    #                         restitution=0.8, 
    #                         color = np.array([0.7, 0.7, 0.7]))
    ground_plane = scene.add_default_ground_plane(z_position=0, name="ground_plane", 
                            prim_path= "/World/ground_plane", 
                            static_friction=0.5, 
                            dynamic_friction=0.5, 
                            restitution=0.8)
    ground_plane_path = ground_plane.prim_path

    # dynamic_cuboid = scene.add(
    #     DynamicCuboid(
    #         prim_path="/dynamic_cuboid",
    #         name="dynamic_cuboid",
    #         position=np.array([0, 0, 2.0]),
    #         orientation=euler_angles_to_quat(np.array([45, 45, 45]), degrees = True),
    #         scale=np.array([1.0, 1.0, 1.0]),
    #         size=0.2,
    #         color=np.array([0.8, 0.6, 1.0]),
    #         mass=50.0
    #     )
    # )   
    dynamic_sphere = scene.add(
        DynamicSphere(
            prim_path="/dynamic_sphere",
            name="dynamic_sphere",
            position=np.array([0, 0, 5.5]),
            radius=0.2,
            color=np.array([0.2, 0.6, 1.0]),
            mass=100.0
        )
    )   

    # scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    # scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()

    import_config.merge_fixed_joints = True
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = True
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = False
    import_config.default_drive_strength = 10.0 # strange behavior: not parsed, unless == 0
    import_config.default_position_drive_damping = 1.0 # strange behavior: not parsed, unless == 0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION # JOINT_DRIVE_POSITION, JOINT_DRIVE_VELOCITY, JOINT_DRIVE_NONE
    import_config.distance_scale = 1
    # import_config.density = 0.0

    # Get path to extension data:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
    extension_path = ext_manager.get_extension_path(ext_id)
    # import URDF
    success, kyon_prim_path_default = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=kyon_urdf_path,
        import_config=import_config, 
    )
    kyon_prim_path = "/World/Kyon"
    move_prim(kyon_prim_path_default, kyon_prim_path)

    kyon_orientation = euler_angles_to_quat(np.array([0, 0, 0]), degrees = True)
    kyon_position = Gf.Vec3f(0, 0, 0.8)
    kyon_jnt_imp_controller = ArticulationController()
    kyon_robot = Robot(prim_path=kyon_prim_path, name="awesome_kyon", 
                                              position=kyon_position,
                                              orientation=kyon_orientation, 
                                              articulation_controller = kyon_jnt_imp_controller)
    kyon_prim = kyon_robot.prim

    kyon_articulation = world.scene.add(kyon_robot) # necessary. otherwise we are not
    # able to set positions, default states, etc...
    enable_kyon_self_coll = True
    kyon_robot.set_enabled_self_collisions(enable_kyon_self_coll)
    
    # kyon_robot.switch_control_mode()
    # camera_state = ViewportCameraState("/OmniverseKit_Persp")
    # camera_state.set_position_world(Gf.Vec3d(1.22, -1.24, 1.13), True)
    # camera_state.set_target_world(Gf.Vec3d(-0.96, 1.08, 0.0), True)

    world.reset(soft=False) # firTruest reset should be with soft flag set to False.
    # Additionally, this method takes care of initializing articulation handles 
    # with the first reset called and will also do one simulation step internally.

    hip_roll_default_config = torch.tensor([0.3, -0.3, 0.3, -0.3])
    hip_pitch_default_config = torch.tensor([-0.3, -0.3, 0.3, 0.3])
    knee_pitch_default_config = torch.tensor([0.3, 0.3, -0.3, -0.3])
    wheels_default_config = torch.tensor([0.0, 0.0, 0.0, 0.0])
    kyon_default_joint_positions = torch.cat((hip_roll_default_config, 
                                                hip_pitch_default_config, 
                                                knee_pitch_default_config, 
                                                wheels_default_config),
                                                0)
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
    
    kyon_default_joint_efforts = torch.cat((torch.tensor([0, 0, 0, 0]), 
                                            torch.tensor([0, 0, 0, 0]), 
                                            torch.tensor([0, 0, 0, 0]), 
                                            torch.tensor([0.0, -0.0, 0.0, -0.0])),
                                            0)

    kyon_robot.set_joints_default_state(positions = kyon_default_joint_positions, 
                                        velocities = kyon_default_joint_velocities)
    kyon_robot.set_default_state(position = kyon_position, 
                                orientation = kyon_orientation) # default root orientation and position
    
    hip_roll_kp = 500.0
    hip_roll_kd = 20.0
    hip_pitch_kp = 500.0
    hip_pitch_kd = 20.0
    knee_pitch_kp = 500.0
    knee_pitch_kd = 20.0 
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
    kyon_jnt_imp_controller.set_gains(kps = kyon_joint_kps,
                                      kds = kyon_joint_kds) # this calls the low level set_gains from 
    #ArticulationView. So, if all gains are set and no switch_control_mode is called, 
    # ArticulationController can also be used as a full impedance controller.
    # kyon_jnt_imp_controller.switch_control_mode("effort") # do not call this if the intention is
    # to use full joint impedance control: this will internally call set_gains and only allow for one 
    # between position, velocity and effort control modes to be used at the same time !!!

    kyon_robot.initialize() # initialize robot (it probably also initializes the ArticulationController)

    # kyon_joints_state = kyon_robot.get_joints_state() 
    kyon_joints_positions = kyon_robot.get_joint_positions()# these methods have to be called 
    # after the call to initialize(), otherwise they return None objects
    kyon_joints_velocities = kyon_robot.get_joint_velocities()

    kyon_dofs_names = kyon_robot.dof_names

    print("KYON joint names: " + ', '.join(kyon_dofs_names))
    print("KYON n. DOFs: " + str(len(kyon_dofs_names)))
    print("KYON joint configuration: " + str(kyon_joints_positions))
    print("joint positions device: " + str(kyon_joints_positions.get_device())) # -1 if on CPU
    print("KYON joint velocities: " + str(kyon_joints_velocities))
    print("joint velocities device: " + str(kyon_joints_velocities.get_device()))
    print("KYON joint position gains: " + str(kyon_jnt_imp_controller.get_gains()[0]))
    print("KYON joint velocity gains: " + str(kyon_jnt_imp_controller.get_gains()[1]))
    print("KYON joint drive mode: " + str(kyon_jnt_imp_controller.get_effort_modes()))
    print("KYON joint limits: " + str(kyon_jnt_imp_controller.get_joint_limits()))
    print("KYON max efforts: " + str(kyon_jnt_imp_controller.get_max_efforts()))

    kyon_articulation.set_joint_positions(kyon_default_joint_positions)
    world.pause()
    if(headless):
        world.play()
    while simulation_app.is_running():
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset(soft=True)
            world.step(render=not headless)
            
            control_action = ArticulationAction(joint_positions=kyon_default_joint_positions, 
                                   joint_velocities=kyon_default_joint_velocities, 
                                   joint_efforts=kyon_default_joint_efforts)
            kyon_jnt_imp_controller.apply_action(control_action)

        else:
            world.step(render=not headless)

        print("KYON joint configuration: " + str(kyon_robot.get_joint_positions()))

    simulation_app.close()