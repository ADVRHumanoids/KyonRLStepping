from omni.isaac.kit import SimulationApp 

simulation_app = SimulationApp( {"headless": False} )

import omni.kit.commands 
from omni.kit.viewport.utility.camera_state import ViewportCameraState

from abc import abstractmethod
# from omni.isaac.core.tasks import BaseTask
# from omni.isaac.core.prims import RigidPrimView, RigidPrim, XFormPrim
from omni.isaac.core import World
# from omni.isaac.core.objects import DynamicSphere
# from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
# from omni.isaac.core.utils.nucleus import find_nucleus_server
# from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
# from omni.isaac.core.materials import PreviewSurface
# from omni.isaac.cloner import GridCloner
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.rotations import euler_angles_to_quat 
from omni.isaac.core.simulation_context import SimulationContext

import numpy as np

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
    
    device = "cpu" # either "cpu" or "cuda"
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0/60.0, 
        rendering_dt=1.0/60.0,
        backend="torch", 
        device=device,
        physics_prim_path="/physicsScene"
    )
    
    # get world stage
    scene = world.scene
    # get stage handle
    stage = omni.usd.get_context().get_stage()

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

    # scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    # scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = True
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = False
    import_config.default_drive_strength = 100.0
    import_config.default_position_drive_damping = 10.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0
    # Get path to extension data:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
    extension_path = ext_manager.get_extension_path(ext_id)
    # import URDF
    success, kyon_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=kyon_urdf_path,
        import_config=import_config, 
    )

    kyon_orientation = euler_angles_to_quat(np.array([0, 0, 0]), degrees = True)
    kyon_position = Gf.Vec3f(0, 0, 0.8)
    kyon_jnt_imp_controller = ArticulationController()
    kyon_robot = Robot(prim_path=kyon_prim_path, name="awesome_kyon", 
                                              position=kyon_position,
                                              orientation=kyon_orientation, 
                                              articulation_controller = kyon_jnt_imp_controller)
    kyon_robot.set_default_state(position = kyon_position, 
                                orientation = kyon_orientation)
    enable_kyon_self_coll = True
    kyon_robot.set_enabled_self_collisions(enable_kyon_self_coll)
    # kyon_joints_state = kyon_robot.get_joints_state()
    # kyon_joints_positions = kyon_robot.get_joint_positions()
    # kyon_joints_velocities = kyon_robot.get_joint_velocities()

    # kyon_robot.initialize()
    # kyon_dofs_names = kyon_robot.dof_names

    # kyon_robot.set_joint_default_state(positions = kyon_joints_positions, 
    #                                    velocities = None,
    #                                    efforts = None)
    # camera_state = ViewportCameraState("/OmniverseKit_Persp")
    # camera_state.set_position_world(Gf.Vec3d(1.22, -1.24, 1.13), True)
    # camera_state.set_target_world(Gf.Vec3d(-0.96, 1.08, 0.0), True)

    
    # enable physics
    
    # kyon_articulation = world.scene.add(kyon_robot)
    # add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

    world.reset(soft=False)
    world.pause()
    while simulation_app.is_running():
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset(soft=True)
            world.step(render=True)
        else:
            world.step(render=True)

    simulation_app.close()