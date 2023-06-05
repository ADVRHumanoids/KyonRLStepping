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
try:

    xacro_gen = subprocess.check_call(["xacro",\
                                    kyon_xacro_path, \
                                    add_wheels, \
                                    "-o", 
                                    kyon_urdf_path])

except:

    print('FAILED TO GENERATE URDF.')
        
if __name__ == "__main__":

    world = World(
        stage_units_in_meters=1.0, 
        rendering_dt=1.0/60.0,
        backend="torch", 
        device="cpu",
    )

    # world.scene.add_default_ground_plane()

    urdf_interface = _urdf.acquire_urdf_interface()
    
    # setting up import configuration:

    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = True
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.self_collision = True
    import_config.create_physics_scene = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_strength = 0.0
    import_config.default_position_drive_damping = 0.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0

    # Get path to extension data:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
    extension_path = ext_manager.get_extension_path(ext_id)

    # import URDF
    success, prim_name = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=kyon_urdf_path,
        import_config=import_config, 
    )

    camera_state = ViewportCameraState("/OmniverseKit_Persp")
    camera_state.set_position_world(Gf.Vec3d(1.22, -1.24, 1.13), True)
    camera_state.set_target_world(Gf.Vec3d(-0.96, 1.08, 0.0), True)

    # get stage handle
    stage = omni.usd.get_context().get_stage()

    result, plane_path = omni.kit.commands.execute(
            "AddGroundPlaneCommand",
            stage=stage,
            planePath="/groundPlane",
            axis="Z",
            size=1500.0,
            position=Gf.Vec3f(0, 0, -10.0),
            color=Gf.Vec3f(0.5),
        )
    
    # make sure the ground plane is under root prim and not robot
    omni.kit.commands.execute(
        "MovePrimCommand", path_from=plane_path, path_to="/groundPlane", keep_world_transform=True
    )
    
    # enable physics
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    # set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

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