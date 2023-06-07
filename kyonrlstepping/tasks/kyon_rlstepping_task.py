from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view

from omni.isaac.core.robots import Robot

from omni.isaac.core.utils.prims import move_prim

from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.rotations import euler_angles_to_quat 

from omni.isaac.core.controllers import ArticulationController

import omni.kit

import gymnasium as gym
from gym import spaces
import numpy as np
import torch
import math

class KyonRlSteppingTask(BaseTask):
    def __init__(self, name, offset=None) -> None:

        # task-specific parameters
        self._position = np.array([0, 0, 0.8])
        self._orientation = euler_angles_to_quat(np.array([0, 0, 0]), degrees = True)

        # values used for defining RL buffers
        self._num_observations = 4
        self._num_actions = 1
        self._device = "cuda" # defaults to "cuda" ("cpu" also valid)
        self.num_envs = 1

        # a few class buffers to store RL-related states
        self.obs = torch.zeros((self.num_envs, self._num_observations))
        self.resets = torch.zeros((self.num_envs, 1))

        # set the action and observation space for RL
        self.action_space = spaces.Box(np.ones(self._num_actions) * -1.0, np.ones(self._num_actions) * 1.0)
        self.observation_space = spaces.Box(
            np.ones(self._num_observations) * -np.Inf, np.ones(self._num_observations) * np.Inf
        )

        # trigger __init__ of parent class
        BaseTask.__init__(self, name=name, offset=offset)
    
    def _import_urdf(self, 
                    urdf_path):

        self._urdf_path = urdf_path

        self._urdf_interface = _urdf.acquire_urdf_interface()
        self._import_config = _urdf.ImportConfig()
        self._import_config.merge_fixed_joints = True
        self._import_config.convex_decomp = False
        self._import_config.import_inertia_tensor = True
        self._import_config.fix_base = False
        self._import_config.make_default_prim = True
        self._import_config.self_collision = True
        self._import_config.create_physics_scene = False
        self._import_config.import_inertia_tensor = False
        self._import_config.default_drive_strength = 10.0 # strange behavior: not parsed, unless == 0
        self._import_config.default_position_drive_damping = 1.0 # strange behavior: not parsed, unless == 0
        self._import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        self._import_config.distance_scale = 1
        # self._import_config.density = 0.0

        # Get path to extension data:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
        extension_path = ext_manager.get_extension_path(ext_id)

        # import URDF
        success, kyon_prim_path_default = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=self._import_config, 
        )
        self._kyon_prim_base_path = "/World/Kyon"
        move_prim(kyon_prim_path_default, self._kyon_prim_base_path) # we override 
        # default prim path from URDFParseAndImportFile

        self._kyon_jnt_imp_controller = ArticulationController() # we create
        # a joint impedance controller

        self._kyon_robot = Robot(prim_path=self._kyon_prim_base_path, name="kyon_robot", 
                            position=self._position,
                            orientation=self._orientation, 
                            articulation_controller = self._kyon_jnt_imp_controller)
        
        self._kyon_dofs_names = self._kyon_robot.dof_names

        enable_kyon_self_coll = True
        self._kyon_robot.set_enabled_self_collisions(enable_kyon_self_coll)

        self._kyon_prim = self._kyon_robot.prim
        
        self._kyons = ArticulationView(prim_paths_expr="/World/Kyon*", name="kyon_view")

        return success

    def set_up_scene(self, 
                    scene, 
                    urdf_path: str) -> None:
        
        self._import_urdf(urdf_path)
        
        self._kyons_articulations = scene.add(self._kyons)

        scene.add_default_ground_plane()

        # set default camera viewport position and target
        self.set_initial_camera_params()

    def set_initial_camera_params(self, camera_position=[10, 10, 3], camera_target=[0, 0, 0]):
        set_camera_view(eye=camera_position, target=camera_target, camera_prim_path="/OmniverseKit_Persp")

    def post_reset(self):
        self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # randomize all envs
        indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        self.reset(indices)

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self._device)
        num_resets = len(env_ids)

        # randomize DOF positions
        dof_pos = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        dof_pos[:, self._cart_dof_idx] = 1.0 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        dof_pos[:, self._pole_dof_idx] = 0.125 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # randomize DOF velocities
        dof_vel = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        dof_vel[:, self._cart_dof_idx] = 0.5 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        dof_vel[:, self._pole_dof_idx] = 0.25 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # apply resets
        indices = env_ids.to(dtype=torch.int32)
        self._cartpoles.set_joint_positions(dof_pos, indices=indices)
        self._cartpoles.set_joint_velocities(dof_vel, indices=indices)

        # bookkeeping
        self.resets[env_ids] = 0

    def pre_physics_step(self, actions) -> None:
        reset_env_ids = self.resets.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            self.reset(reset_env_ids)

        actions = torch.tensor(actions)

        forces = torch.zeros((self._cartpoles.count, self._cartpoles.num_dof), dtype=torch.float32, device=self._device)
        forces[:, self._cart_dof_idx] = self._max_push_effort * actions[0]

        indices = torch.arange(self._cartpoles.count, dtype=torch.int32, device=self._device)
        self._cartpoles.set_joint_efforts(forces, indices=indices)

    def get_observations(self):
        dof_pos = self._cartpoles.get_joint_positions()
        dof_vel = self._cartpoles.get_joint_velocities()

        # collect pole and cart joint positions and velocities for observation
        cart_pos = dof_pos[:, self._cart_dof_idx]
        cart_vel = dof_vel[:, self._cart_dof_idx]
        pole_pos = dof_pos[:, self._pole_dof_idx]
        pole_vel = dof_vel[:, self._pole_dof_idx]

        self.obs[:, 0] = cart_pos
        self.obs[:, 1] = cart_vel
        self.obs[:, 2] = pole_pos
        self.obs[:, 3] = pole_vel

        return self.obs

    def calculate_metrics(self) -> None:
        cart_pos = self.obs[:, 0]
        cart_vel = self.obs[:, 1]
        pole_angle = self.obs[:, 2]
        pole_vel = self.obs[:, 3]

        # compute reward based on angle of pole and cart velocity
        reward = 1.0 - pole_angle * pole_angle - 0.01 * torch.abs(cart_vel) - 0.005 * torch.abs(pole_vel)
        # apply a penalty if cart is too far from center
        reward = torch.where(torch.abs(cart_pos) > self._reset_dist, torch.ones_like(reward) * -2.0, reward)
        # apply a penalty if pole is too far from upright
        reward = torch.where(torch.abs(pole_angle) > np.pi / 2, torch.ones_like(reward) * -2.0, reward)

        return reward.item()

    def is_done(self) -> None:
        cart_pos = self.obs[:, 0]
        pole_pos = self.obs[:, 2]

        # reset the robot if cart has reached reset_dist or pole is too far from upright
        resets = torch.where(torch.abs(cart_pos) > self._reset_dist, 1, 0)
        resets = torch.where(torch.abs(pole_pos) > math.pi / 2, 1, resets)
        self.resets = resets

        return resets.item()
