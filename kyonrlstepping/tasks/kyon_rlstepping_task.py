from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.viewports import set_camera_view

from omni.isaac.core.utils.rotations import euler_angles_to_quat 

import omni.kit

import gymnasium as gym
from gym import spaces
import numpy as np
import torch
import math

from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.prims import move_prim
from omni.isaac.cloner import GridCloner
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.urdf._urdf import UrdfJointTargetType

from omni.isaac.core.utils.types import ArticulationActions

from omni.isaac.core.scenes.scene import Scene

from omnicustomgym.tasks.custom_task import CustomTask

class KyonRlSteppingTask(CustomTask):
    def __init__(self, 
                num_envs = 1,
                device = "cuda", 
                cloning_offset: np.array = np.array([0.0, 0.0, 0.0]),
                replicate_physics: bool = True,
                offset=None, 
                env_spacing = 5.0) -> None:

        # trigger __init__ of parent class
        CustomTask.__init__(self,
                        name = self.__class__.__name__, 
                        robot_name = "kyon",
                        num_envs = num_envs,
                        device = device, 
                        cloning_offset = cloning_offset,
                        replicate_physics = replicate_physics,
                        offset = offset, 
                        env_spacing = env_spacing)
        
        self.xrdf_cmd_vals = [True, False, False, False, False] # overrides base class default values

    def _xrdf_cmds(self):

        cmds = []
        
        wheels = "true" if self.xrdf_cmd_vals[0] else "false"
        upper_body = "true" if self.xrdf_cmd_vals[1] else "false"
        sensors = "true" if self.xrdf_cmd_vals[2] else "false"
        floating_joint = "true" if self.xrdf_cmd_vals[3] else "false"
        payload = "true" if self.xrdf_cmd_vals[4] else "false"

        cmds.append("wheels:=" + wheels)
        cmds.append("upper_body:=" + upper_body)
        cmds.append("sensors:=" + sensors)
        cmds.append("floating_joint:=" + floating_joint)
        cmds.append("payload:=" + payload)

        return cmds
      
    def post_reset(self):
        # self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        # self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # # randomize all envs
        # indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        # self.reset(indices)

        a = 1
    
    def reset(self, env_ids=None):
        # if env_ids is None:
        #     env_ids = torch.arange(self.num_envs, device=self._device)
        # num_resets = len(env_ids)

        # # randomize DOF positions
        # dof_pos = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        # dof_pos[:, self._cart_dof_idx] = 1.0 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        # dof_pos[:, self._pole_dof_idx] = 0.125 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # # randomize DOF velocities
        # dof_vel = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        # dof_vel[:, self._cart_dof_idx] = 0.5 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        # dof_vel[:, self._pole_dof_idx] = 0.25 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # # apply resets
        # indices = env_ids.to(dtype=torch.int32)
        # self._cartpoles.set_joint_positions(dof_pos, indices=indices)
        # self._cartpoles.set_joint_velocities(dof_vel, indices=indices)

        # # bookkeeping
        # self.resets[env_ids] = 0

        a = 1

    def pre_physics_step(self, actions) -> None:
        
        wheel_idxs = self._jnt_imp_controller.get_jnt_idxs_matching("wheel")
        # pos_ref = torch.mul(torch.sub(torch.rand((1, len(knee_indxs)), device = self._device), 0.5), 2.0)
        eff_ref = torch.mul(torch.sub(torch.rand((1, len(wheel_idxs)), device = self._device), 0.5), 5.0)
        success = self._jnt_imp_controller.set_refs(eff_ref=eff_ref, 
                                        pos_ref=None, 
                                        vel_ref=None, 
                                        jnt_indxs=wheel_idxs, 
                                        robot_indxs=torch.tensor([0], device=self._device))

        self._jnt_imp_controller.apply_refs()

    def get_observations(self):

        self._get_jnts_state() # updates joints states

        return self.obs

    def calculate_metrics(self) -> None:

        # compute reward based on angle of pole and cart velocity
        reward = 0

        return reward

    def is_done(self) -> None:
        # cart_pos = self.obs[:, 0]
        # pole_pos = self.obs[:, 2]

        # # reset the robot if cart has reached reset_dist or pole is too far from upright
        # resets = torch.where(torch.abs(cart_pos) > self._reset_dist, 1, 0)
        # resets = torch.where(torch.abs(pole_pos) > math.pi / 2, 1, resets)
        # self.resets = resets

        return True
