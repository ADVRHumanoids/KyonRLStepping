from omni_custom_gym.tasks.custom_task import CustomTask

from control_cluster_utils.utilities.control_cluster_utils import RobotClusterCmd

import numpy as np
import torch

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_isaac

class KyonRlSteppingTask(CustomTask):
    def __init__(self, 
                cluster_dt: float, 
                integration_dt: float,
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
        
        self.cluster_dt = cluster_dt
        self.integration_dt = integration_dt
        
    def _xrdf_cmds(self):

        cmds = get_xrdf_cmds_isaac()

        return cmds
      
    def post_reset(self):
        # self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        # self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # # randomize all envs
        # indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        # self.reset(indices)

        a = 1
    
    def reset(self, env_ids=None):

        # self._jnt_imp_controller.set_refs(pos_ref=self._homer.get_homing())
        
        self._jnt_imp_controller.apply_refs()

    def pre_physics_step(self, 
            actions: RobotClusterCmd) -> None:
        
        no_gains_pos = torch.full((self.num_envs, self.robot_n_dofs), 
                    1.0, 
                    device = self.torch_device, 
                    dtype=torch.float32)
        
        no_gains_vel = torch.full((self.num_envs, self.robot_n_dofs), 
                    0.1, 
                    device = self.torch_device, 
                    dtype=torch.float32)

        self._jnt_imp_controller.set_gains(pos_gains = no_gains_pos,
                            vel_gains = no_gains_vel)
        
        self._jnt_imp_controller.set_refs(pos_ref = actions.jnt_cmd.q, 
                                    vel_ref = actions.jnt_cmd.v,
                                    eff_ref = actions.jnt_cmd.eff)
                
        self._jnt_imp_controller.apply_refs()

        print("####################")
        # print(self._robots_art_view.get_applied_actions().joint_efforts)
        print(actions.jnt_cmd.eff)
        print("####################")

    def get_observations(self):
        
        self._get_robots_state() # updates joints states

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
