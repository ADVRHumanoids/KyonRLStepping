from omni_custom_gym.tasks.custom_task import CustomTask

import numpy as np

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

        a = 2

    def pre_physics_step(self, actions) -> None:
        
        self._get_robots_state()

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
