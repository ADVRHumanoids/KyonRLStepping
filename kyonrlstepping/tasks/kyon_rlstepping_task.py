from omni_robo_gym.tasks.custom_task import CustomTask

from control_cluster_bridge.utilities.control_cluster_defs import RobotClusterCmd

import numpy as np
import torch

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_isaac

class KyonRlSteppingTask(CustomTask):
    def __init__(self, 
                cluster_dt: float, 
                integration_dt: float,
                num_envs = 1,
                device = "cuda", 
                cloning_offset: np.array = None,
                replicate_physics: bool = True,
                offset=None, 
                env_spacing = 5.0, 
                spawning_radius = 1.0, 
                use_flat_ground = True,
                default_jnt_stiffness = 100.0,
                default_jnt_damping = 10.0,
                jnt_stiffness_at_startup = 50,
                jnt_damping_at_startup = 5,
                robot_names = ["kyon0"],
                robot_pkg_names = ["kyon"],
                contact_prims = None,
                contact_offsets = None,
                sensor_radii = None,
                use_diff_velocities = True,
                dtype = torch.float64) -> None:

        if cloning_offset is None:
        
            cloning_offset = np.array([[0.0, 0.0, 0.0]] * num_envs)
        
        if contact_prims is None:

            contact_prims = {}

            for i in range(len(robot_names)):
                
                contact_prims[robot_names[i]] = [] # no contact sensors

        # trigger __init__ of parent class
        CustomTask.__init__(self,
                    name = self.__class__.__name__, 
                    integration_dt = integration_dt,
                    robot_names = robot_names,
                    robot_pkg_names = robot_pkg_names,
                    num_envs = num_envs,
                    contact_prims = contact_prims,
                    contact_offsets = contact_offsets,
                    sensor_radii = sensor_radii,
                    device = device, 
                    cloning_offset = cloning_offset,
                    spawning_radius = spawning_radius,
                    replicate_physics = replicate_physics,
                    offset = offset, 
                    env_spacing = env_spacing, 
                    use_flat_ground = use_flat_ground,
                    default_jnt_stiffness = default_jnt_stiffness,
                    default_jnt_damping = default_jnt_damping,
                    dtype = dtype, 
                    self_collide = [False] * len(robot_names), 
                    fix_base = [False] * len(robot_names),
                    merge_fixed = [True] * len(robot_names))
        
        self.cluster_dt = cluster_dt
        self.use_diff_velocities = use_diff_velocities

        self.jnt_stiffness_at_startup = jnt_stiffness_at_startup
        self.jnt_damping_at_startup = jnt_damping_at_startup
        
    def _xrdf_cmds(self):

        n_robots = len(self.robot_names)

        cmds = get_xrdf_cmds_isaac(n_robots=n_robots)

        return cmds
      
    def post_reset(self):
        # self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        # self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # # randomize all envs
        # indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        # self.reset(indices)

        a = 1
    
    def reset(self, env_ids=None):

        super().reset()

    def pre_physics_step(self, 
            robot_name: str, 
            actions: RobotClusterCmd = None) -> None:
        
        if actions is not None:
            
            self.jnt_imp_controllers[robot_name].set_refs(
                                        pos_ref = actions.jnt_cmd.q, 
                                        vel_ref = actions.jnt_cmd.v, 
                                        eff_ref = actions.jnt_cmd.eff)
                    
            self.jnt_imp_controllers[robot_name].apply_refs()
            
            # print("cmd debug" + "\n" + 
            #         "q_cmd: " + str(actions.jnt_cmd.q) + "\n" + 
            #         "v_cmd: " + str(actions.jnt_cmd.v) + "\n" + 
            #         "eff_cmd: " + str(actions.jnt_cmd.eff))

    def get_observations(self):
        
        if self.use_diff_velocities:

            self._get_robots_state(self.integration_dt) # updates robot states
            # but velocities are obtained via num. differentiation
        
        else:

            self._get_robots_state() # velocities directly from simulator (can 
            # introduce relevant artifacts, making them unrealistic)

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
