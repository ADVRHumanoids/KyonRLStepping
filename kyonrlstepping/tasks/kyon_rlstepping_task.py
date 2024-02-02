from omni_robo_gym.tasks.custom_task import CustomTask

from control_cluster_bridge.utilities.shared_data.rhc_data import RhcCmds
from control_cluster_bridge.utilities.shared_data.jnt_imp_control import JntImpCntrlData

from typing import List

import numpy as np
import torch

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_isaac

class KyonRlSteppingTask(CustomTask):
    
    def __init__(self, 
            integration_dt: float,
            num_envs = 1,
            device = "cuda", 
            cloning_offset: np.array = None,
            replicate_physics: bool = True,
            solver_position_iteration_count: int = 4,
            solver_velocity_iteration_count: int = 1,
            solver_stabilization_thresh: float = 1e-5,
            offset=None, 
            env_spacing = 5.0, 
            spawning_radius = 1.0, 
            use_flat_ground = True,
            default_jnt_stiffness = 100.0,
            default_jnt_damping = 10.0,
            default_wheel_stiffness = 0.0,
            default_wheel_damping = 10.0,
            startup_jnt_stiffness = 50,
            startup_jnt_damping = 5,
            startup_wheel_stiffness = 0.0,
            startup_wheel_damping = 10.0,
            robot_names = ["kyon0"],
            robot_pkg_names = ["kyon"],
            contact_prims = None,
            contact_offsets = None,
            sensor_radii = None,
            use_diff_velocities = True,
            override_art_controller = False,
            dtype = torch.float64,
            debug_mode_jnt_imp = False) -> None:

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
                    solver_position_iteration_count = solver_position_iteration_count,
                    solver_velocity_iteration_count = solver_velocity_iteration_count,
                    solver_stabilization_thresh = solver_stabilization_thresh,
                    offset = offset, 
                    env_spacing = env_spacing, 
                    use_flat_ground = use_flat_ground,
                    default_jnt_stiffness = default_jnt_stiffness,
                    default_jnt_damping = default_jnt_damping,
                    default_wheel_stiffness = default_wheel_stiffness,
                    default_wheel_damping = default_wheel_damping,
                    override_art_controller = override_art_controller,
                    dtype = dtype, 
                    self_collide = [False] * len(robot_names), 
                    fix_base = [False] * len(robot_names),
                    merge_fixed = [True] * len(robot_names),
                    debug_mode_jnt_imp = debug_mode_jnt_imp)
        
        self.use_diff_velocities = use_diff_velocities
        
        self._debug_mode_jnt_imp = debug_mode_jnt_imp

        self.startup_jnt_stiffness = startup_jnt_stiffness
        self.startup_jnt_damping = startup_jnt_damping
        self.startup_wheel_stiffness = startup_wheel_stiffness
        self.startup_wheel_damping = startup_wheel_damping

        self.jnt_imp_cntrl_shared_data = {}
    
    def _custom_post_init(self):

        # will be called at the end of the post-initialization steps

        for i in range(0, len(self.robot_names)):
            
            robot_name = self.robot_names[i]

            from SharsorIPCpp.PySharsorIPC import VLevel

            self.jnt_imp_cntrl_shared_data[robot_name] = JntImpCntrlData(is_server = True, 
                                            n_envs = self.num_envs, 
                                            n_jnts = self.robot_n_dofs[robot_name],
                                            jnt_names = self.jnt_imp_controllers[robot_name].jnts_names,
                                            namespace = robot_name, 
                                            verbose = True, 
                                            vlevel = VLevel.V2)

            self.jnt_imp_cntrl_shared_data[robot_name].run()

    def _update_jnt_imp_cntrl_shared_data(self):

        if self._debug_mode_jnt_imp:

            for i in range(0, len(self.robot_names)):
            
                robot_name = self.robot_names[i]

                success = True

                # updating all the jnt impedance data - > this introduces some overhead. 
                # disable this with debug_mode_jnt_imp when debugging is not necessary
                success = self.jnt_imp_cntrl_shared_data[robot_name].pos_err_view.write(
                    self.jnt_imp_controllers[robot_name].pos_err(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].vel_err_view.write(
                    self.jnt_imp_controllers[robot_name].vel_err(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].pos_gains_view.write(
                    self.jnt_imp_controllers[robot_name].pos_gains(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].vel_gains_view.write(
                    self.jnt_imp_controllers[robot_name].vel_gains(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].eff_ff_view.write(
                    self.jnt_imp_controllers[robot_name].eff_ref(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].pos_view.write(
                    self.jnt_imp_controllers[robot_name].pos(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].pos_ref_view.write(
                    self.jnt_imp_controllers[robot_name].pos_ref(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].vel_view.write(
                    self.jnt_imp_controllers[robot_name].vel(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].vel_ref_view.write(
                    self.jnt_imp_controllers[robot_name].vel_ref(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].eff_view.write(
                    self.jnt_imp_controllers[robot_name].eff(), 0, 0
                    ) and success
                success = self.jnt_imp_cntrl_shared_data[robot_name].imp_eff_view.write(
                    self.jnt_imp_controllers[robot_name].imp_eff(), 0, 0
                    ) and success

                if not success:
                    
                    message = f"[{self.__class__.__name__}]" + \
                        f"[{self._journal.status}]" + \
                        ": Could not update all jnt. imp. controller info on shared memory," + \
                        " probably because the data was already owned at the time of writing. Data might be lost"

                    print(message)

    def _xrdf_cmds(self):

        n_robots = len(self.robot_names)

        cmds = get_xrdf_cmds_isaac(n_robots=n_robots, 
                                robot_pkg_name=self.robot_pkg_names[0]) # assuming
        # kyon package path is the first element (true if only one robot type, i.e. Kyon)

        return cmds
      
    def post_reset(self):

        # self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        # self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # # randomize all envs
        # indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        # self.reset(indices)

        a = 1
    
    def reset(self, 
            env_ids: List[int]=None,
            robot_names: List[str]=None):

        super().reset(env_ids=env_ids, 
                    robot_names=robot_names)

    def _step_jnt_imp_control(self,
                        robot_name: str, 
                        actions: RhcCmds = None,
                        env_indxs: torch.Tensor = None):

        if env_indxs is not None:

            if not isinstance(env_indxs, torch.Tensor):
                    
                msg = "Provided env_indxs should be a torch tensor of indexes!"
            
                raise Exception(f"[{self.__class__.__name__}]" + f"[{self._journal.exception}]: " + msg)
        
        # always updated imp. controller internal state (jnt imp control is supposed to be
        # always running)
        success = self.jnt_imp_controllers[robot_name].update_state(pos = self.jnts_q(robot_name=robot_name), 
                                                    vel = self.jnts_v(robot_name=robot_name),
                                                    eff = None)
        
        if not all(success):
            
            print(success)

            exception = "Could not update the whole joint impedance state!!"
            
            raise Exception(exception)

        if actions is not None:
            
            # if new actions are received, also update references

            if env_indxs is None:

                self.jnt_imp_controllers[robot_name].set_refs(
                                            pos_ref = actions.jnts_state.get_q(gpu=self.using_gpu), 
                                            vel_ref = actions.jnts_state.get_v(gpu=self.using_gpu), 
                                            eff_ref = actions.jnts_state.get_eff(gpu=self.using_gpu),
                                            robot_indxs = env_indxs)
            else:

                self.jnt_imp_controllers[robot_name].set_refs(
                                            pos_ref = actions.jnts_state.get_q(gpu=self.using_gpu)[env_indxs, :], 
                                            vel_ref = actions.jnts_state.get_v(gpu=self.using_gpu)[env_indxs, :], 
                                            eff_ref = actions.jnts_state.get_eff(gpu=self.using_gpu)[env_indxs, :],
                                            robot_indxs = env_indxs)
        
        # # jnt imp. controller actions are always applied
        self.jnt_imp_controllers[robot_name].apply_cmds()

        self._update_jnt_imp_cntrl_shared_data() # only if debug_mode_jnt_imp is enabled
        
    def pre_physics_step(self, 
            robot_name: str, 
            actions: RhcCmds = None,
            env_indxs: torch.Tensor = None) -> None:

        # just step joint impedance control
        self._step_jnt_imp_control(robot_name = robot_name,
                                actions = actions,
                                env_indxs = env_indxs)

    def get_observations(self):
        
        if self.use_diff_velocities:
            
            self._get_robots_state(dt = self.integration_dt) # updates robot states
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

    def close(self):

        for i in range(0, len(self.robot_names)):
            
            robot_name = self.robot_names[i]

            # closing shared memory
            self.jnt_imp_cntrl_shared_data[robot_name].close()
