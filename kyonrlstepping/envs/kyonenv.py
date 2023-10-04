from omni_custom_gym.gym.omni_vect_env.vec_envs import RobotVecEnv

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

import torch 
import numpy as np

class KyonEnv(RobotVecEnv):

    def set_task(self, 
                task, 
                backend="torch", 
                sim_params=None, 
                init_sim=True, 
                np_array_dtype = np.float32, 
                verbose = False, 
                debug = False) -> None:

        super().set_task(task, 
                backend=backend, 
                sim_params=sim_params, 
                init_sim=init_sim)
        
        self.robot_names = self.task.robot_names
        self.robot_pkg_names = self.task.robot_pkg_names
        self.cluster_clients = {}

        # now the task and the simulation is guaranteed to be initialized
        # -> we have the data to initialize the cluster client
        for i in range(len(self.robot_names)):

            self.cluster_clients[self.robot_names[i]] = KyonRHClusterClient(cluster_size=task.num_envs, 
                            device=task.torch_device, 
                            cluster_dt=task.cluster_dt, 
                            control_dt=task.integration_dt, 
                            jnt_names = task.robot_dof_names[self.robot_names[i]], 
                            np_array_dtype = np_array_dtype, 
                            verbose = verbose, 
                            debug = debug, 
                            robot_name=self.robot_names[i])
        
        self.init_jnt_cmd_to_safe_vals()

        self._is_cluster_ready = False
        
    def step(self, 
        index: int, 
        actions = None):
        
        for i in range(len(self.robot_names)):

            if self.cluster_clients[self.robot_names[i]].is_first_control_step():
                
                # first time the cluster is ready (i.e. the controllers are ready and connected)

                self.task.init_root_abs_offsets(self.robot_names[i]) # we get the current absolute positions and use them as 
                # references

                self.task.synch_default_root_states() # we update the default root state now, so that we
                # can use it at the next call to reset

            if self.cluster_clients[self.robot_names[i]].is_cluster_instant(index):
                
                # assign last robot state observation to the cluster client
                self.update_cluster_state(self.robot_names[i])

                # the control cluster may run at a different rate wrt the simulation

                self.cluster_clients[self.robot_names[i]].solve() # we solve all the underlying TOs in the cluster
                # (the solve will do nothing unless the cluster is ready)

                print(f"[{self.__class__.__name__}]" + f"[{self.journal.info}]: " + \
                    "cluster client n." + str(i) + " solve time -> " + \
                    str(self.cluster_clients[self.robot_names[i]].solution_time))
                
            if self.cluster_clients[self.robot_names[i]].cluster_ready() and \
                self.cluster_clients[self.robot_names[i]].controllers_active:
                
                if self.cluster_clients[self.robot_names[i]].is_first_control_step():

                    gains_pos = torch.full((self.num_envs, self.robot_n_dofs), 
                                100.0, 
                                device = self.torch_device, 
                                dtype=self.torch_dtype)
                    gains_vel = torch.full((self.num_envs, self.robot_n_dofs), 
                                10, 
                                device = self.torch_device, 
                                dtype=self.torch_dtype)
                    self.jnt_imp_controllers[self.robot_names[i]].set_gains(
                                        pos_gains = gains_pos,
                                        vel_gains = gains_vel)
                    
                    wheels_indxs = self.jnt_imp_controllers[self.robot_names[i]].get_jnt_idxs_matching(
                                            name_pattern="wheel")
                    wheels_pos_gains = torch.full((self.num_envs, len(wheels_indxs)), 
                                                0.0, 
                                                device = self.torch_device, 
                                                dtype=self.torch_dtype)
                    
                    wheels_vel_gains = torch.full((self.num_envs, len(wheels_indxs)), 
                                                10.0, 
                                                device = self.torch_device, 
                                                dtype=self.torch_dtype)
                    
                    self.jnt_imp_controllers[self.robot_names[i]].set_gains(
                                    pos_gains = wheels_pos_gains,
                                    vel_gains = wheels_vel_gains,
                                    jnt_indxs=wheels_indxs)
                
                self.task.pre_physics_step(robot_name = self.robot_names[i], 
                                actions = self.cluster_clients[self.robot_names[i]].controllers_cmds)
                
            else:

                self.task.pre_physics_step(robot_name = self.robot_names[i],
                                actions = None)
                
        self._world.step(render=self._render)
        
        self.sim_frame_count += 1

        observations = self.task.get_observations()

        rewards = self.task.calculate_metrics()

        dones = self.task.is_done()

        info = {}

        return observations, rewards, dones, info
        
    def reset(self):

        self._world.reset()

        self.task.reset()
        
        self._world.step(render=self._render)

        observations = self.task.get_observations()

        self.init_jnt_cmd_to_safe_vals()

        return observations
    
    def update_cluster_state(self, 
                        robot_name: str):

        self.cluster_clients[robot_name].robot_states.root_state.p[:, :] = torch.sub(self.task.root_p[robot_name], 
                                                                self.task.root_abs_offsets[robot_name]) # we only get the relative position
        # w.r.t. the initial spawning pose
        self.cluster_clients[robot_name].robot_states.root_state.q[:, :] = self.task.root_q[robot_name]
        self.cluster_clients[robot_name].robot_states.root_state.v[:, :] = self.task.root_v[robot_name]
        self.cluster_clients[robot_name].robot_states.root_state.omega[:, :] = self.task.root_omega[robot_name]

        self.cluster_clients[robot_name].robot_states.jnt_state.q[:, :] = self.task.jnts_q[robot_name]
        self.cluster_clients[robot_name].robot_states.jnt_state.v[:, :] = self.task.jnts_v[robot_name]

    def init_jnt_cmd_to_safe_vals(self):
        
        for i in range(len(self.robot_names)):
            
            robot_name = self.robot_names[i]

            self.task.jnt_imp_controllers[robot_name].set_refs(
                            pos_ref = self.task.homers[robot_name].get_homing())
            self.task.jnt_imp_controllers[robot_name].apply_refs()

        # self.cluster_client.controllers_cmds.jnt_cmd.v = 
        # self.cluster_client.controllers_cmds.jnt_cmd.eff = 

    def close(self):

        for i in range(len(self.robot_names)):

            self.cluster_clients[self.robot_names[i]].close()
        
        super().close() # this has to be called last 
        # so that isaac's simulation is close properly

        