from omni_custom_gym.gym.omni_vect_env.vec_envs import RobotVecEnv

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

class KyonEnv(RobotVecEnv):

    def set_task(self, 
                task, 
                backend="torch", 
                sim_params=None, 
                init_sim=True) -> None:

        super().set_task(task, 
                backend=backend, 
                sim_params=sim_params, 
                init_sim=init_sim)
        
        # now the task and the simulation is guaranteed to be initialized
        # -> we have the data to initialize the cluster client
        self.cluster_client = KyonRHClusterClient(cluster_size=task.num_envs, 
                        device=task.torch_device, 
                        cluster_dt=task.cluster_dt, 
                        control_dt=task.integration_dt, 
                        jnt_names = task.robot_dof_names)
        
        self.init_cluster_cmd_to_safe_vals()
        
    def step(self, 
        index: int, 
        actions = None):

        # assign last robot state observation to the cluster client
        self.update_cluster_state()

        if self.cluster_client.is_cluster_instant(index):
            
            # the control cluster may run at a different rate wrt the simulation

            self.cluster_client.solve() # we solve all the underlying TOs in the cluster

            print(f"[{self.__class__.__name__}]" + f"[{self.info}]: " + \
                "cumulative cluster solution time -> " + \
                str(self.cluster_client.solution_time))
        
        self.task.pre_physics_step(self.cluster_client.controllers_cmds)

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

        self.init_cluster_cmd_to_safe_vals()

        return observations
    
    def update_cluster_state(self):

        self.cluster_client.robot_states.root_state.p = self.task.root_p
        self.cluster_client.robot_states.root_state.q = self.task.root_q
        self.cluster_client.robot_states.root_state.v = self.task.root_v
        self.cluster_client.robot_states.root_state.omega = self.task.root_omega
        self.cluster_client.robot_states.jnt_state.q = self.task.jnts_q
        self.cluster_client.robot_states.jnt_state.v = self.task.jnts_v

    def init_cluster_cmd_to_safe_vals(self):

        self.cluster_client.controllers_cmds.jnt_cmd.q = self.task._homer.get_homing()

        # self.cluster_client.controllers_cmds.jnt_cmd.v = 
        # self.cluster_client.controllers_cmds.jnt_cmd.eff = 
