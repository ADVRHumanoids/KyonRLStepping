from omni_robo_gym.gym.omni_vect_env.vec_envs import RobotVecEnv
from omni_robo_gym.utils.math_utils import quat_to_omega
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

import torch 
import numpy as np

class KyonEnv(RobotVecEnv):

    def set_task(self, 
                task, 
                cluster_dt: float, 
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
        self.cluster_dt = cluster_dt

        # now the task and the simulation is guaranteed to be initialized
        # -> we have the data to initialize the cluster client
        for i in range(len(self.robot_names)):
            
            if self.robot_names[i] in task.omni_contact_sensors:

                n_contact_sensors = task.omni_contact_sensors[self.robot_names[i]].n_sensors
                contact_names = task.omni_contact_sensors[self.robot_names[i]].contact_prims
            
            else:
                
                n_contact_sensors = -1
                contact_names = None

            self.cluster_clients[self.robot_names[i]] = KyonRHClusterClient(cluster_size=task.num_envs, 
                        device=task.torch_device, 
                        cluster_dt=self.cluster_dt, 
                        control_dt=task.integration_dt, 
                        jnt_names = task.robot_dof_names[self.robot_names[i]], 
                        n_contact_sensors = n_contact_sensors,
                        contact_linknames = contact_names, 
                        np_array_dtype = np_array_dtype, 
                        verbose = False, 
                        debug = debug, 
                        robot_name=self.robot_names[i])
        
        self._is_cluster_ready = False
        self._trigger_solution = True
        
        self.controllers_were_active = False
    
    def step(self, 
        index: int, 
        actions = None):
        
        # Stepping order explained:
        # 1) we trigger the solution of the cluster, which will employ the previous state (i.e.
        #    the controller applies an action which is delayed of a cluster_dt wrt to the state it
        #    used -> this is realistic, since between state retrieval and action computation there 
        #    will always be at least a cluster_dt delay
        # 2) we then proceed to set the latest available cluster cmd to 
        #    the low-level impedance controller
        #    (the reference will actually change every cluster_dt/integration_dt steps)-> controllers in the cluster, in the meantime, 
        #    are solving in parallel between them and wrt to simulation stepping. Aside from being
        #    much more realistic, this also avoids serial evaluation of the controllers and the
        #    sim. stepping, which can cause significant rt-factor degradation.
        # 3) we step the simulation
        # 4) as soon as the simulation was stepped, we check the cluster solution status and
        #    wait for the solution if necessary. This means that the bottlneck of the step()
        #    will be the slowest between simulation stepping and cluster solution.
        # 5) we update the cluster state with the one reached after the sim stepping
        
        for i in range(len(self.robot_names)):
            
            # first time the cluster is ready (i.e. the controllers are ready and connected)
            if self.cluster_clients[self.robot_names[i]].is_first_control_step():
            
                # we get the current absolute positions and use them as 
                # references
                self.task.init_root_abs_offsets(self.robot_names[i]) 

                # we update the default root state now, so that we
                # can use it at the next call to reset
                self.task.synch_default_root_states()

                # we initialize vals of the state for the cluster
                self.update_cluster_state(self.robot_names[i], index)

            # 1) this runs at a dt = control_cluster dt (sol. triggering) + 
            if self.cluster_clients[self.robot_names[i]].is_cluster_instant(index) and \
                self._trigger_solution:

                # every control_cluster_dt, trigger the solution of the cluster
                # with the latest available state. trigger_solution() will also 
                # perform runtime checks to ensure the cluster is ready and active
                self.cluster_clients[self.robot_names[i]].trigger_solution() # this is non-blocking

            # 2) this runs at a dt = simulation dt i.e. the highest possible rate,
            #    using the latest available RHC solution (the new one is not available yet)
            if self.cluster_clients[self.robot_names[i]].cluster_ready() and \
                self.cluster_clients[self.robot_names[i]].controllers_active:
                
                if not self.controllers_were_active:
                    
                    # transition from controllers stopped to active -->
                    # we set the joint impedance controllers to the desired runtime state 

                    self.task.update_jnt_imp_control(robot_name = self.robot_names[i], 
                                    jnt_stiffness = self.task.startup_jnt_stiffness, 
                                    jnt_damping = self.task.startup_jnt_damping, 
                                    wheel_stiffness = self.task.startup_wheel_stiffness, 
                                    wheel_damping =self.task.startup_wheel_damping)

                self.task.pre_physics_step(robot_name = self.robot_names[i], 
                                actions = self.cluster_clients[self.robot_names[i]].controllers_cmds)
                
                self.controllers_were_active = self.cluster_clients[self.robot_names[i]].controllers_active

            else:
                
                # either cluster is not ready yet (initializing) or the RHC controllers
                # are not active yet

                self.task.pre_physics_step(robot_name = self.robot_names[i],
                                actions = None)
            
        # 3) simulation stepping # integration_dt
        self._world.step(render=self._render)

        # this runs at a dt = control_cluster dt

        if self.cluster_clients[self.robot_names[i]].is_cluster_instant(index):

            if not self._trigger_solution:
                        
                for i in range(len(self.robot_names)):
                    
                    # we reach the next control instant -> we get the solution
                    # we also reset the flag, so next call to step() will trigger again the
                    # cluster
                
                    # 3) wait for solution (will also read latest computed cmds)
                    self.cluster_clients[self.robot_names[i]].wait_for_solution() # this is blocking
                    
                    self._trigger_solution = True # this allows for the next trigger 

                    # 4) update cluster state
                    self.update_cluster_state(self.robot_names[i], index)

            else: # we are in the same step() call as the trigger

                self._trigger_solution = False # -> next cluster instant we get the solution
                # from the cluster                

        self.sim_frame_count += 1

        # RL stuff
        observations = self.task.get_observations()

        rewards = self.task.calculate_metrics()

        dones = self.task.is_done()

        info = {}

        return observations, rewards, dones, info
        
    def reset(self):

        self._world.reset()

        self.task.reset(self.task.integration_dt)
        
        self._world.step(render=self._render)

        return None
    
    def update_cluster_state(self, 
                        robot_name: str, 
                        step_index: int = -1):
        
        self.cluster_clients[robot_name].robot_states.root_state.p[:, :] = torch.sub(self.task.root_p[robot_name], 
                                                                self.task.root_abs_offsets[robot_name]) # we only get the relative position
        # w.r.t. the initial spawning pose
        self.cluster_clients[robot_name].robot_states.root_state.q[:, :] = self.task.root_q[robot_name]
        self.cluster_clients[robot_name].robot_states.root_state.v[:, :] = self.task.root_v[robot_name]
        self.cluster_clients[robot_name].robot_states.root_state.omega[:, :] = self.task.root_omega[robot_name]

        self.cluster_clients[robot_name].robot_states.jnt_state.q[:, :] = self.task.jnts_q[robot_name]
        self.cluster_clients[robot_name].robot_states.jnt_state.v[:, :] = self.task.jnts_v[robot_name]

        # contact state

        # for each contact link
        
        for i in range(0, self.cluster_clients[robot_name].n_contact_sensors):
            
            contact_link = self.cluster_clients[robot_name].contact_linknames[i]
            
            if not self.gpu_pipeline_enabled:

                # assigning measured net contact forces
                self.cluster_clients[robot_name].contact_states.contact_state.get(contact_link)[:, :] = \
                    self.task.omni_contact_sensors[robot_name].get(dt = self.task.integration_dt, 
                                                            contact_link = contact_link,
                                                            clone = False)

            else:
                
                if ((step_index + 1) % 10000) == 0:
                    
                    # sporadic warning
                    warning = f"[{self.__class__.__name__}]" + f"[{self.journal.warning}]: " + \
                        f"Contact state from link {contact_link} cannot be retrieved in IsaacSim if using use_gpu_pipeline is set to True!"

                    print(warning)

    def close(self):

        for i in range(len(self.robot_names)):

            self.cluster_clients[self.robot_names[i]].close()
        
        self.task.terminate() # performs closing steps for task

        super().close() # this has to be called last 
        # so that isaac's simulation is close properly

        