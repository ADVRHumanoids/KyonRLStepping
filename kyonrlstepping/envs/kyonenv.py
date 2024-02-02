from omni_robo_gym.gym.omni_vect_env.vec_envs import RobotVecEnv
from omni_robo_gym.utils.math_utils import quat_to_omega
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonRHClusterClient

from SharsorIPCpp.PySharsorIPC import VLevel, Journal, LogType

from typing import List

import torch 
import numpy as np

import time

class KyonEnv(RobotVecEnv):

    def __init__(self,
                headless: bool = True, 
                sim_device: int = 0, 
                enable_livestream: bool = False, 
                enable_viewport: bool = False,
                debug = False):

        super().__init__(headless = headless, 
                sim_device = sim_device, 
                enable_livestream = enable_livestream, 
                enable_viewport = enable_viewport,
                debug = debug)

        # debug data
        self.debug_data = {}
        self.debug_data["time_to_step_world"] = np.nan
        self.debug_data["time_to_get_agent_data"] = np.nan
        self.debug_data["cluster_sol_time"] = {}
        self.debug_data["cluster_state_update_dt"] = {}

        self.cluster_timers = {}
        self.env_timer = time.perf_counter()

        self.step_counter = 0
        self.cluster_clients = {}
        self._trigger_cluster = {}
        self._cluster_dt = {}

    def set_task(self, 
                task, 
                cluster_dt: List[float], 
                backend="torch", 
                sim_params=None, 
                init_sim=True, 
                np_array_dtype = np.float32, 
                cluster_client_verbose = False, 
                cluster_client_debug = False) -> None:

        super().set_task(task, 
                backend=backend, 
                sim_params=sim_params, 
                init_sim=init_sim)
        
        self.robot_names = self.task.robot_names
        self.robot_pkg_names = self.task.robot_pkg_names
        
        if not isinstance(cluster_dt, List):
            
            exception = "cluster_dt must be a list!"

            Journal.log(self.__class__.__name__,
                "set_task",
                exception,
                LogType.EXCEP,
                throw_when_excep = True)

        if not (len(cluster_dt) == len(self.robot_names)):

            exception = f"cluster_dt length{len(cluster_dt)} does not match robot number {len(self.robot_names)}!"

            Journal.log(self.__class__.__name__,
                "set_task",
                exception,
                LogType.EXCEP,
                throw_when_excep = True)
        
        # now the task and the simulation is guaranteed to be initialized
        # -> we have the data to initialize the cluster client
        for i in range(len(self.robot_names)):
            
            robot_name = self.robot_names[i]

            if not isinstance(cluster_dt[i], float):

                exception = f"cluster_dt should be a list of float values!"

                Journal.log(self.__class__.__name__,
                    "set_task",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)
            
            self._cluster_dt[robot_name] = cluster_dt[i]

            self._trigger_cluster[robot_name] = True # allow first trigger

            if robot_name in task.omni_contact_sensors:

                n_contact_sensors = task.omni_contact_sensors[robot_name].n_sensors
                contact_names = task.omni_contact_sensors[robot_name].contact_prims
            
            else:
                
                n_contact_sensors = -1
                contact_names = None

            self.cluster_clients[robot_name] = KyonRHClusterClient(cluster_size=task.num_envs, 
                        device=task.torch_device, 
                        cluster_dt=self._cluster_dt[robot_name], 
                        control_dt=task.integration_dt(), 
                        jnt_names = task.robot_dof_names[robot_name], 
                        n_contact_sensors = n_contact_sensors,
                        contact_linknames = contact_names, 
                        np_array_dtype = np_array_dtype, 
                        verbose = cluster_client_verbose, 
                        debug = cluster_client_debug, 
                        robot_name=robot_name)
            
            self.debug_data["cluster_sol_time"][robot_name] = np.nan
            self.debug_data["cluster_state_update_dt"][robot_name] = np.nan

            if self.debug:

                self.cluster_timers[robot_name] = time.perf_counter()
                        
        self.using_gpu = task.using_gpu
    
    def close(self):

        for i in range(len(self.robot_names)):

            self.cluster_clients[self.robot_names[i]].close()
        
        self.task.close() # performs closing steps for task

        super().close() # this has to be called last 
        # so that isaac's simulation is close properly

    def step(self, 
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
            
            robot_name = self.robot_names[i]

            # running cluster client if not already running
            if not self.cluster_clients[robot_name].is_running():
                
                self.cluster_clients[robot_name].run()

            # 1) this runs at a dt = cluster_clients[robot_name] dt (sol. triggering) 
            if self.cluster_clients[robot_name].is_cluster_instant(self.step_counter) and \
                self._trigger_cluster[robot_name]:

                just_activated = self.cluster_clients[robot_name].get_transitioned_controllers()

                if just_activated is not None:
                    
                    # transition from controllers stopped to active -->
                    # we perform some itialization steps (jnt imp. controllers includes)

                    # we get the current absolute positions for the transitioned controllers and 
                    # use them as references
                    self.task.update_root_offsets(robot_name,
                                    env_indxs = just_activated) 

                    # we update the default root state now, so that we
                    # can use it at the next call to reset
                    self.task.synch_default_root_states(robot_name = robot_name,
                                    env_indxs = just_activated)

                    # we initialize vals of the state for the activated controllers
                    self._update_cluster_state(robot_name = robot_name, 
                                    env_indxs = just_activated)

                    # some controllers transitioned to running state    
                    self.task.update_jnt_imp_control(robot_name = robot_name, 
                                    jnt_stiffness = self.task.startup_jnt_stiffness, 
                                    jnt_damping = self.task.startup_jnt_damping, 
                                    wheel_stiffness = self.task.startup_wheel_stiffness, 
                                    wheel_damping = self.task.startup_wheel_damping,
                                    env_indxs = just_activated)

                # every control_cluster_dt, trigger the solution of the active controllers in the cluster
                # with the latest available state
                self.cluster_clients[robot_name].trigger_solution()

            # 2) this runs at a dt = simulation dt i.e. the highest possible rate,
            #    using the latest available RHC solution (the new one is not available yet)
            # (sets low level cmds to jnt imp controller)
            active = self.cluster_clients[robot_name].get_transitioned_controllers()

            self.task.pre_physics_step(robot_name = robot_name, 
                            actions = self.cluster_clients[robot_name].get_actions(),
                            env_indxs = active) # applies updated rhc actions to low-level
            # joint imp. control only for active controllers     
        
        # 3) simulation stepping (@ integration_dt) (for all robots and all environments)
        self._step_world()

        for i in range(len(self.robot_names)):
            
            robot_name = self.robot_names[i]

            # this runs at a dt = control_cluster dt
            if self.cluster_clients[robot_name].is_cluster_instant(self.step_counter):
            
                if not self._trigger_cluster[robot_name]:                        
            
                    # we reach the next control instant -> we get the solution
                    # we also reset the flag, so next call to step() will trigger again the
                    # cluster
                
                    # 3) wait for solution (will also read latest computed cmds)
                    self.cluster_clients[robot_name].wait_for_solution() # this is blocking
                    
                    self._trigger_cluster[robot_name] = True # this allows for the next trigger 

                    # 4) update cluster state
                    if self.debug:

                        self.cluster_timers[robot_name]  = time.perf_counter()

                    # update cluster state 
                    active_controllers = self.cluster_clients[robot_name].get_active_controllers()
                    self._update_cluster_state(robot_name = robot_name, 
                                    env_indxs = active_controllers)
                    
                    if self.debug:

                        self.debug_data["cluster_state_update_dt"][robot_name] = \
                            time.perf_counter() - self.cluster_timers[robot_name]
                        
                        self.debug_data["cluster_sol_time"][robot_name] = \
                            self.cluster_clients[robot_name].solution_time
                        
                else: # we are in the same step() call as the cluster trigger

                    self._trigger_cluster[robot_name] = False # -> next cluster instant we get/wait the solution
                    # from the cluster                

        self.step_counter += 1

        # RL stuff
        if self.debug:
            
            self.env_timer = time.perf_counter()
            
        observations = self.task.get_observations()

        rewards = self.task.calculate_metrics()

        dones = self.task.is_done()

        info = {}

        if self.debug:
                        
            self.debug_data["time_to_get_agent_data"] = time.perf_counter() - self.env_timer

        return observations, rewards, dones, info
    
    def reset(self,
            env_ids: List[int]=None,
            robot_names: List[str]=None,
            reset_world: bool = False):

        if reset_world:

            self._world.reset()

        self.task.reset(env_ids = env_ids,
            robot_names = robot_names)
        
        # perform a simulation step
        self._world.step(render=self._render)

        return None
    
    def _step_world(self):

        if self.debug:
            
            self.env_timer = time.perf_counter()

        self._world.step(render=self._render)

        if self.debug:
            
            self.debug_data["time_to_step_world"] = time.perf_counter() - self.env_timer

    def _update_contact_state(self, 
                    robot_name: str, 
                    env_indxs: torch.Tensor = None):

        for i in range(0, self.cluster_clients[robot_name].n_contact_sensors):
            
            contact_link = self.cluster_clients[robot_name].contact_linknames[i]
            
            if not self.using_gpu:
                
                f_contact = self.task.omni_contact_sensors[robot_name].get(dt = self.task.integration_dt(), 
                                                            contact_link = contact_link,
                                                            env_indxs = env_indxs,
                                                            clone = False)
                # assigning measured net contact forces
                self.cluster_clients[robot_name].get_state().contact_wrenches.set_f_contact(f=f_contact,
                                            contact_name=contact_link,
                                            robot_idxs = env_indxs,
                                            gpu=self.using_gpu)
                    
            else:
                
                if ((self.step_counter + 1) % 10000) == 0:
                    
                    # sporadic warning
                    warning = f"[{self.__class__.__name__}]" + f"[{self.journal.warning}]: " + \
                        f"Contact state from link {contact_link} cannot be retrieved in IsaacSim if using use_gpu_pipeline is set to True!"

                    print(warning)

    def _update_cluster_state(self, 
                    robot_name: str, 
                    env_indxs: torch.Tensor = None):
        
        if not isinstance(env_indxs, torch.Tensor):
            
            msg = "Provided env_indxs should be a torch tensor of indexes!"
        
            raise Exception(f"[{self.__class__.__name__}]" + f"[{self._journal.exception}]: " + msg)
        
        if not len(env_indxs.shape) == 1:

            msg = "Provided env_indxs should be a 1D torch tensor!"
        
            raise Exception(f"[{self.__class__.__name__}]" + f"[{self._journal.exception}]: " + msg)

        # floating base
        relative_pos = torch.sub(self.task.root_p(robot_name=robot_name,
                                            env_idxs=env_indxs), 
                                self.task.root_offsets(robot_name=robot_name, 
                                                        env_idxs=env_indxs))

        self.cluster_clients[robot_name].get_state().root_state.set_p(p = relative_pos,
                                                            robot_idxs = env_indxs,
                                                            gpu = self.using_gpu) # we only set the relative position
        # w.r.t. the initial spawning pose
        self.cluster_clients[robot_name].get_state().root_state.set_q(q = self.task.root_q(robot_name=robot_name,
                                                                                            env_idxs=env_indxs),
                                                                robot_idxs = env_indxs,
                                                                gpu = self.using_gpu)

        self.cluster_clients[robot_name].get_state().root_state.set_v(v=self.task.root_v(robot_name=robot_name,
                                                                                            env_idxs=env_indxs),
                                                                robot_idxs = env_indxs,
                                                                gpu = self.using_gpu) 

        self.cluster_clients[robot_name].get_state().root_state.set_omega(gpu = self.using_gpu,
                                                                    robot_idxs = env_indxs,
                                                                    omega=self.task.root_omega(robot_name=robot_name,
                                                                                            env_idxs=env_indxs)) 

        # joints
        self.cluster_clients[robot_name].get_state().jnts_state.set_q(q=self.task.jnts_q(robot_name=robot_name,
                                                                                            env_idxs=env_indxs), 
                                                                    robot_idxs = env_indxs,
                                                                    gpu = self.using_gpu)

        self.cluster_clients[robot_name].get_state().jnts_state.set_v(v=self.task.jnts_v(robot_name=robot_name,
                                                                                            env_idxs=env_indxs),
                                                                    robot_idxs = env_indxs,
                                                                    gpu = self.using_gpu) 

        # Updating contact state for selected contact links
        self._update_contact_state(robot_name=robot_name,
                            env_indxs=env_indxs)
