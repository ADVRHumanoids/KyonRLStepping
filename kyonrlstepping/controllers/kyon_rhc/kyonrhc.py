from control_cluster_bridge.controllers.rhc import RHController
from control_cluster_bridge.utilities.homing import RobotHomer

from kyonrlstepping.controllers.kyon_rhc.horizon_imports import * 

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_taskref import KyonRhcTaskRef
from kyonrlstepping.controllers.kyon_rhc.gait_manager import GaitManager

from kyonrlstepping.utils.rhc2shared import RHC2SharedInternal

import numpy as np

import torch

class KyonRHC(RHController):

    def __init__(self, 
            controller_index: int,
            srdf_path: str,
            urdf_path: str,
            config_path: str,
            cluster_size: int, # needed by shared mem manager
            robot_name: str = "kyon0",
            n_intervals:float = 25,
            dt: float = 0.02,
            max_solver_iter = 1, # defaults to rt-iteration
            add_data_lenght: int = 2,
            enable_replay = False, 
            verbose = False, 
            debug = False, 
            array_dtype = torch.float32, 
            publish_sol = False,
            debug_sol = True # whether to publish rhc rebug data
            ):

        self.step_counter = 0
        self.sol_counter = 0

        self.max_solver_iter = max_solver_iter
    
        self.publish_sol = publish_sol

        self.robot_name = robot_name
        
        self._enable_replay = enable_replay
        
        self._n_intervals = n_intervals
        self._dt = dt
        self._t_horizon = self._n_intervals * self._dt

        self.config_path = config_path

        self.urdf_path = urdf_path
        # read urdf and srdf files
        with open(self.urdf_path, 'r') as file:

            self.urdf = file.read()

        super().__init__(controller_index = controller_index, 
                        cluster_size = cluster_size,
                        srdf_path = srdf_path,
                        n_nodes = self._n_intervals + 1,
                        namespace = self.robot_name,
                        verbose = verbose, 
                        debug = debug,
                        array_dtype = array_dtype,
                        debug_sol = debug_sol)

        self.add_data_lenght = add_data_lenght # length of the array holding additional info from the solver

        self._quat_remap = [1, 2, 3, 0] # mapping from robot quat. to Horizon's quaternion convention

        self.rhc_costs={}
        self.rhc_constr={}

        if self.publish_sol:
            
            # object to forward individual internal rhc state to shared memery
            # for external debugging
            self.rhc2shared_bridge = RHC2SharedInternal( 
                                        namespace=self.robot_name,
                                        index=self.controller_index,
                                        n_jnts=self.n_dofs,
                                        n_rhc_nodes=self._n_intervals,
                                        verbose=self._verbose,
                                        basename="RHC2SharedInternal")
            
            self.rhc2shared_bridge.run()
        
        else:

            self.rhc2shared_bridge = None
            
    def _init_problem(self):
        
        print(f"[{self.__class__.__name__}" + str(self.controller_index) + "]" + \
                f"[{self.journal.status}]" + ": initializing RHC problem " + \
                f"with dt: {self._dt} s, t_horizon: {self._t_horizon} s, n_intervals: {self._n_intervals}")

        self.urdf = self.urdf.replace('continuous', 'revolute') # continous joint is parametrized
        # in So2, so will add 

        self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self._assign_server_side_jnt_names(self._get_robot_jnt_names())

        self._prb = Problem(self._n_intervals, 
                        receding=True, 
                        casadi_type=cs.SX)
        self._prb.setDt(self._dt)

        self._homer = RobotHomer(srdf_path=self.srdf_path, 
                            jnt_names_prb=self._server_side_jnt_names)

        import numpy as np
        base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        init = base_init.tolist() + list(self._homer.get_homing())

        FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
        
        kyon_wheel_radius = 0.124 # hardcoded!!!!

        init_pos_foot = FK(q=init)['ee_pos']
        base_init[2] = -init_pos_foot[2] + kyon_wheel_radius

        self._model = FullModelInverseDynamics(problem=self._prb,
                                kd=self._kin_dyn,
                                q_init=self._homer.get_homing_map(),
                                base_init=base_init)
        
        self._ti = TaskInterface(prb=self._prb, 
                            model=self._model, 
                            max_solver_iter=self.max_solver_iter,
                            debug = self._debug, 
                            verbose = self._verbose)
        
        self._ti.setTaskFromYaml(self.config_path)

        # setting initial CoM ref, so that it's coherent
        CoM = self._kin_dyn.centerOfMass()
        init_CoM_pos = CoM(q=init)['com']
        CoM_pose = self._ti.getTask('CoM_pose')
        CoM_tmp = base_init.copy()
        CoM_tmp[2] = init_CoM_pos[2] + base_init[2]
        CoM_pose.setRef(np.atleast_2d(CoM_tmp).T)

        self._tg = trajectoryGenerator.TrajectoryGenerator()

        self._pm = pymanager.PhaseManager(self._n_intervals)

        # adding timelines
        c_phases = dict()
        for c in self._model.cmap.keys():

            c_phases[c] = self._pm.addTimeline(f'{c}_timeline')

        stance_duration_default = 5
        flight_duration_default = 5

        for c in self._model.cmap.keys():

            # stance phase normal
            stance_phase = pyphase.Phase(stance_duration_default, f'stance_{c}')

            # add contact constraints to phase
            if self._ti.getTask(f'{c}_contact') is not None:

                stance_phase.addItem(self._ti.getTask(f'{c}_contact'))

            else:

                raise Exception(f"[{self.__class__.__name__}]" + \
                                f"[{self.journal.exception}]" + \
                                ": task not found")

            # register phase to timeline
            c_phases[c].registerPhase(stance_phase)

            # flight phase normal
            flight_phase = pyphase.Phase(flight_duration_default, f'flight_{c}')

            init_z_foot = self._model.kd.fk(c)(q=self._model.q0)['ee_pos'].elements()[2]

            ref_trj = np.zeros(shape=[7, flight_duration_default])
            
            # trajectory on z
            ref_trj[2, :] = np.atleast_2d(self._tg.from_derivatives(flight_duration_default, 
                                                        init_z_foot, 
                                                        init_z_foot, 
                                                        0.1, 
                                                        [0, 0, 0]))
            
            if self._ti.getTask(f'z_{c}') is not None:

                flight_phase.addItemReference(self._ti.getTask(f'z_{c}'), ref_trj)
                
            else:
                 
                raise Exception(f"[{self.__class__.__name__}]" + f"[{self.journal.exception}]" + f": task {c}_contact not found")
            
            # flight_phase.addConstraint(prb.getConstraints(f'{c}_vert'), nodes=[0 ,flight_duration-1])  # nodes=[0, 1, 2]
            c_phases[c].registerPhase(flight_phase)

        
        for c in self._model.cmap.keys():
            stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
            # flight = c_phases[c].getRegisteredPhase(f'flight_{c}')
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)
            c_phases[c].addPhase(stance)

        self._ti.model.q.setBounds(self._ti.model.q0, self._ti.model.q0, nodes=0)
        self._ti.model.v.setBounds(self._ti.model.v0, self._ti.model.v0, nodes=0)
        # ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
        self._ti.model.q.setInitialGuess(self._ti.model.q0)
        self._ti.model.v.setInitialGuess(self._ti.model.v0)

        f0 = [0, 0, self._kin_dyn.mass() / 4 * 9.8]
        for _, cforces in self._ti.model.cmap.items():
            for c in cforces:
                c.setInitialGuess(f0)

        self._ti.finalize()

        self._ti.bootstrap()

        self._ti.init_inv_dyn_for_res() # we initialize some objects for sol. postprocessing purposes

        self._ti.load_initial_guess()

        contact_phase_map = {c: f'{c}_timeline' for c in self._model.cmap.keys()}
        
        self._gm = GaitManager(self._ti, self._pm, contact_phase_map)

        self.n_dofs = self._get_ndofs() # after loading the URDF and creating the controller we
        # know n_dofs -> we assign it (by default = None)

        self.n_contacts = len(self._model.cmap.keys())
        
        # self.horizon_anal = analyzer.ProblemAnalyzer(self._prb)

        print(f"[{self.__class__.__name__}" + str(self.controller_index) + "]" +  f"[{self.journal.status}]" + "Initialized RHC problem")

    def _init_rhc_task_cmds(self) -> KyonRhcTaskRef:

        return KyonRhcTaskRef(gait_manager=self._gm, 
                        n_contacts=len(self._model.cmap.keys()), 
                        index=self.controller_index, 
                        q_remapping=self._quat_remap, 
                        dtype=self.array_dtype, 
                        verbose=self._verbose, 
                        namespace=self.robot_name)
    
    def _get_robot_jnt_names(self):

        joints_names = self._kin_dyn.joint_names()

        to_be_removed = ["universe", 
                        "reference", 
                        "world", 
                        "floating", 
                        "floating_base"]
        
        for name in to_be_removed:

            if name in joints_names:
                joints_names.remove(name)

        return joints_names
    
    def _get_ndofs(self):
        
        return len(self._model.joint_names)

    def _get_cmd_jnt_q_from_sol(self):
        
        # wrapping joint q commands between 2pi and -2pi
        # (to be done for the simulator)
        return torch.tensor(self._ti.solution['q'][7:, 1], 
                        dtype=self.array_dtype).reshape(1,  
                                        self.robot_cmds.jnt_cmd.q.shape[1]).fmod(2 * torch.pi)
    
    def _get_cmd_jnt_v_from_sol(self):

        return torch.tensor(self._ti.solution['v'][6:, 1], 
                        dtype=self.array_dtype).reshape(1, 
                                        self.robot_cmds.jnt_cmd.v.shape[1])

    def _get_cmd_jnt_eff_from_sol(self):
        
        efforts_on_first_node = self._ti.eval_efforts_on_first_node()

        return torch.tensor(efforts_on_first_node[6:, 0], 
                        dtype=self.array_dtype).reshape(1, 
                self.robot_cmds.jnt_cmd.eff.shape[1])
    
    def _get_additional_slvr_info(self):

        return torch.tensor([self._ti.solution["opt_cost"], 
                            self._ti.solution["n_iter2sol"]], 
                        dtype=self.array_dtype)
    
    def _assemble_meas_robot_state(self, 
                        to_numpy: bool = False):

        if to_numpy:
            
            return torch.cat((self.robot_state.root_state.get_p(), 
                    self.robot_state.root_state.get_q(), 
                    self.robot_state.jnt_state.get_q(), 
                    self.robot_state.root_state.get_v(), 
                    self.robot_state.root_state.get_omega(), 
                    self.robot_state.jnt_state.get_v()), 
                    dim=1
                    ).numpy().T
        
        if not to_numpy:

            return torch.cat((self.robot_state.root_state.get_p(), 
                    self.robot_state.root_state.get_q(), 
                    self.robot_state.jnt_state.get_q(), 
                    self.robot_state.root_state.get_v(), 
                    self.robot_state.root_state.get_omega(), 
                    self.robot_state.jnt_state.get_v()), 
                    dim=1
                    ).T

        return 
    
    def _assemble_meas_robot_configuration(self, 
                        to_numpy: bool = False):

        if to_numpy:
            
            return torch.cat((self.robot_state.root_state.get_p(), 
                    self.robot_state.root_state.get_q(), 
                    self.robot_state.jnt_state.get_q()), 
                    dim=1
                    ).numpy().T
        
        if not to_numpy:

            return torch.cat((self.robot_state.root_state.get_p(), 
                    self.robot_state.root_state.get_q(), 
                    self.robot_state.jnt_state.get_q()), 
                    dim=1
                    ).T

        return 
    
    def _update_open_loop(self):

        shift_num = -1 # shift data by one node

        # building ig for state
        xig = np.roll(self._ti.solution['x_opt'], 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            xig[:, -1 - i] = self._ti.solution['x_opt'][:, -1]

        # building ig for inputs
        uig = np.roll(self._ti.solution['u_opt'], 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            uig[:, -1 - i] = self._ti.solution['u_opt'][:, -1]

        # assigning ig
        self._prb.getState().setInitialGuess(xig)
        self._prb.getInput().setInitialGuess(uig)

        # open loop update:
        # state on first node is second state 
        # shifted
        self._prb.setInitialState(x0=xig[:, 1])
    
    def _update_closed_loop(self):

        shift_num = -1 # shift data by one node

        # building ig for state
        xig = np.roll(self._ti.solution['x_opt'], 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            xig[:, -1 - i] = self._ti.solution['x_opt'][:, -1]

        # building ig for inputs
        uig = np.roll(self._ti.solution['u_opt'], 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            uig[:, -1 - i] = self._ti.solution['u_opt'][:, -1]

        # assigning ig
        self._prb.getState().setInitialGuess(xig)
        self._prb.getInput().setInitialGuess(uig)

        # sets state on node 0 from measurements
        robot_state = self._assemble_meas_robot_state(to_numpy=True)
        self._prb.setInitialState(x0=
                        robot_state)
    
    def _publish_rhc_sol_data(self):

        self.rhc2shared_bridge.update(q_opt=self._ti.solution['q'], 
                                q_robot=self._assemble_meas_robot_configuration(to_numpy=True))

    def _publish_rob_state_data(self):

        a = 2
        
    def _solve(self):
        
        # self._update_open_loop() # updates the TO ig and 
        # initial conditions using data from the solution itself

        self._update_closed_loop() # updates the TO ig and 
        # # initial conditions using robot measurements
        
        self._pm.shift() # shifts phases of one dt
        
        self.rhc_task_refs.update() # updates rhc references
        # with the latests available

        # check what happend as soon as we try to step on the 1st controller
        # if self.controller_index == 0:

        #     if any((self.rhc_task_refs.phase_id.get_contacts().numpy() < 0.5).flatten().tolist() ):
            
        #         self.step_counter = self.step_counter + 1

        #         print("OOOOOOOOOO I am trying to steppppppp")
        #         print("RHC control index:" + str(self.step_counter))

            # self.horizon_anal.printParameters(elem="f_wheel_1_ref")
            # self.horizon_anal.printParameters(elem="f_wheel_2_ref")
            # self.horizon_anal.printParameters(elem="f_wheel_3_ref")
            # self.horizon_anal.printParameters(elem="f_wheel_4_ref")
            # self.horizon_anal.printVariables(elem="f_ball_1")
            # self.horizon_anal.printParameters(elem="f_wheel_1")
            # self.horizon_anal.printParameters(elem="f_wheel_1")
            # self.horizon_anal.printParameters(elem="f_wheel_1")

            # self.horizon_anal.print()

        try:

            result = self._ti.rti() # solves the problem

            self.sol_counter = self.sol_counter + 1

            if self.publish_sol:

                self._publish_rhc_sol_data() 
                self._publish_rob_state_data()

            if self._debug_sol:
                
                # we update cost and constr. dictionaries
                self.rhc_costs.update(self._ti.solver_rti.getCostsValues())
                self.rhc_constr.update(self._ti.solver_rti.getConstraintsValues())

            return True
        
        except Exception as e:
            
            print(f"[{self.__class__.__name__}" + str(self.controller_index) + "]" + \
              f"[{self.journal.exception}]" + ": rti() failed" + 
              f" with exception{type(e).__name__}")

            if self.publish_sol:
                
                # we publish solution anyway ??

                self._publish_rhc_sol_data() 
                self._publish_rob_state_data()

            return False

    def _reset(self):
        
        # resets controller ig and initial
        # states/inputs
        self._ti.reset()

    def _get_cost_data(self):
        
        cost_dict = self._ti.solver_rti.getCostsValues()
        
        cost_names = list(cost_dict.keys())

        cost_dims = [1] * len(cost_names) # costs are always scalar

        return cost_names, cost_dims
    
    def _get_constr_data(self):
        
        constr_dict = self._ti.solver_rti.getConstraintsValues()

        constr_names = list(constr_dict.keys())

        constr_dims = [-1] * len(constr_names)

        i = 0
        for constr in constr_dict:

            constr_val = constr_dict[constr]
        
            constr_shape = constr_val.shape
            
            constr_dims[i] = constr_shape[0]

            i+=1

        return constr_names, constr_dims
    
    def _get_q_from_sol(self):

        return self._ti.solution['q']

    def _get_v_from_sol(self):

        # to be overridden by child class
        
        return self._ti.solution['v']
    
    def _get_a_from_sol(self):

        # to be overridden by child class
        
        return self._ti.solution['a']
    
    def _get_a_dot_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_f_from_sol(self):

        # to be overridden by child class
        
        return self._ti.solution['a']
    
    def _get_f_dot_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_eff_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_cost_from_sol(self,
                    cost_name: str):

        return self.rhc_costs[cost_name]
    
    def _get_constr_from_sol(self,
                    constr_name: str):

        # to be overridden by child class
        
        return self.rhc_constr[constr_name]