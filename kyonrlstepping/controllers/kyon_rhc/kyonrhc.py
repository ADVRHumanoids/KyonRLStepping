from control_cluster_bridge.controllers.rhc import RHController
from control_cluster_bridge.utilities.homing import RobotHomer

from kyonrlstepping.controllers.kyon_rhc.horizon_imports import * 

from kyonrlstepping.controllers.kyon_rhc.gait_manager import GaitManager

import numpy as np

import torch

import time 

class KyonRHC(RHController):

    def __init__(self, 
            controller_index: int,
            srdf_path: str,
            urdf_path: str,
            config_path: str,
            cluster_size: int, # needed by shared mem manager
            robot_name: str = "kyon0",
            t_horizon:float = 3.0,
            n_nodes: int = 30,
            add_data_lenght: int = 2,
            enable_replay = False, 
            verbose = False, 
            debug = False, 
            array_dtype = torch.float32):

        self.step_counter = 0
        self.sol_counter = 0

        self.robot_name = robot_name
        
        self._enable_replay = enable_replay
        self._t_horizon = t_horizon
        self._n_nodes = n_nodes

        self.config_path = config_path

        self.urdf_path = urdf_path
        # read urdf and srdf files
        with open(self.urdf_path, 'r') as file:

            self.urdf = file.read()

        super().__init__(controller_index = controller_index, 
                        cluster_size = cluster_size,
                        srdf_path = srdf_path,
                        namespace = self.robot_name,
                        verbose = verbose, 
                        debug = debug,
                        array_dtype = array_dtype)

        self.add_data_lenght = add_data_lenght # length of the array holding additional info from the solver

        self._quat_remap = [1, 2, 3, 0] # mapping from robot quat. to Horizon's quaternion convention
    
    def _init_problem(self):
        
        print(f"[{self.__class__.__name__}" + str(self.controller_index) + "]" + \
              f"[{self.journal.status}]" + ": initializing RHC problem")

        self.urdf = self.urdf.replace('continuous', 'revolute')
        self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self._assign_server_side_jnt_names(self._get_robot_jnt_names())

        self._dt = self._t_horizon / self._n_nodes
        self._prb = Problem(self._n_nodes, 
                        receding=True, 
                        casadi_type=cs.SX)
        self._prb.setDt(self._dt)

        self._homer = RobotHomer(srdf_path=self.srdf_path, 
                            jnt_names_prb=self._server_side_jnt_names)

        import numpy as np
        base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        init = base_init.tolist() + list(self._homer.get_homing())

        FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
        init_pos_foot = FK(q=init)['ee_pos']
        base_init[2] = -init_pos_foot[2]

        self._model = FullModelInverseDynamics(problem=self._prb,
                                kd=self._kin_dyn,
                                q_init=self._homer.get_homing_map(),
                                base_init=base_init)
        
        self._ti = TaskInterface(prb=self._prb, 
                                model=self._model, 
                                debug = self._debug, 
                                verbose = self._verbose)
        
        self._ti.setTaskFromYaml(self.config_path)

        CoM_pose = self._ti.getTask('CoM_pose')
        CoM_pose.setRef(np.atleast_2d(base_init).T)

        self._tg = trajectoryGenerator.TrajectoryGenerator()

        self._pm = pymanager.PhaseManager(self._n_nodes)
        c_phases = dict()
        for c in self._model.cmap.keys():
            c_phases[c] = self._pm.addTimeline(f'{c}_timeline')

        stance_duration = 5
        flight_duration = 5
        for c in self._model.cmap.keys():

            # stance phase normal
            stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
            if self._ti.getTask(f'{c}_contact') is not None:
                stance_phase.addItem(self._ti.getTask(f'{c}_contact'))
            else:
                raise Exception(f"[{self.__class__.__name__}]" + f"[{self.journal.exception}]" + ": task not found")

            c_phases[c].registerPhase(stance_phase)

            # flight phase normal
            flight_phase = pyphase.Phase(flight_duration, f'flight_{c}')
            init_z_foot = self._model.kd.fk(c)(q=self._model.q0)['ee_pos'].elements()[2]
            ref_trj = np.zeros(shape=[7, flight_duration])
            ref_trj[2, :] = np.atleast_2d(self._tg.from_derivatives(flight_duration, 
                                                        init_z_foot, 
                                                        init_z_foot, 
                                                        0.1, [None, 0, None]))
            if self._ti.getTask(f'z_{c}') is not None:
                flight_phase.addItemReference(self._ti.getTask(f'z_{c}'), ref_trj)
            else:
                 raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + f": task {c}_contact not found")
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
        
        self.horizon_anal = analyzer.ProblemAnalyzer(self._prb)

        print(f"[{self.__class__.__name__}" + str(self.controller_index) + "]" +  f"[{self.journal.status}]" + "Initialized RHC problem")

    def _init_rhc_task_cmds(self):

        a = 6
    
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
    
        return torch.tensor(self._ti.solution['q'][7:, 0], 
                        dtype=self.array_dtype).reshape(1, 
                                        self.n_dofs)
    
    def _get_cmd_jnt_v_from_sol(self):

        return torch.tensor(self._ti.solution['v'][6:, 0], 
                        dtype=self.array_dtype).reshape(1, 
                                        self.n_dofs)

    def _get_cmd_jnt_eff_from_sol(self):
        
        efforts_on_first_node = self._ti.eval_efforts_on_first_node()

        return torch.tensor(efforts_on_first_node[6:, 0], 
                        dtype=self.array_dtype).reshape(1, 
                                        self.n_dofs)
    
    def _get_additional_slvr_info(self):

        return torch.tensor([self._ti.solution["opt_cost"], 
                            self._ti.solution["n_iter2sol"]], 
                        dtype=self.array_dtype)
    
    def _update_closed_loop(self):

        a = "Traversaro"
        
    def _update_open_loop(self):

        # set initial state and initial guess
        shift_num = -1

        x_opt =  self._ti.solution['x_opt']
        xig = np.roll(x_opt, shift_num, axis=1)
        for i in range(abs(shift_num)):
            xig[:, -1 - i] = x_opt[:, -1]

        self._prb.getState().setInitialGuess(xig)
        
        self._prb.setInitialState(x0=xig[:, 0])

    def _solve(self):
        
        start_time = time.time()

        self._update_open_loop() 

        open_loop_update = time.time() - start_time

        start_time = time.time()

        self._pm.shift() # shifts phases of one dt

        shift_time = time.time() - start_time

        # self.rhc_task_refs.update() # updates rhc references
        # # with the latests available

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

        start_time = time.time()

        self._ti.rti() # solves the problem

        rti_time = time.time() - start_time

        debug_msg = str(f"Controller n.{self.controller_index}: ") + \
                    f"Open loop update: {open_loop_update}\n" + \
                    f"Phase shift: {shift_time}\n" + \
                    f"rti: {rti_time}\n"
        
        print(debug_msg)
        
        self.sol_counter = self.sol_counter + 1
        
        # time.sleep(0.02)
