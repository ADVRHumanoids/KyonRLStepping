from lrhc_control.controllers.rhc.horizon_based.horizon_imports import * 
from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc
from lrhc_control.controllers.rhc.horizon_based.gait_manager import GaitManager

from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import Journal, LogType

import numpy as np

import os
import shutil

import time

from kyonrlstepping.controllers.horizon_based.kyon_rhc_task_refs import KyonRHCRefs
from kyonrlstepping.controllers.horizon_based.utils.sysutils import PathsGetter
from scipy.spatial.transform import Rotation

class KyonRhc(HybridQuadRhc):

    def __init__(self, 
            srdf_path: str,
            urdf_path: str,
            robot_name: str, # used for shared memory namespaces
            codegen_dir: str,
            with_wheels: bool = False, 
            n_nodes: float = 31,
            dt: float = 0.05,
            injection_node: int = 10,
            max_solver_iter = 1, # defaults to rt-iteration
            open_loop: bool = True,
            dtype = np.float32, 
            verbose = False, 
            debug = False,
            refs_in_hor_frame = True,
            timeout_ms: int = 60000
            ):

        paths = PathsGetter()
        config_path = paths.RHCCONFIGPATH_WHEELS if with_wheels else paths.RHCCONFIGPATH_NO_WHEELS

        super().__init__(srdf_path=srdf_path,
            urdf_path=urdf_path,
            config_path=config_path,
            robot_name=robot_name, # used for shared memory namespaces
            codegen_dir=codegen_dir, 
            n_nodes=n_nodes,
            dt=dt,
            injection_node=injection_node,
            max_solver_iter=max_solver_iter, # defaults to rt-iteration
            open_loop=open_loop,
            dtype=dtype,
            verbose=verbose, 
            debug=debug,
            refs_in_hor_frame=refs_in_hor_frame,
            timeout_ms=timeout_ms)
    
    def _quaternion_multiply(self, 
                    q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([x, y, z, w])

    def _init_rhc_task_cmds(self):
        
        rhc_refs = KyonRHCRefs(gait_manager=self._gm,
                    robot_index=self.controller_index,
                    namespace=self.namespace,
                    safe=False, 
                    verbose=self._verbose,
                    vlevel=VLevel.V2)
        
        rhc_refs.run()

        rhc_refs.rob_refs.set_jnts_remapping(jnts_remapping=self._to_controller)
        rhc_refs.rob_refs.set_q_remapping(q_remapping=self._get_quat_remap())
              
        # writing initializations
        rhc_refs.reset(p_ref=np.atleast_2d(self._base_init)[:, 0:3], 
            q_ref=np.atleast_2d(self._base_init)[:, 3:7] # will be remapped according to just set q_remapping
            )
        
        return rhc_refs
    
    def _init_problem(self):

        # overrides parent

        self.urdf = self.urdf.replace('continuous', 'revolute') # continous joint is parametrized
        # in So2, so will add 

        self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())

        self._prb = Problem(self._n_intervals, 
                        receding=True, 
                        casadi_type=cs.SX)
        self._prb.setDt(self._dt)

        self._init_robot_homer()

        init = self._base_init.tolist() + list(self._homer.get_homing())
        FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
        kyon_wheel_radius = 0.124 # hardcoded!!!!
        init_pos_foot = FK(q=init)['ee_pos']
        self._base_init[2] = -init_pos_foot[2]  # override init
        if 'wheel_joint_1' in self._kin_dyn.joint_names():
            self._base_init[2] += kyon_wheel_radius

        self._model = FullModelInverseDynamics(problem=self._prb,
                                kd=self._kin_dyn,
                                q_init=self._homer.get_homing_map(),
                                base_init=self._base_init)
        
        self._ti = TaskInterface(prb=self._prb, 
                            model=self._model, 
                            max_solver_iter=self.max_solver_iter,
                            debug = self._debug,
                            verbose = self._verbose, 
                            codegen_workdir = self._codegen_dir)
        
        self._ti.setTaskFromYaml(self.config_path)

        # setting initial base pos ref
        base_pos = self._ti.getTask('base_height')
        base_pos.setRef(np.atleast_2d(self._base_init).T)

        self._tg = trajectoryGenerator.TrajectoryGenerator()

        self._pm = pymanager.PhaseManager(self._n_nodes, debug=False) # intervals or nodes?????

        self._create_whitelist()
        self._init_contact_timelines()
        self._add_zmp()

        self._ti.model.q.setBounds(self._ti.model.q0, self._ti.model.q0, nodes=0)
        self._ti.model.v.setBounds(self._ti.model.v0, self._ti.model.v0, nodes=0)
        self._ti.model.q.setInitialGuess(self._ti.model.q0)
        self._ti.model.v.setInitialGuess(self._ti.model.v0)
        f0 = [0, 0, self._kin_dyn.mass() / 4 * 9.8]
        for _, cforces in self._ti.model.cmap.items():
            for c in cforces:
                c.setInitialGuess(f0)
        # setting ref for force reg.
        force_ref = self._ti.getTask('joint_regularization')
        force_ref.setRef(index=2, # force
                    ref=np.atleast_2d(np.array(f0)).T)
        force_ref.setRef(index=3, # force
                    ref=np.atleast_2d(np.array(f0)).T)
        force_ref.setRef(index=4, # force
                    ref=np.atleast_2d(np.array(f0)).T)
        force_ref.setRef(index=5, # force
                    ref=np.atleast_2d(np.array(f0)).T)

        vel_lims = self._model.kd.velocityLimits()
        import horizon.utils as utils
        self._prb.createResidual('max_vel', 1e1 * utils.utils.barrier(vel_lims[7:] - self._model.v[7:]))
        self._prb.createResidual('min_vel', 1e1 * utils.utils.barrier1(-1 * vel_lims[7:] - self._model.v[7:]))

        self._ti.finalize()
        self._ti.bootstrap()
        self._ti.init_inv_dyn_for_res() # we initialize some objects for sol. postprocessing purposes
        self._ti.load_initial_guess()

        contact_phase_map = {c: f'{c}_timeline' for c in self._model.cmap.keys()}
        
        self._gm = GaitManager(self._ti, self._pm, contact_phase_map, self._injection_node)

        self.n_dofs = self._get_ndofs() # after loading the URDF and creating the controller we
        # know n_dofs -> we assign it (by default = None)

        self.n_contacts = len(self._model.cmap.keys())
        
        # self.horizon_anal = analyzer.ProblemAnalyzer(self._prb)
    
    def _init_contact_timelines(self):
        
        c_timelines = dict()
        for c in self._model.cmap.keys():
            c_timelines[c] = self._pm.createTimeline(f'{c}_timeline')

        short_stance_duration = 1
        stance_duration = 8
        flight_duration = 8
        post_landing_stance = 5
        for c in self._model.cmap.keys():
            # stance phase normal
            stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
            stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
            if self._ti.getTask(f'{c}_contact') is not None:
                stance_phase.addItem(self._ti.getTask(f'{c}_contact'))
                stance_phase_short.addItem(self._ti.getTask(f'{c}_contact'))
            else:
                raise Exception('task not found')

            # flight phase normal
            flight_phase = c_timelines[c].createPhase(flight_duration+post_landing_stance, f'flight_{c}')
            init_z_foot = self._model.kd.fk(c)(q=self._model.q0)['ee_pos'].elements()[2]
            ee_vel = self._model.kd.frameVelocity(c, self._model.kd_frame)(q=self._model.q, qdot=self._model.v)['ee_vel_linear']
            ref_trj = np.zeros(shape=[7, flight_duration])
            ref_trj[2, :] = np.atleast_2d(self._tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.15, [None, 0, None]))
            if self._ti.getTask(f'z_{c}') is not None:
                flight_phase.addItemReference(self._ti.getTask(f'z_{c}'), ref_trj, nodes=list(range(0, flight_duration)))
                flight_phase.addItem(self._ti.getTask(f'{c}_contact'), nodes=list(range(flight_duration, flight_duration+post_landing_stance)))
            else:
                raise Exception('task not found')
            cstr = self._prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
            flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

        for c in self._model.cmap.keys():
            # stance = c_timelines[c].getRegisteredPhase(f'stance_{c}_short')
            stance = c_timelines[c].getRegisteredPhase(f'stance_{c}_short')
            while c_timelines[c].getEmptyNodes() > 0:
                c_timelines[c].addPhase(stance)

    def _create_whitelist(self):

        # weight more roll joints
        white_list_indices = list()
        black_list_indices = list()
        white_list = []

        if 'wheel_joint_1' in self._model.kd.joint_names():
            black_list = ['wheel_joint_1', 'wheel_joint_2', 'wheel_joint_3', 'wheel_joint_4']
        else:
            black_list = []

        postural_joints = np.array(list(range(7, self._model.nq)))
        for joint in black_list:
            black_list_indices.append(self._model.joint_names.index(joint))
        for joint in white_list:
            white_list_indices.append(7 + self._model.joint_names.index(joint))
        postural_joints = np.delete(postural_joints, black_list_indices)

        if white_list:
            self._prb.createResidual("min_q_white_list", 5. * (self._model.q[white_list_indices] - self._model.q0[white_list_indices]))
        # if black_list:
        #     prb.createResidual('velocity_regularization', 0.1 * model.v[postural_joints])
    
    def _add_zmp(self):

        input_zmp = []

        input_zmp.append(self._model.q)
        input_zmp.append(self._model.v)
        input_zmp.append(self._model.a)

        for f_var in self._model.fmap.values():
            input_zmp.append(f_var)

        c_mean = cs.SX([0, 0, 0])
        for c_name, f_var in self._model.fmap.items():
            fk_c_pos = self._kin_dyn.fk(c_name)(q=self._model.q)['ee_pos']
            c_mean += fk_c_pos

        c_mean /= len(self._model.cmap.keys())

        zmp_nominal_weight = 10.
        zmp_fun = self._zmp(self._model)(*input_zmp)

        if 'wheel_joint_1' in self._model.kd.joint_names():
            zmp_residual = self._prb.createIntermediateResidual('zmp',  zmp_nominal_weight * (zmp_fun[0:2] - c_mean[0:2]))

    def _zmp(self, model):

        num = cs.SX([0, 0])
        den = cs.SX([0])
        pos_contact = dict()
        force_val = dict()

        q = cs.SX.sym('q', model.nq)
        v = cs.SX.sym('v', model.nv)
        a = cs.SX.sym('a', model.nv)

        com = model.kd.centerOfMass()(q=q, v=v, a=a)['com']

        n = cs.SX([0, 0, 1])
        for c in model.fmap.keys():
            pos_contact[c] = model.kd.fk(c)(q=q)['ee_pos']
            force_val[c] = cs.SX.sym('force_val', 3)
            num += (pos_contact[c][0:2] - com[0:2]) * cs.dot(force_val[c], n)
            den += cs.dot(force_val[c], n)

        zmp = com[0:2] + (num / den)
        input_list = []
        input_list.append(q)
        input_list.append(v)
        input_list.append(a)

        for elem in force_val.values():
            input_list.append(elem)

        f = cs.Function('zmp', input_list, [zmp])
        return f

    def _set_ig(self):

        shift_num = -1 # shift data by one node

        x_opt = self._ti.solution['x_opt']
        u_opt = self._ti.solution['u_opt']

        # building ig for state
        xig = np.roll(x_opt, 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            xig[:, -1 - i] = x_opt[:, -1]
        # building ig for inputs
        uig = np.roll(u_opt, 
                shift_num, axis=1) # rolling state sol.
        for i in range(abs(shift_num)):
            # state on last node is copied to the elements
            # which are "lost" during the shift operation
            uig[:, -1 - i] = u_opt[:, -1]

        # assigning ig
        self._prb.getState().setInitialGuess(xig)
        self._prb.getInput().setInitialGuess(uig)

        return xig, uig

    def _update_open_loop(self):

        xig, _ = self._set_ig()

        # open loop update:
        self._prb.setInitialState(x0=xig[:, 0]) # (xig has been shifted, so node 0
        # is node 1 in the last opt solution)
    
    def _update_closed_loop(self):

        self._set_ig()

        # sets state on node 0 from measurements
        robot_state = self._assemble_meas_robot_state(x_opt=self._ti.solution['x_opt'])
        # robot_state = self._assemble_meas_robot_state()

        self._prb.setInitialState(x0=
                        robot_state)
    
    def _assemble_meas_robot_state(self,
                            x_opt = None):

        # overrides parent
        q_jnts = self.robot_state.jnts_state.get(data_type="q", robot_idxs=self.controller_index).reshape(-1, 1)
        v_jnts = self.robot_state.jnts_state.get(data_type="v", robot_idxs=self.controller_index).reshape(-1, 1)
        q_root = self.robot_state.root_state.get(data_type="q", robot_idxs=self.controller_index).reshape(-1, 1)
        p = self.robot_state.root_state.get(data_type="p", robot_idxs=self.controller_index).reshape(-1, 1)
        v_root = self.robot_state.root_state.get(data_type="v", robot_idxs=self.controller_index).reshape(-1, 1)
        omega = self.robot_state.root_state.get(data_type="omega", robot_idxs=self.controller_index).reshape(-1, 1)
        
        # we need twist in local base frame, but measured one is global
        if x_opt is not None:
            # CHECKING q_root for sign consistency!
            # numerical problem: two quaternions can represent the same rotation
            # if difference between the base q in the state x on first node and the sensed q_root < 0, change sign
            state_quat_conjugate = np.copy(x_opt[3:7, 0])
            state_quat_conjugate[:3] *= -1.0
            # normalize the quaternion
            state_quat_conjugate = state_quat_conjugate / np.linalg.norm(x_opt[3:7, 0])
            diff_quat = self._quaternion_multiply(q_root, state_quat_conjugate)
            if diff_quat[3] < 0:
                q_root[:] = -q_root

        r_base = Rotation.from_quat(q_root.flatten()).as_matrix()

        return np.concatenate((p, q_root, q_jnts, r_base.T @ v_root, r_base.T @ omega, v_jnts),
                axis=0)