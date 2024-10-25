from lrhc_control.controllers.rhc.horizon_based.horizon_imports import * 
from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc
from lrhc_control.controllers.rhc.horizon_based.gait_manager import GaitManager

from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import Journal, LogType

import numpy as np

import re
from typing import Dict 

from kyonrlstepping.controllers.horizon_based.kyon_rhc_task_refs import KyonRHCRefs
from kyonrlstepping.utils.sysutils import PathsGetter

class KyonRhc(HybridQuadRhc):

    def __init__(self, 
            srdf_path: str,
            urdf_path: str,
            robot_name: str, # used for shared memory namespaces
            codegen_dir: str,
            with_wheels: bool = False, 
            n_nodes: float = 31,
            dt: float = 0.03,
            injection_node: int = 10,
            max_solver_iter = 1, # defaults to rt-iteration
            open_loop: bool = True,
            close_loop_all: bool = False,
            dtype = np.float32, 
            verbose = False, 
            debug = False,
            refs_in_hor_frame = True,
            timeout_ms: int = 60000,
            custom_opts: Dict = {}):

        paths = PathsGetter()
        config_path = paths.RHCCONFIGPATH_WHEELS if with_wheels else paths.RHCCONFIGPATH_NO_WHEELS
        
        self._with_wheels=with_wheels

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
            close_loop_all=close_loop_all,
            dtype=dtype,
            verbose=verbose, 
            debug=debug,
            refs_in_hor_frame=refs_in_hor_frame,
            timeout_ms=timeout_ms,
            custom_opts=custom_opts)
        
        self._fail_idx_scale=1e-9
        self._fail_idx_thresh_open_loop=1e0
        self._fail_idx_thresh_closed_loop=5e0
        if open_loop:
            self._fail_idx_thresh=self._fail_idx_thresh_open_loop
        else:
            self._fail_idx_thresh=self._fail_idx_thresh_closed_loop

    def _set_rhc_pred_idx(self):
        self._pred_node_idx=round((self._n_nodes-1)*2/3)
        
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
        
        vel_bounds_weight=1.0
        meas_state_attractor_weight=100.0
        self._phase_force_reg=1e-2
        self._yaw_vertical_weight=2.0
        # overrides parent
        self._prb = Problem(self._n_intervals, 
                        receding=True, 
                        casadi_type=cs.SX)
        self._prb.setDt(self._dt)

        if self._custom_opts["replace_continuous_joints"]:
            # continous joints are parametrized in So2
            self.urdf = self.urdf.replace('continuous', 'revolute')
        self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf) # used for getting joint names 
        self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())
        
        self._continuous_joints=self._get_continuous_jnt_names()
        # reduced
        self._continuous_joints_idxs=[]
        self._continuous_joints_idxs_cos=[]
        self._continuous_joints_idxs_sin=[]
        self._continuous_joints_idxs_red=[]
        self._rev_joints_idxs=[]
        self._rev_joints_idxs_red=[]
        # qfull
        self._continuous_joints_idxs_qfull=[]
        self._continuous_joints_idxs_cos_qfull=[]
        self._continuous_joints_idxs_sin_qfull=[]
        self._continuous_joints_idxs_red_qfull=[]
        self._rev_joints_idxs_qfull=[]
        self._rev_joints_idxs_red_qfull=[]
        wheel_pattern = r"wheel_joint"
        are_there_wheels=any(re.search(wheel_pattern, s) for s in self._get_robot_jnt_names())
        
        self._init_robot_homer()

        if (not are_there_wheels) and (self._with_wheels):
            # not allowed
            Journal.log(self.__class__.__name__,
                "_init_problem",
                f"Provided _with_wheels arg was set to True, but wheels not found in URDF!",
                LogType.EXCEP,
                throw_when_excep=True)
        fixed_joint_map={}
        if are_there_wheels and (not self._with_wheels): # we need to fix some joints
            fixed_joints = [f'{wheel_pattern}_{i + 1}' for i in range(4)]
            fixed_jnt_vals = len(fixed_joints)*[0.] # default to 0
            fixed_joint_map=dict(zip(fixed_joints, fixed_jnt_vals))
            fixed_jnts_homing=self._homer.get_homing_vals(jnt_names=fixed_joints)# ordered as fixed_joints
            # update fixed joints map from homing:
            for i in range(len(fixed_joints)):
                jnt_name=fixed_joints[i]
                fixed_joint_map[jnt_name]=fixed_jnts_homing[i]

        self._using_wheels=False
        if are_there_wheels and self._with_wheels:
            self._using_wheels=True
        if not len(fixed_joint_map)==0: # we need to recreate kin dyn and homers
            self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf,fixed_joints=fixed_joint_map)
            self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())
            self._init_robot_homer()
        
        self._wheel_jnts_idxs=[]
        jnt_homing=[""]*(len(self._homer.get_homing())+len(self._continuous_joints))
        jnt_names=self._get_robot_jnt_names()
        for i in range(len(jnt_names)):
            jnt=jnt_names[i]
            index=self._get_jnt_id(jnt)# accounting for floating joint
            homing_idx=index-7 # homing is only for actuated joints
            if re.search(wheel_pattern, jnt): # found wheel joint
                self._wheel_jnts_idxs.append(index)
            homing_value=self._homer.get_homing_val(jnt)
            if jnt in self._continuous_joints:
                jnt_homing[homing_idx]=np.cos(homing_value).item()
                jnt_homing[homing_idx+1]=np.sin(homing_value).item()
                # just actuated joints
                self._continuous_joints_idxs.append(homing_idx) # cos
                self._continuous_joints_idxs.append(homing_idx+1) # sin
                self._continuous_joints_idxs_cos.append(homing_idx)
                self._continuous_joints_idxs_sin.append(homing_idx+1)
                self._continuous_joints_idxs_red.append(i)
                # q full
                self._continuous_joints_idxs_qfull.append(index) # cos
                self._continuous_joints_idxs_qfull.append(index+1) # sin
                self._continuous_joints_idxs_cos_qfull.append(index)
                self._continuous_joints_idxs_sin_qfull.append(index+1)
                self._continuous_joints_idxs_red_qfull.append(i+7)
            else:
                jnt_homing[homing_idx]=homing_value
                # just actuated joints
                self._rev_joints_idxs.append(homing_idx) 
                self._rev_joints_idxs_red.append(i) 
                # q full
                self._rev_joints_idxs_qfull.append(index) 
                self._rev_joints_idxs_red_qfull.append(i+7) 

        self._jnts_q_reduced=None
        if not len(self._continuous_joints)==0:
            self._jnts_q_reduced=np.zeros((1,self.nv()-6),dtype=self._dtype)
            self._jnts_q_expanded=np.zeros((1,self.nq()-7),dtype=self._dtype)
            self._full_q_reduced=np.zeros((7+len(jnt_names), self._n_nodes),dtype=self._dtype)

        self._f0 = [0, 0, self._kin_dyn.mass() / 4 * 9.8]

        init = self._base_init.tolist() + jnt_homing

        FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
        self._wheel_radius = 0.124 # hardcoded!!!!
        ground_level = FK(q=init)['ee_pos']
        self._base_init[2] = -ground_level[2]  # override init     
        if are_there_wheels:
            self._base_init[2] += self._wheel_radius # even if in fixed joints, 
        # in the real robot the wheel is there. This way the feet z in homing is at height
        
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
        if base_pos is not None:
            base_pos.setRef(np.atleast_2d(self._base_init).T)

        self._tg = trajectoryGenerator.TrajectoryGenerator()

        self._pm = pymanager.PhaseManager(self._n_nodes, debug=False) # intervals or nodes?????

        # self._create_whitelist()
        self._init_contact_timelines()
        # self._add_zmp()

        self._ti.model.q.setBounds(self._ti.model.q0, self._ti.model.q0, nodes=0)
        self._ti.model.v.setBounds(self._ti.model.v0, self._ti.model.v0, nodes=0)
        self._ti.model.q.setInitialGuess(self._ti.model.q0)
        self._ti.model.v.setInitialGuess(self._ti.model.v0)
        for _, cforces in self._ti.model.cmap.items():
            n_contact_f=len(cforces)
            for c in cforces:
                c.setInitialGuess(np.array(self._f0)/n_contact_f)        

        vel_lims = self._model.kd.velocityLimits()
        import horizon.utils as utils
        self._prb.createResidual('vel_lb_barrier', vel_bounds_weight*utils.utils.barrier(vel_lims[7:] - self._model.v[7:]))
        self._prb.createResidual('vel_ub_barrier', vel_bounds_weight*utils.utils.barrier1(-1 * vel_lims[7:] - self._model.v[7:]))

        # if not self._open_loop:
        #     # we create a residual cost to be used as an attractor to the measured state on the first node
        #     # hard constraints injecting meas. states are pure EVIL!
        #     prb_state=self._prb.getState()
        #     full_state=prb_state.getVars()
        #     state_dim=prb_state.getBounds()[0].shape[0]
        #     meas_state=self._prb.createParameter(name="measured_state",
        #         dim=state_dim, nodes=0)     
        #     self._prb.createResidual('meas_state_attractor', meas_state_attractor_weight * (full_state - meas_state), 
        #                 nodes=[0])

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
        
        short_stance_duration = 1
        flight_duration = 10
        post_landing_stance = 5
        step_height=0.12
        for c in self._model.cmap.keys():

            # stance phases
            self._c_timelines[c] = self._pm.createTimeline(f'{c}_timeline')
            stance_phase_short = self._c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short')
            if self._ti.getTask(f'{c}') is not None:
                stance_phase_short.addItem(self._ti.getTask(f'{c}'))
                ref_trj = np.zeros(shape=[7, short_stance_duration])
                stance_phase_short.addItemReference(self._ti.getTask(f'z_{c}'),
                    ref_trj, nodes=list(range(0, short_stance_duration)))
            else:
                Journal.log(self.__class__.__name__,
                    "_init_contact_timelines",
                    f"contact task {c} not found",
                    LogType.EXCEP,
                    throw_when_excep=True)
            i=0
            n_forces=len(self._ti.model.cmap[c])
            for force in self._ti.model.cmap[c]:
                f_ref=self._prb.createParameter(name=f"{c}_force_reg_f{i}_ref",
                    dim=3) 
                force_reg=self._prb.createResidual(f'{c}_force_reg_f{i}', self._phase_force_reg * (force - f_ref), 
                    nodes=[])
                f_ref.assign(np.atleast_2d(np.array(self._f0)).T/n_forces)    
                stance_phase_short.addCost(force_reg, nodes=list(range(0, short_stance_duration)))
                i+=1

            # flight phases
            flight_phase = self._c_timelines[c].createPhase(flight_duration+post_landing_stance, f'flight_{c}')
            init_z_foot = self._model.kd.fk(c)(q=self._model.q0)['ee_pos'].elements()[2]
            ee_vel = self._model.kd.frameVelocity(c, self._model.kd_frame)(q=self._model.q, qdot=self._model.v)['ee_vel_linear']

            # post landing contact + force reg
            if not post_landing_stance<1:
                if self._ti.getTask(f'{c}') is not None:
                    flight_phase.addItem(self._ti.getTask(f'{c}'), nodes=list(range(flight_duration, flight_duration+post_landing_stance)))
                    ref_trj = np.zeros(shape=[7, post_landing_stance])
                    flight_phase.addItemReference(self._ti.getTask(f'z_{c}'),
                        ref_trj,
                        nodes=list(range(flight_duration, flight_duration+post_landing_stance)))
                    i=0
                    for force in self._ti.model.cmap[c]:
                        force_reg=self._prb.getCosts(f'{c}_force_reg_f{i}')
                        flight_phase.addCost(force_reg, nodes=list(range(flight_duration, flight_duration+post_landing_stance)))
                        i+=1
                else:
                    Journal.log(self.__class__.__name__,
                        "_init_contact_timelines",
                        f"contact task {c} not found!",
                        LogType.EXCEP,
                        throw_when_excep=True)
            # reference traj
            der= [None, 0, 0]
            second_der=[None, 0, 0]
            # flight pos
            if self._ti.getTask(f'z_{c}') is not None:
                ref_trj = np.zeros(shape=[7, flight_duration])
                ref_trj[2, :] = np.atleast_2d(self._tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, step_height,
                    derivatives=der,
                    second_der=second_der))
                flight_phase.addItemReference(self._ti.getTask(f'z_{c}'), ref_trj, nodes=list(range(0, flight_duration)))
            else:
                Journal.log(self.__class__.__name__,
                    "_init_contact_timelines",
                    f"contact pos traj tracking task z_{c} not found-> it won't be used",
                    LogType.WARN,
                    throw_when_excep=True)
            # flight vel
            if self._ti.getTask(f'vz_{c}') is not None:
                ref_vtrj = np.zeros(shape=[1, flight_duration])
                ref_vtrj[:, :] = np.atleast_2d(self._tg.derivative_of_trajectory(flight_duration, init_z_foot, init_z_foot, step_height, 
                    derivatives=der,
                    second_der=second_der))
                flight_phase.addItemReference(self._ti.getTask(f'vz_{c}'), ref_vtrj, nodes=list(range(0, flight_duration)))
            else:
                Journal.log(self.__class__.__name__,
                    "_init_contact_timelines",
                    f"contact vel traj tracking task vz_{c} not found-> it won't be used",
                    LogType.WARN,
                    throw_when_excep=True)
            
            cstr = self._prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
            flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

            # p_ref_landing=self._prb.createParameter(name=f"{c}_p_landing_ref",
            #     dim=1)
            # foot_pos=self._model.kd.fk(c)(q=self._model.q)['ee_pos'][2, :]

            # p_ref_constr=self._prb.createResidual(f'{c}_landing_pos', 1e2*(foot_pos - p_ref_landing), 
            #         nodes=[])
            # p_ref_landing.assign(np.atleast_2d(np.array([0.0])))    
            # flight_phase.addCost(p_ref_constr, nodes=[flight_duration-1])
            # keep ankle vertical
            # c_ori = self._model.kd.fk(c)(q=self._model.q)['ee_rot'][2, :]
            # cost_ori = self._prb.createResidual(f'{c}_ori', self._yaw_vertical_weight * (c_ori.T - np.array([0, 0, 1])))
            # flight_phase.addCost(cost_ori, nodes=list(range(0, flight_duration+post_landing_stance)))

        self._reset_contact_timelines()

    def _reset_contact_timelines(self):
        for c in self._model.cmap.keys():
            # fill timeline with stances
            contact_timeline=self._c_timelines[c]
            contact_timeline.clear() # remove phases
            stance = contact_timeline.getRegisteredPhase(f'stance_{c}_short')
            while contact_timeline.getEmptyNodes() > 0:
                contact_timeline.addPhase(stance)
            # f reg
            # if self._add_f_reg_timeline:
            #     freg_tline=self._f_reg_timelines[c]
            #     freg_tline.clear()
            #     f_stance = freg_tline.getRegisteredPhase(f'freg_{c}_short')
            #     for i in range(self._n_nodes-1): # not defined on last node
            #         freg_tline.addPhase(f_stance)

    def _create_whitelist(self):

        # weight more roll joints
        white_list_indices = list()
        black_list_indices = list()
        white_list = []

        if self._wheel_names[0] in self._model.kd.joint_names():
            black_list = self._wheel_names
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
    
    def _assemble_meas_robot_state(self,
                        x_opt = None,
                        close_all: bool=False):

        # overrides parent
        q_jnts = self.robot_state.jnts_state.get(data_type="q", robot_idxs=self.controller_index).reshape(1, -1)
        v_jnts = self.robot_state.jnts_state.get(data_type="v", robot_idxs=self.controller_index).reshape(1, -1)
        q_root = self.robot_state.root_state.get(data_type="q", robot_idxs=self.controller_index).reshape(1, -1)
        p = self.robot_state.root_state.get(data_type="p", robot_idxs=self.controller_index).reshape(1, -1)
        v_root = self.robot_state.root_state.get(data_type="v", robot_idxs=self.controller_index).reshape(1, -1)
        omega = self.robot_state.root_state.get(data_type="omega", robot_idxs=self.controller_index).reshape(1, -1)

        if (not len(self._continuous_joints)==0): # we need do expand some meas. rev jnts to So2
            # copy rev joints
            self._jnts_q_expanded[:, self._rev_joints_idxs]=q_jnts[:, self._rev_joints_idxs_red]
            self._jnts_q_expanded[:, self._continuous_joints_idxs_cos]=np.cos(q_jnts[:, self._continuous_joints_idxs_red]) # cos
            self._jnts_q_expanded[:, self._continuous_joints_idxs_sin]=np.sin(q_jnts[:, self._continuous_joints_idxs_red]) # sin
            q_jnts=self._jnts_q_expanded
        # meas twist is assumed to be provided in BASE link!!!
        if not close_all and not self._using_wheels: # use internal MPC for the base and joints
            p[:, 0:3]=self._get_root_full_q_from_sol(node_idx=1)[:, 0:3] # base pos is open loop
            # v_root[0:3,:]=self._get_root_twist_from_sol(node_idx=1)[0:3, :]
            # q_jnts[:, :]=self._get_jnt_q_from_sol(node_idx=1,reduce=False)           
            v_jnts[:, :]=self._get_jnt_v_from_sol(node_idx=1)
        if not close_all and self._using_wheels: 
            p[:, 0:3]=self._get_root_full_q_from_sol(node_idx=1)[:, 0:3] # base pos is open loop
            # v_root[0:3,:]=self._get_root_twist_from_sol(node_idx=1)[0:3, :]
            q_jnts[:, :]=self._get_jnt_q_from_sol(node_idx=1,reduce=False,clamp=False)       
            v_jnts[:, :]=self._get_jnt_v_from_sol(node_idx=1)
        # r_base = Rotation.from_quat(q_root.flatten()).as_matrix() # from base to world (.T the opposite)
        
        if x_opt is not None:
            # CHECKING q_root for sign consistency!
            # numerical problem: two quaternions can represent the same rotation
            # if difference between the base q in the state x on first node and the sensed q_root < 0, change sign
            state_quat_conjugate = np.copy(x_opt[3:7, 0])
            state_quat_conjugate[:3] *= -1.0
            # normalize the quaternion
            state_quat_conjugate = state_quat_conjugate / np.linalg.norm(x_opt[3:7, 0])
            diff_quat = self._quaternion_multiply(q_root.flatten(), state_quat_conjugate)
            if diff_quat[3] < 0:
                q_root[:, :] = -q_root[:, :]
        
        return np.concatenate((p, q_root, q_jnts, v_root, omega, v_jnts),
                axis=1).reshape(-1,1)
    
    def _get_q_from_sol(self):
        full_q=super()._get_q_from_sol()
        if self._custom_opts["replace_continuous_joints"]:
            return full_q
        else:
            cont_jnts=full_q[self._continuous_joints_idxs_qfull, :]
            cos=cont_jnts[::2, :]
            sin=cont_jnts[1::2, :]
            # copy root
            self._full_q_reduced[0:7, :]=full_q[0:7, :]
            # copy rev joint vals
            self._full_q_reduced[self._rev_joints_idxs_red_qfull, :]=full_q[self._rev_joints_idxs_qfull, :]
            # and continuous
            angle=np.arctan2(sin, cos)
            self._full_q_reduced[self._continuous_joints_idxs_red_qfull, :]=angle
            return self._full_q_reduced
            
    def _get_jnt_q_from_sol(self, node_idx=1, 
            reduce: bool = True,
            clamp: bool = True):
        
        full_jnts_q=self._ti.solution['q'][7:, node_idx:node_idx+1].reshape(1,-1)

        if self._custom_opts["replace_continuous_joints"] or (not reduce):
            if clamp:
                return np.fmod(full_jnts_q, 2*np.pi)
            else:
                return full_jnts_q
        else:
            cos_sin=full_jnts_q[:,self._continuous_joints_idxs].reshape(-1,2)
            # copy rev joint vals
            self._jnts_q_reduced[:, self._rev_joints_idxs_red]=np.fmod(full_jnts_q[:, self._rev_joints_idxs], 2*np.pi).reshape(1, -1)
            # and continuous
            self._jnts_q_reduced[:, self._continuous_joints_idxs_red]=np.arctan2(cos_sin[:, 1], cos_sin[:, 0]).reshape(1,-1)
            return self._jnts_q_reduced


