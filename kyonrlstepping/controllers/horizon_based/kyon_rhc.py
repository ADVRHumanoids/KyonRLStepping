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
            close_loop_all=close_loop_all,
            dtype=dtype,
            verbose=verbose, 
            debug=debug,
            refs_in_hor_frame=refs_in_hor_frame,
            timeout_ms=timeout_ms)
        
        self._fail_idx_scale=1e-9
        self._fail_idx_thresh_open_loop=1e0
        self._fail_idx_thresh_closed_loop=1e2
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

        # FIXED JOINTS MAP init fixed map to zero -> homing from SRDF will be used
        self._wheel_names = [f'j_wheel_{i + 1}' for i in range(4)]
        # ankle_yaws = [f'ankle_yaw_{i + 1}' for i in range(4)]
        # arm_joints = [f'j_arm1_{i + 1}' for i in range(6)] + [f'j_arm2_{i + 1}' for i in range(6)]
        # torso_jnts=["torso_yaw"]
        # head_jnts= ['d435_head_joint', 'velodyne_joint']
        # fixed_joints = self._wheel_names+ankle_yaws+arm_joints+torso_jnts+head_jnts
        # fixed_jnt_vals = len(fixed_joints)*[0.] # default to 0
        # fixed_joint_map=dict(zip(fixed_joints, fixed_jnt_vals))

        self.urdf = self.urdf.replace('continuous', 'revolute') # continous joint is parametrized
        # in So2

        self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf) # used for getting joint names in
        # child class-> will be overritten
        self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())

        self._init_robot_homer()
        
        # fixed_jnts_homing=self._homer.get_homing_vals(jnt_names=fixed_joints)# ordered as fixed_joints
        # update fixed joints map from homing:
        # for i in range(len(fixed_joints)):
        #     jnt_name=fixed_joints[i]
        #     fixed_joint_map[jnt_name]=fixed_jnts_homing[i]
        # self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf)

        self._f0 = [0, 0, self._kin_dyn.mass() / 4 * 9.8]

        init = self._base_init.tolist() + list(self._homer.get_homing())
        FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
        self._wheel_radius = 0.124 # hardcoded!!!!
        ground_level = FK(q=init)['ee_pos']
        self._base_init[2] = -ground_level[2]  # override init     
        # self._base_init[2] += self._wheel_radius # even if in fixed joints, 
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
