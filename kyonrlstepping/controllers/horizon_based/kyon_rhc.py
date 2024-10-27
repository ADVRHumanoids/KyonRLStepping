from lrhc_control.controllers.rhc.horizon_based.horizon_imports import * 
from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc

import numpy as np

from typing import Dict 

from kyonrlstepping.utils.sysutils import PathsGetter

class KyonRhc(HybridQuadRhc):

    def __init__(self, 
            srdf_path: str,
            urdf_path: str,
            robot_name: str, # used for shared memory namespaces
            codegen_dir: str,
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
        config_path=paths.RHCCONFIGPATH_NO_WHEELS
        if ("wheels" in custom_opts) and \
            ("true" in custom_opts["wheels"] or \
            "True" in custom_opts["wheels"]):
            config_path = paths.RHCCONFIGPATH_WHEELS
            if ("replace_continuous_joints" in custom_opts) and \
                (not custom_opts["replace_continuous_joints"]):
                # use continuous joints -> different config
                config_path = paths.RHCCONFIGPATH_WHEELS_CONTINUOUS
        
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
    
    def _init_problem(self):
        
        super()._init_problem(fixed_jnt_patterns=None,
            foot_linkname="ball_1",
            flight_duration=15,
            post_landing_stance=5,
            step_height=0.1,
            keep_yaw_vert=False,
            yaw_vertical_weight=2.0,
            phase_force_reg=1e-2,
            vel_bounds_weight=1.0)
        
    # def _init_problem(self):
        
    #     vel_bounds_weight=1.0
    #     meas_state_attractor_weight=100.0
    #     self._phase_force_reg=1e-2
    #     self._yaw_vertical_weight=2.0
    #     # overrides parent
    #     self._prb = Problem(self._n_intervals, 
    #                     receding=True, 
    #                     casadi_type=cs.SX)
    #     self._prb.setDt(self._dt)

    #     if self._custom_opts["replace_continuous_joints"]:
    #         # continous joints are parametrized in So2
    #         self.urdf = self.urdf.replace('continuous', 'revolute')
    #     self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf) # used for getting joint names 
    #     self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())
        
    #     self._continuous_joints=self._get_continuous_jnt_names()
    #     # reduced
    #     self._continuous_joints_idxs=[]
    #     self._continuous_joints_idxs_cos=[]
    #     self._continuous_joints_idxs_sin=[]
    #     self._continuous_joints_idxs_red=[]
    #     self._rev_joints_idxs=[]
    #     self._rev_joints_idxs_red=[]
    #     # qfull
    #     self._continuous_joints_idxs_qfull=[]
    #     self._continuous_joints_idxs_cos_qfull=[]
    #     self._continuous_joints_idxs_sin_qfull=[]
    #     self._continuous_joints_idxs_red_qfull=[]
    #     self._rev_joints_idxs_qfull=[]
    #     self._rev_joints_idxs_red_qfull=[]
    #     wheel_pattern = r"wheel_joint"
    #     are_there_wheels=any(re.search(wheel_pattern, s) for s in self._get_robot_jnt_names())
        
    #     self._init_robot_homer()

    #     if (not are_there_wheels) and (self._with_wheels):
    #         # not allowed
    #         Journal.log(self.__class__.__name__,
    #             "_init_problem",
    #             f"Provided _with_wheels arg was set to True, but wheels not found in URDF!",
    #             LogType.EXCEP,
    #             throw_when_excep=True)
    #     fixed_joint_map={}
    #     if are_there_wheels and (not self._with_wheels): # we need to fix some joints
    #         fixed_joints = [f'{wheel_pattern}_{i + 1}' for i in range(4)]
    #         fixed_jnt_vals = len(fixed_joints)*[0.] # default to 0
    #         fixed_joint_map=dict(zip(fixed_joints, fixed_jnt_vals))
    #         fixed_jnts_homing=self._homer.get_homing_vals(jnt_names=fixed_joints)# ordered as fixed_joints
    #         # update fixed joints map from homing:
    #         for i in range(len(fixed_joints)):
    #             jnt_name=fixed_joints[i]
    #             fixed_joint_map[jnt_name]=fixed_jnts_homing[i]

    #     self._using_wheels=False
    #     if are_there_wheels and self._with_wheels:
    #         self._using_wheels=True
    #     if not len(fixed_joint_map)==0: # we need to recreate kin dyn and homers
    #         self._kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf,fixed_joints=fixed_joint_map)
    #         self._assign_controller_side_jnt_names(jnt_names=self._get_robot_jnt_names())
    #         self._init_robot_homer()
        
    #     self._wheel_jnts_idxs=[]
    #     jnt_homing=[""]*(len(self._homer.get_homing())+len(self._continuous_joints))
    #     jnt_names=self._get_robot_jnt_names()
    #     for i in range(len(jnt_names)):
    #         jnt=jnt_names[i]
    #         index=self._get_jnt_id(jnt)# accounting for floating joint
    #         homing_idx=index-7 # homing is only for actuated joints
    #         if re.search(wheel_pattern, jnt): # found wheel joint
    #             self._wheel_jnts_idxs.append(index)
    #         homing_value=self._homer.get_homing_val(jnt)
    #         if jnt in self._continuous_joints:
    #             jnt_homing[homing_idx]=np.cos(homing_value).item()
    #             jnt_homing[homing_idx+1]=np.sin(homing_value).item()
    #             # just actuated joints
    #             self._continuous_joints_idxs.append(homing_idx) # cos
    #             self._continuous_joints_idxs.append(homing_idx+1) # sin
    #             self._continuous_joints_idxs_cos.append(homing_idx)
    #             self._continuous_joints_idxs_sin.append(homing_idx+1)
    #             self._continuous_joints_idxs_red.append(i)
    #             # q full
    #             self._continuous_joints_idxs_qfull.append(index) # cos
    #             self._continuous_joints_idxs_qfull.append(index+1) # sin
    #             self._continuous_joints_idxs_cos_qfull.append(index)
    #             self._continuous_joints_idxs_sin_qfull.append(index+1)
    #             self._continuous_joints_idxs_red_qfull.append(i+7)
    #         else:
    #             jnt_homing[homing_idx]=homing_value
    #             # just actuated joints
    #             self._rev_joints_idxs.append(homing_idx) 
    #             self._rev_joints_idxs_red.append(i) 
    #             # q full
    #             self._rev_joints_idxs_qfull.append(index) 
    #             self._rev_joints_idxs_red_qfull.append(i+7) 

    #     self._jnts_q_reduced=None
    #     if not len(self._continuous_joints)==0:
    #         self._jnts_q_reduced=np.zeros((1,self.nv()-6),dtype=self._dtype)
    #         self._jnts_q_expanded=np.zeros((1,self.nq()-7),dtype=self._dtype)
    #         self._full_q_reduced=np.zeros((7+len(jnt_names), self._n_nodes),dtype=self._dtype)

    #     self._f0 = [0, 0, self._kin_dyn.mass() / 4 * 9.8]

    #     init = self._base_init.tolist() + jnt_homing

    #     FK = self._kin_dyn.fk('ball_1') # just to get robot reference height
    #     self._wheel_radius = 0.124 # hardcoded!!!!
    #     ground_level = FK(q=init)['ee_pos']
    #     self._base_init[2] = -ground_level[2]  # override init     
    #     # if are_there_wheels:
    #     #     self._base_init[2] += self._wheel_radius # even if in fixed joints, 
    #     # in the real robot the wheel is there. This way the feet z in homing is at height
        
    #     self._model = FullModelInverseDynamics(problem=self._prb,
    #         kd=self._kin_dyn,
    #         q_init=self._homer.get_homing_map(),
    #         base_init=self._base_init)

    #     self._ti = TaskInterface(prb=self._prb, 
    #                         model=self._model, 
    #                         max_solver_iter=self.max_solver_iter,
    #                         debug = self._debug,
    #                         verbose = self._verbose, 
    #                         codegen_workdir = self._codegen_dir)
    #     self._ti.setTaskFromYaml(self.config_path)
        
    #     # setting initial base pos ref
    #     base_pos = self._ti.getTask('base_height')
    #     if base_pos is not None:
    #         base_pos.setRef(np.atleast_2d(self._base_init).T)

    #     self._tg = trajectoryGenerator.TrajectoryGenerator()

    #     self._pm = pymanager.PhaseManager(self._n_nodes, debug=False) # intervals or nodes?????

    #     # self._create_whitelist()
    #     self._init_contact_timelines()
    #     # self._add_zmp()

    #     self._ti.model.q.setBounds(self._ti.model.q0, self._ti.model.q0, nodes=0)
    #     self._ti.model.v.setBounds(self._ti.model.v0, self._ti.model.v0, nodes=0)
    #     self._ti.model.q.setInitialGuess(self._ti.model.q0)
    #     self._ti.model.v.setInitialGuess(self._ti.model.v0)
    #     for _, cforces in self._ti.model.cmap.items():
    #         n_contact_f=len(cforces)
    #         for c in cforces:
    #             c.setInitialGuess(np.array(self._f0)/n_contact_f)        

    #     vel_lims = self._model.kd.velocityLimits()
    #     import horizon.utils as utils
    #     self._prb.createResidual('vel_lb_barrier', vel_bounds_weight*utils.utils.barrier(vel_lims[7:] - self._model.v[7:]))
    #     self._prb.createResidual('vel_ub_barrier', vel_bounds_weight*utils.utils.barrier1(-1 * vel_lims[7:] - self._model.v[7:]))

    #     # if not self._open_loop:
    #     #     # we create a residual cost to be used as an attractor to the measured state on the first node
    #     #     # hard constraints injecting meas. states are pure EVIL!
    #     #     prb_state=self._prb.getState()
    #     #     full_state=prb_state.getVars()
    #     #     state_dim=prb_state.getBounds()[0].shape[0]
    #     #     meas_state=self._prb.createParameter(name="measured_state",
    #     #         dim=state_dim, nodes=0)     
    #     #     self._prb.createResidual('meas_state_attractor', meas_state_attractor_weight * (full_state - meas_state), 
    #     #                 nodes=[0])

    #     self._ti.finalize()
    #     self._ti.bootstrap()

    #     self._ti.init_inv_dyn_for_res() # we initialize some objects for sol. postprocessing purposes
    #     self._ti.load_initial_guess()

    #     contact_phase_map = {c: f'{c}_timeline' for c in self._model.cmap.keys()}
        
    #     self._gm = GaitManager(self._ti, self._pm, contact_phase_map, self._injection_node)

    #     self.n_dofs = self._get_ndofs() # after loading the URDF and creating the controller we
    #     # know n_dofs -> we assign it (by default = None)

    #     self.n_contacts = len(self._model.cmap.keys())
        
    #     # self.horizon_anal = analyzer.ProblemAnalyzer(self._prb)

