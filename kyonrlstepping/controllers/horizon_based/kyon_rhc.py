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
        self._files_suffix=""
        if open_loop:
            self._files_suffix="_open"
        config_path=paths.RHCCONFIGPATH_NO_WHEELS+self._files_suffix+".yaml"
        
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
        self._fail_idx_thresh_closed_loop=10

        if open_loop:
            self._fail_idx_thresh=self._fail_idx_thresh_open_loop
        else:
            self._fail_idx_thresh=self._fail_idx_thresh_closed_loop

        # adding some additional config files for jnt imp control
        self._rhc_fpaths.append(paths.JNT_IMP_CONFIG_XBOT+".yaml")
        self._rhc_fpaths.append(paths.JNT_IMP_CONFIG+".yaml")
            
    def _set_rhc_pred_idx(self):
        self._pred_node_idx=round((self._n_nodes-1)*2/3)
    
    def _set_rhc_cmds_idx(self):
        self._rhc_cmds_node_idx=2

    def _config_override(self):
        paths = PathsGetter()
        if ("wheels" in self._custom_opts) and \
            ("true" in self._custom_opts["wheels"] or \
            "True" in self._custom_opts["wheels"]):
            self.config_path = paths.RHCCONFIGPATH_WHEELS+self._files_suffix+".yaml"
            if ("replace_continuous_joints" in self._custom_opts) and \
                (not self._custom_opts["replace_continuous_joints"]):
                # use continuous joints -> different config
                self.config_path = paths.RHCCONFIGPATH_WHEELS_CONTINUOUS+self._files_suffix+".yaml"

    def _init_problem(self):
        
        flight_duration_sec=0.6 # [s]
        flight_duration=int(flight_duration_sec/self._dt)
        post_flight_duration_sec=0.2 # [s]
        post_flight_duration=int(post_flight_duration_sec/self._dt)
        super()._init_problem(fixed_jnt_patterns=None,
            wheels_patterns=["wheel_"],
            foot_linkname="ball_1",
            flight_duration=flight_duration,
            post_flight_stance=post_flight_duration,
            step_height=0.1,
            keep_yaw_vert=False,
            yaw_vertical_weight=1.0,
            phase_force_reg=2e-2,
            vel_bounds_weight=1.0)