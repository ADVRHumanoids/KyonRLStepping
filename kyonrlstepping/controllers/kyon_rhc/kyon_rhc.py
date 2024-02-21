from lrhc_control.controllers.rhc.horizon_based.horizon_imports import * 

from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc

import torch

class KyonRhc(HybridQuadRhc):

    def __init__(self, 
            srdf_path: str,
            urdf_path: str,
            config_path: str,
            cluster_size: int, # needed by shared mem manager
            robot_name: str, # used for shared memory namespaces
            codegen_dir: str, 
            n_nodes:float = 25,
            dt: float = 0.02,
            max_solver_iter = 1, # defaults to rt-iteration
            add_data_lenght: int = 2,
            enable_replay = False, 
            verbose = False, 
            debug = False, 
            profile_all = False,
            array_dtype = torch.float32, 
            publish_sol = False,
            debug_sol = True, # whether to publish rhc rebug data,
            solver_deb_prints = False,
            ):

        super().__init__(srdf_path=srdf_path,
            urdf_path=urdf_path,
            config_path=config_path,
            cluster_size=cluster_size, # needed by shared mem manager
            robot_name=robot_name, # used for shared memory namespaces
            codegen_dir=codegen_dir, 
            n_nodes=n_nodes,
            dt=dt,
            max_solver_iter=max_solver_iter, # defaults to rt-iteration
            add_data_lenght=add_data_lenght,
            enable_replay=enable_replay, 
            verbose=verbose, 
            debug=debug, 
            profile_all=profile_all,
            array_dtype=array_dtype,
            publish_sol=publish_sol,
            debug_sol=debug_sol, # whether to publish rhc rebug data,
            solver_deb_prints=solver_deb_prints)