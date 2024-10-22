from lrhc_control.controllers.rhc.hybrid_quad_client import HybridQuadrupedClusterClient

from kyonrlstepping.controllers.horizon_based.kyon_rhc import KyonRhc
from kyonrlstepping.utils.xrdf_cmd import get_xrdf_cmds_horizon
from lrhc_control.utils.custom_arg_parsing import extract_custom_xacro_args, merge_xacro_cmds

from typing import List, Dict

class KyonRHCLusterClient(HybridQuadrupedClusterClient):
    
    def __init__(self, 
            namespace: str, 
            cluster_size: int,
            urdf_xacro_path: str,
            srdf_xacro_path: str,
            set_affinity: bool = False,
            use_mp_fork: bool = False,
            isolated_cores_only: bool = False,
            core_ids_override_list: List[int] = None,
            verbose: bool = False,
            debug: bool = False,
            open_loop: bool = True,
            base_dump_dir: str = "/tmp",
            timeout_ms: int = 60000,
            codegen_override: str = "",
            custom_opts: Dict={}):

        super().__init__(namespace = namespace, 
            urdf_xacro_path=urdf_xacro_path,
            srdf_xacro_path=srdf_xacro_path,
            cluster_size=cluster_size,
            isolated_cores_only = isolated_cores_only,
            set_affinity = set_affinity,
            use_mp_fork = use_mp_fork,
            core_ids_override_list = core_ids_override_list,
            verbose = verbose,
            debug = debug,
            open_loop=open_loop,
            base_dump_dir=base_dump_dir,
            timeout_ms=timeout_ms,
            codegen_override=codegen_override,
            custom_opts=custom_opts)
        
        try:
            self._with_wheels = self._custom_opts["with_wheels"]
        except:
            self._with_wheels = False

    def _xrdf_cmds(self):
        parts = self._urdf_xacro_path.split('/')
        urdf_descr_root_path = '/'.join(parts[:-2])
        cmds = get_xrdf_cmds_horizon(urdf_descr_root_path=urdf_descr_root_path)
        return cmds

    def _generate_controller(self,
        idx: int):
        
        codegen_dir=self._process_codegen_dir(idx=idx)

        controller = KyonRhc(
                urdf_path=self._urdf_path, 
                srdf_path=self._srdf_path,
                robot_name=self._namespace,
                codegen_dir=codegen_dir,
                with_wheels=self._with_wheels,
                n_nodes=31, 
                dt=0.03,
                injection_node=5,
                max_solver_iter=1,
                open_loop = self._open_loop,
                close_loop_all=False,
                verbose = self._verbose, 
                debug = self._debug,
                refs_in_hor_frame=True,
                timeout_ms=self._timeout_ms)
        
        return controller 