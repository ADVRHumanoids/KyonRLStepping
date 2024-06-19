from lrhc_control.controllers.rhc.lrhc_cluster_client import LRhcClusterClient
from lrhc_control.controllers.rhc.hybrid_quad_client import HybridQuadrupedClusterClient

from kyonrlstepping.controllers.horizon_based.kyon_rhc import KyonRhc
from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_horizon

from typing import List

class KyonRHCLusterClient(HybridQuadrupedClusterClient):
    
    def __init__(self, 
            namespace: str, 
            cluster_size: int,
            set_affinity: bool = False,
            use_mp_fork: bool = False,
            isolated_cores_only: bool = False,
            core_ids_override_list: List[int] = None,
            verbose: bool = False,
            debug: bool = False,
            open_loop: bool = True,
            with_wheels: bool = False,
            base_dump_dir: str = "/tmp",
            timeout_ms: int = 60000):
        
        self._with_wheels = with_wheels

        super().__init__(namespace = namespace, 
            robot_pkg_name="kyon",
            cluster_size=cluster_size,
            isolated_cores_only = isolated_cores_only,
            set_affinity = set_affinity,
            use_mp_fork = use_mp_fork,
            core_ids_override_list = core_ids_override_list,
            verbose = verbose,
            debug = debug,
            open_loop=open_loop,
            base_dump_dir=base_dump_dir,
            timeout_ms=timeout_ms)

    def _xrdf_cmds(self):
        cmds = get_xrdf_cmds_horizon(robot_pkg_name = self.robot_pkg_name,
                            with_wheels = self._with_wheels)
        return cmds

    def _generate_controller(self,
        idx: int):
        
        controller = KyonRhc(
                urdf_path=self._urdf_path, 
                srdf_path=self._srdf_path,
                robot_name=self._namespace,
                codegen_dir=self.codegen_dir() + f"/{self._codegen_dir_name}Rhc{idx}",
                with_wheels=self._with_wheels,
                n_nodes=31, 
                dt=0.03,
                injection_node=5,
                max_solver_iter = 1,
                open_loop = self._open_loop,
                verbose = self._verbose, 
                debug = self._debug,
                refs_in_hor_frame=True,
                timeout_ms=self._timeout_ms)
        
        return controller 