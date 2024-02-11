from lrhc_control.controllers.rhc.lrhc_cluster_client import LRhcClusterClient

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_horizon

from typing import List

class KyonLRhcClusterClient(LRhcClusterClient):
    
    def __init__(self, 
            namespace: str, 
            cluster_size: int,
            isolated_cores_only: bool = False,
            use_only_physical_cores: bool = False,
            core_ids_override_list: List[int] = None,
            verbose: bool = False):
        
        robot_pkg_name = "kyon"

        super().__init__(namespace = namespace, 
                        robot_pkg_name = robot_pkg_name,
                        cluster_size=cluster_size,
                        isolated_cores_only = isolated_cores_only,
                        use_only_physical_cores = use_only_physical_cores,
                        core_ids_override_list = core_ids_override_list,
                        verbose = verbose)
    
        
    def _xrdf_cmds(self):
        
        cmds = get_xrdf_cmds_horizon(robot_pkg_name = self.robot_pkg_name)

        return cmds