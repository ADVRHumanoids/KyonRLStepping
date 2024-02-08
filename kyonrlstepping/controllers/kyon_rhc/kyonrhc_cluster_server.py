from control_cluster_bridge.cluster_server.control_cluster_server import ControlClusterServer
from typing import List

class KyonRhcClusterServer(ControlClusterServer):

    def __init__(self, 
            cluster_size, 
            device, 
            cluster_dt, 
            control_dt, 
            jnt_names,
            np_array_dtype, 
            verbose, 
            debug,
            n_contact_sensors: int = -1,
            contact_linknames: List[str] = None, 
            robot_name = "kyon0"):

        self.robot_name = robot_name
                
        super().__init__( 
            cluster_size=cluster_size, 
            device=device, 
            cluster_dt=cluster_dt, 
            control_dt=control_dt, 
            jnt_names=jnt_names,
            n_contact_sensors = n_contact_sensors,
            contact_linknames = contact_linknames, 
            np_array_dtype=np_array_dtype, 
            verbose=verbose, 
            debug=debug,
            namespace=self.robot_name)