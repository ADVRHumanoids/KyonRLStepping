from control_cluster_bridge.cluster_client.control_cluster_client import ControlClusterClient

class KyonRHClusterClient(ControlClusterClient):

    def __init__(self, 
            cluster_size, 
            device, 
            cluster_dt, 
            control_dt, 
            jnt_names, 
            np_array_dtype, 
            verbose, 
            debug,
            robot_name = "kyon0"):

        self.robot_name = robot_name
                
        super().__init__( 
            cluster_size=cluster_size, 
            device=device, 
            cluster_dt=cluster_dt, 
            control_dt=control_dt, 
            jnt_names=jnt_names, 
            np_array_dtype=np_array_dtype, 
            verbose=verbose, 
            debug=debug,
            namespace=self.robot_name)
