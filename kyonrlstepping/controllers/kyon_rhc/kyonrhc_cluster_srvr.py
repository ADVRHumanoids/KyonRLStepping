from control_cluster_bridge.cluster_server.control_cluster_srvr import ControlClusterSrvr

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_horizon

from SharsorIPCpp.PySharsorIPC import Journal, LogType

from typing import List

class KyonRHClusterSrvr(ControlClusterSrvr):
    
    def __init__(self, 
            robot_name: str, 
            isolated_cores_only: bool = False,
            use_only_physical_cores: bool = False,
            core_ids_override_list: List[int] = None,
            verbose: bool = False):

        self._temp_path = "/tmp/" + f"{self.__class__.__name__}"
        
        self.namespace = robot_name

        self.robot_pkg_name = "kyon"

        super().__init__(namespace = self.namespace, 
                        isolated_cores_only = isolated_cores_only,
                        use_only_physical_cores = use_only_physical_cores,
                        core_ids_override_list = core_ids_override_list,
                        verbose = verbose)
        
        self._generate_srdf()

        self._generate_urdf()
        
    def _xrdf_cmds(self):
        
        cmds = get_xrdf_cmds_horizon(robot_pkg_name = self.robot_pkg_name)

        return cmds
        
    def _generate_srdf(self):
        
        Journal.log(self.__class__.__name__,
                        "_generate_srdf",
                        "generating SRDF for Control Cluster server",
                        LogType.STAT,
                        throw_when_excep = True)
    
        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        xacro_name = self.robot_pkg_name
        self._srdf_path = self._temp_path + "/" + xacro_name + ".srdf"
        xacro_path = rospackage.get_path(self.robot_pkg_name + "_srdf") + "/srdf/" + xacro_name + ".srdf.xacro"
        
        cmds = self._xrdf_cmds()
        if cmds is None:

            cmds = []
        
        import subprocess
        try:

            xacro_cmd = ["xacro"] + [xacro_path] + cmds + ["-o"] + [self._srdf_path]
            xacro_gen = subprocess.check_call(xacro_cmd)

            Journal.log(self.__class__.__name__,
                        "_generate_srdf",
                        "generated SRDF for Control Cluster server",
                        LogType.STAT,
                        throw_when_excep = True)
            
        except:
            
            Journal.log(self.__class__.__name__,
                        "_generate_srdf",
                        "Failed to generate Kyon\'s SRDF!!!.",
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def _generate_urdf(self):
        
        Journal.log(self.__class__.__name__,
                        "_generate_urdf",
                        "Generating URDF for Control Cluster server",
                        LogType.STAT,
                        throw_when_excep = True)
        
        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        xacro_name = self.robot_pkg_name
        self._urdf_path = self._temp_path + "/" + xacro_name + ".urdf"
        xacro_path = rospackage.get_path(self.robot_pkg_name + "_urdf") + "/urdf/" + xacro_name + ".urdf.xacro"
        
        cmds = self._xrdf_cmds()
        if cmds is None:

            cmds = []

        import subprocess
        try:
            
            xacro_cmd = ["xacro"] + [xacro_path] + cmds + ["-o"] + [self._urdf_path]

            xacro_gen = subprocess.check_call(xacro_cmd)
            
            Journal.log(self.__class__.__name__,
                        "_generate_srdf",
                        "Generated URDF for Control Cluster server",
                        LogType.STAT,
                        throw_when_excep = True)
            
        except:

            Journal.log(self.__class__.__name__,
                        "_generate_urdf",
                        "Failed to generate Kyon\'s URDF!!!.",
                        LogType.EXCEP,
                        throw_when_excep = True)    