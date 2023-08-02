from control_cluster_utils.cluster_server.control_cluster_srvr import ControlClusterSrvr

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_utils import KyonClusterCmd, JntImpCntrlAction

from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_horizon

class KyonRHClusterSrvr(ControlClusterSrvr):
    
    def __init__(self):

        self._temp_path = "/tmp/" + f"{self.__class__.__name__}"
                
        super().__init__()
        
        self.robot_name = "kyon"

        self._generate_srdf()

        self._generate_urdf()
        
    
    def _xrdf_cmds(self):
        
        cmds = get_xrdf_cmds_horizon()

        return cmds
        
    def _generate_srdf(self):
        
        print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generating SRDF for Control Cluster server")

        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        xacro_name = self.robot_name
        self._srdf_path = self._temp_path + "/" + xacro_name + ".srdf"
        xacro_path = rospackage.get_path(self.robot_name + "_srdf") + "/srdf/" + xacro_name + ".srdf.xacro"
        
        cmds = self._xrdf_cmds()
        if cmds is None:

            cmds = []
        
        import subprocess
        try:

            xacro_cmd = ["xacro"] + [xacro_path] + cmds + ["-o"] + [self._srdf_path]
            xacro_gen = subprocess.check_call(xacro_cmd)

            print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generated SRDF for Control Cluster server")

        except:

            raise Exception(f"[{self.__class__.__name__}]"  + 
                            f"[{self.status}]" + 
                            ": failed to generate Kyon\'s SRDF!!!.")
    
    def _generate_urdf(self):
        
        print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generating URDF for Control Cluster server")

        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        xacro_name = self.robot_name
        self._urdf_path = self._temp_path + "/" + xacro_name + ".urdf"
        xacro_path = rospackage.get_path(self.robot_name + "_urdf") + "/urdf/" + xacro_name + ".urdf.xacro"
        
        cmds = self._xrdf_cmds()
        if cmds is None:

            cmds = []

        import subprocess
        try:
            
            xacro_cmd = ["xacro"] + [xacro_path] + cmds + ["-o"] + [self._urdf_path]

            xacro_gen = subprocess.check_call(xacro_cmd)
            
            print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generated URDF for Control Cluster server")

        except:

            raise Exception(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": failed to generate Kyon\'s URDF!!!.")
    