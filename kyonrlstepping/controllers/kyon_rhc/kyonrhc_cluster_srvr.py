from control_cluster_utils.cluster_server.control_cluster_srvr import ControlClusterSrvr

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_utils import KyonClusterCmd, JntImpCntrlAction

class KyonRHClusterSrvr(ControlClusterSrvr):
    
    def __init__(self, 
                wheels = True, 
                arms = False, 
                payload = False):

        self._temp_path = "/tmp/" + f"{self.__class__.__name__}"
                
        super().__init__()
        
        self._srdf_path = self._generate_srdf(wheels=wheels, 
                            arms=arms, 
                            payload=payload)
        
        self._urdf_path = self._generate_urdf(wheels=wheels, 
                        arms=arms)
    
    def _generate_srdf(self,
                    wheels: bool = True, 
                    arms: bool = False, 
                    payload: bool = False):
        
        print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generating SRDF for Control Cluster server")

        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()

        kyon_xacro_name = "kyon"

        srdf_path = self._temp_path + "/" + kyon_xacro_name + ".srdf"

        xacro_path = rospackage.get_path("kyon_srdf") + "/srdf/" + kyon_xacro_name + ".srdf.xacro"
        
        wheel_cmd_val = "true"
        arms_cmd_val = "false"
        payload_cmd_val = "false"
        if wheels:
            wheel_cmd_val = "true"
        else:
            wheel_cmd_val = "false"
        if arms:
            arms_cmd_val = "true"
        else:
            arms_cmd_val = "false"
        if payload:
            payload_cmd_val = "true"
        else:
            payload_cmd_val = "false"

        add_wheels = "wheels:=" + wheel_cmd_val
        add_arms = "upper_body:=" + arms_cmd_val
        remove_sensors = "sensors:=" + "false"
        keep_floating_joint = "floating_joint:=" + "true"
        no_payload = "payload:=" + payload_cmd_val
        
        import subprocess
        try:

            xacro_gen = subprocess.check_call(["xacro",\
                                            xacro_path, \
                                            add_wheels, \
                                            add_arms, \
                                            remove_sensors, \
                                            no_payload, \
                                            keep_floating_joint, \
                                            "-o", 
                                            srdf_path])

            print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generated SRDF for Control Cluster server")

            return srdf_path

        except:

            raise Exception(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": failed to generate Kyon\'s SRDF!!!.")
    
    def _generate_urdf(self, 
                    wheels: bool = True, 
                    arms: bool = False):
        
        print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generating URDF for Control Cluster server")

        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()

        kyon_xacro_name = "kyon"

        urdf_path = self._temp_path + "/" + kyon_xacro_name + ".urdf"

        xacro_path = rospackage.get_path("kyon_urdf") + "/urdf/" + kyon_xacro_name + ".urdf.xacro"
        
        wheel_cmd_val = "true"
        arms_cmd_val = "false"
        if wheels:
            wheel_cmd_val = "true"
        else:
            wheel_cmd_val = "false"
        if arms:
            arms_cmd_val = "true"
        else:
            arms_cmd_val = "false"
        add_wheels = "wheels:=" + wheel_cmd_val
        add_arms = "upper_body:=" + arms_cmd_val

        keep_floating_joint = "floating_joint:=" + "true"
        remove_sensors = "sensors:=" + "false"
        no_payload = "payload:=" + "false"

        import subprocess
        try:

            xacro_gen = subprocess.check_call(["xacro",\
                                            xacro_path, \
                                            add_wheels, \
                                            add_arms, \
                                            keep_floating_joint, \
                                            remove_sensors, \
                                            no_payload, \
                                            "-o", 
                                            urdf_path])
            
            print(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": generated URDF for Control Cluster server")

            return urdf_path

        except:

            raise Exception(f"[{self.__class__.__name__}]"  + f"[{self.status}]" + ": failed to generate Kyon\'s URDF!!!.")
        
    def start(self):

        super().start()

        self.ctrl_action = JntImpCntrlAction(n_jnts = self.n_dofs, 
                                        cluster_size = self.cluster_size, 
                                        device = self._device)
        
    def set_commands(self, 
                cluster_cmd: KyonClusterCmd):
        
        pass
    
    def _check_cmd_size(self, 
                    cluster_cmd: KyonClusterCmd):
        
        pass
    
    def _synch_cluster_from_controllers(self):

        pass

    def _synch_controllers_from_cluster(self):

        pass

    def get(self):

        return self.ctrl_action