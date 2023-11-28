from kyonrlstepping.utils.xrdf_gen import get_xrdf_cmds_horizon
from rhcviz.utils.xrdf_gen import UrdfGenerator

class KyonUrdfGen(UrdfGenerator):

    def __init__(self, 
            robotname: str,
            name: str = "KyonUrdfRHCViz"):
        
        super().__init__(
            robotname = robotname,
            name = name)

        self.generate_urdf() # actually generated urdf

    def _xrdf_cmds(self):
        
        # implements parent method 

        cmds = {} 
        
        cmds[self.robotname] = self._get_xrdf_cmds_kyon()
        
        return cmds
    
    def _get_xrdf_cmds_kyon(self):
        
        cmds = []
        
        xrdf_cmd_vals = [True, False, False, False, False, False] # horizon needs 
        # the floating base

        wheels = "true" if xrdf_cmd_vals[0] else "false"
        upper_body = "true" if xrdf_cmd_vals[1] else "false"
        gripper = "true" if xrdf_cmd_vals[2] else "false"
        sensors = "true" if xrdf_cmd_vals[3] else "false"
        floating_joint = "true" if xrdf_cmd_vals[4] else "false"
        payload = "true" if xrdf_cmd_vals[5] else "false"
                
        cmds.append("wheels:=" + wheels)
        cmds.append("upper_body:=" + upper_body)
        cmds.append("dagana:=" + gripper)
        cmds.append("sensors:=" + sensors)
        cmds.append("floating_joint:=" + floating_joint)
        cmds.append("payload:=" + payload)

        return cmds
