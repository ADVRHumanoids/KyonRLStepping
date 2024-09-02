from typing import List
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import Journal, LogType
           
def get_xrdf_cmds_horizon(urdf_descr_root_path : str = None):
        return get_xrdf_cmds_horizon_kyon(urdf_descr_root_path=urdf_descr_root_path)

def get_xrdf_cmds_horizon_kyon(urdf_descr_root_path: str = None):

        cmds = []
        
        xrdf_cmd_vals = [False, False, False, False, True, False] # horizon needs 
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
        
        if urdf_descr_root_path is not None:
                cmds.append("root:=" + urdf_descr_root_path)

        return cmds