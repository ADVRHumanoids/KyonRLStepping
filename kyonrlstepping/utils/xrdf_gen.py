def get_xrdf_cmds_isaac():

        cmds = []
        
        xrdf_cmd_vals = [True, False, False, False, False]

        wheels = "true" if xrdf_cmd_vals[0] else "false"
        upper_body = "true" if xrdf_cmd_vals[1] else "false"
        sensors = "true" if xrdf_cmd_vals[2] else "false"
        floating_joint = "true" if xrdf_cmd_vals[3] else "false"
        payload = "true" if xrdf_cmd_vals[4] else "false"

        cmds.append("wheels:=" + wheels)
        cmds.append("upper_body:=" + upper_body)
        cmds.append("sensors:=" + sensors)
        cmds.append("floating_joint:=" + floating_joint)
        cmds.append("payload:=" + payload)

        return cmds

def get_xrdf_cmds_horizon():

        cmds = []
        
        xrdf_cmd_vals = [True, False, False, True, False] # horizon needs 
        # the floating base

        wheels = "true" if xrdf_cmd_vals[0] else "false"
        upper_body = "true" if xrdf_cmd_vals[1] else "false"
        sensors = "true" if xrdf_cmd_vals[2] else "false"
        floating_joint = "true" if xrdf_cmd_vals[3] else "false"
        payload = "true" if xrdf_cmd_vals[4] else "false"

        cmds.append("wheels:=" + wheels)
        cmds.append("upper_body:=" + upper_body)
        cmds.append("sensors:=" + sensors)
        cmds.append("floating_joint:=" + floating_joint)
        cmds.append("payload:=" + payload)

        return cmds