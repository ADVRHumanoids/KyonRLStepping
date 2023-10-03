def get_xrdf_cmds_isaac(n_robots: int, 
                basename = "kyon"):

        cmds = {}
        cmds_aux = []
        
        xrdf_cmd_vals = [True, False, False, False, False]

        wheels = "true" if xrdf_cmd_vals[0] else "false"
        upper_body = "true" if xrdf_cmd_vals[1] else "false"
        sensors = "true" if xrdf_cmd_vals[2] else "false"
        floating_joint = "true" if xrdf_cmd_vals[3] else "false"
        payload = "true" if xrdf_cmd_vals[4] else "false"

        cmds_aux.append("wheels:=" + wheels)
        cmds_aux.append("upper_body:=" + upper_body)
        cmds_aux.append("sensors:=" + sensors)
        cmds_aux.append("floating_joint:=" + floating_joint)
        cmds_aux.append("payload:=" + payload)

        for i in range(n_robots):
                # we use the same settings for all robots
                cmds[basename + str(i)] = cmds_aux

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