from control_cluster_bridge.utilities.keyboard_cmds import RhcRefsFromKeyboard

if __name__ == "__main__":  

    keyb_cmds = RhcRefsFromKeyboard(namespace="kyon0", 
                            verbose=True)

    keyb_cmds.run()