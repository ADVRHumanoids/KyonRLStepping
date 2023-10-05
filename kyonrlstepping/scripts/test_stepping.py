from control_cluster_utils.utilities.keyboard_cmds import RhcRefsFromKeyboard

if __name__ == "__main__":  

    keyb_cmds = RhcRefsFromKeyboard(namespace="kyon0", 
                            verbose=True)

    keyb_cmds.start()