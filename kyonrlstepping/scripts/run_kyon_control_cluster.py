from kyon_rhc.kyonrhc import KyonRHC
from kyon_rhc.kyonrhc_cluster_srvr import KyonRHClusterSrvr

def generate_controllers():

    # create controllers
    cluster_controllers = []
    for i in range(0, control_cluster_srvr.cluster_size):

        cluster_controllers.append(KyonRHC(
                                    trigger_pipename = control_cluster_srvr.trigger_pipenames[i],
                                    success_pipename = control_cluster_srvr.success_pipenames[i], 
                                    cmd_jnt_q_pipename = control_cluster_srvr.cmd_jnt_q_pipenames[i],
                                    cmd_jnt_v_pipename = control_cluster_srvr.cmd_jnt_v_pipenames[i],
                                    cmd_jnt_eff_pipename = control_cluster_srvr.cmd_jnt_eff_pipenames[i],
                                    rhc_info_pipename = control_cluster_srvr.rhc_info_pipenames[i], 
                                    state_root_q_pipename = control_cluster_srvr.state_root_q_pipenames[i], 
                                    state_root_v_pipename = control_cluster_srvr.state_root_v_pipenames[i], 
                                    state_jnt_q_pipename = control_cluster_srvr.state_jnt_q_pipenames[i], 
                                    state_jnt_v_pipename = control_cluster_srvr.state_jnt_v_pipenames[i], 
                                    urdf_path=control_cluster_srvr._urdf_path, 
                                    srdf_path=control_cluster_srvr._srdf_path,
                                    config_path = "/home/apatrizi/RL_ws/kyon/src/kyon_controller/python/KyonRHC/kyon_rhc/config/kyon_horizon_wheel_config.yaml", 
                                    verbose = verbose, 
                                    name = "KyonRHController" + str(i), 
                                    termination_flag = control_cluster_srvr.termination_flag))
    
    return cluster_controllers

cmd_size = 2
verbose = True

control_cluster_srvr = KyonRHClusterSrvr(cmd_size = cmd_size)
controllers = generate_controllers()

for i in range(0, control_cluster_srvr.cluster_size):
    
    # we add the controllers

    result = control_cluster_srvr.add_controller(controllers[i])

control_cluster_srvr.start() 

try:

    while True:
        
        pass

except KeyboardInterrupt:

    # This block will execute when Control-C is pressed
    control_cluster_srvr.terminate() # closes all processes

    import sys
    sys.exit()
