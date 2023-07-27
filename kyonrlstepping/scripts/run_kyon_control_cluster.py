from kyon_rhc.kyonrhc import KyonRHC
from kyon_rhc.kyonrhc_cluster_srvr import KyonRHClusterSrvr

def generate_controllers():

    # create controllers
    cluster_controllers = []
    for i in range(0, control_cluster_srvr.cluster_size):

        cluster_controllers.append(KyonRHC(
                                    controller_index = i,
                                    urdf_path=control_cluster_srvr._urdf_path, 
                                    srdf_path=control_cluster_srvr._srdf_path,
                                    config_path = "/home/apatrizi/RL_ws/kyon/src/kyon_controller/python/KyonRHC/kyon_rhc/config/kyon_horizon_wheel_config.yaml", 
                                    pipes_manager = control_cluster_srvr.pipes_manager, 
                                    verbose = verbose, 
                                    name = "KyonRHController" + str(i), 
                                    termination_flag = control_cluster_srvr.termination_flag))
    
    return cluster_controllers

verbose = True
control_cluster_srvr = KyonRHClusterSrvr(pipes_config_path = 
            "/home/apatrizi/RL_ws/kyon/src/ControlClusterUtils/control_cluster_utils/config/pipes/pipes_config.yaml")
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
