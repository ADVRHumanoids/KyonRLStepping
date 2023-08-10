import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

from kyonrlstepping.controllers.kyon_rhc.kyonrhc import KyonRHC
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_srvr import KyonRHClusterSrvr
from kyonrlstepping.controllers.kyon_rhc.utils.sysutils import PathsGetter
kyonrhc_paths = PathsGetter

import torch

import time 

def generate_controllers():

    kyonrhc_config_path = kyonrhc_paths().CONFIGPATH

    # create controllers
    cluster_controllers = []
    for i in range(0, control_cluster_srvr.cluster_size):

        cluster_controllers.append(KyonRHC(
                                    controller_index = i,
                                    urdf_path=control_cluster_srvr._urdf_path, 
                                    srdf_path=control_cluster_srvr._srdf_path,
                                    cluster_size=control_cluster_srvr.cluster_size,
                                    config_path = kyonrhc_config_path, 
                                    verbose = verbose, 
                                    debug = debug,
                                    array_dtype = dtype))
    
    return cluster_controllers

verbose = True
debug = True

dtype = torch.float32 # this has to be the same wrt the cluster client, otherwise
# messages are not read properly

control_cluster_srvr = KyonRHClusterSrvr() # this blocks until connection with the client is established
controllers = generate_controllers()

for i in range(0, control_cluster_srvr.cluster_size):
    
    # we add the controllers

    result = control_cluster_srvr.add_controller(controllers[i])

control_cluster_srvr.start() # spawns the controllers on separate processes

try:

    while True:
        
        time.sleep(0.1) # we don't want to drain all the CPU
        # for a while True

        pass

except KeyboardInterrupt:

    # This block will execute when Control-C is pressed
    print(f"[{script_name}]" + "[info]: KeyboardInterrupt detected. Cleaning up...")

    control_cluster_srvr.terminate() # closes all processes

    import sys
    sys.exit()
