import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

from kyonrlstepping.controllers.kyon_rhc.kyon_rhc import KyonRhc
from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonLRhcClusterClient
from kyonrlstepping.controllers.kyon_rhc.utils.sysutils import PathsGetter
kyonrhc_paths = PathsGetter

import torch

from perf_sleep.pyperfsleep import PerfSleep

def generate_controllers(robot_name: str):

    kyonrhc_config_path = kyonrhc_paths().CONFIGPATH

    # create controllers
    cluster_controllers = []
    for i in range(0, control_cluster_srvr.cluster_size):

        cluster_controllers.append(KyonRhc(
                urdf_path=control_cluster_srvr._urdf_path, 
                srdf_path=control_cluster_srvr._srdf_path,
                cluster_size=control_cluster_srvr.cluster_size,
                robot_name=robot_name,
                config_path = kyonrhc_config_path,
                dt=0.03,
                n_nodes=31, 
                max_solver_iter = max_solver_iter,
                verbose = verbose, 
                debug = debug,
                solver_deb_prints = solver_deb_prints,
                profile_all = profile_all,
                array_dtype = dtype,
                publish_sol=debug_solution))

    return cluster_controllers

verbose = True

debug = True
solver_deb_prints = False
profile_all = True
debug_solution = True

max_solver_iter = 1

perf_timer = PerfSleep()

dtype = torch.float32 # this has to be the same wrt the cluster client, otherwise
# messages are not read properly

robot_name = "kyon0"
cluster_size = 5

core_ids_override_list = None
# core_ids_override_list = [11, 12, 13, 14, 15]
control_cluster_srvr = KyonLRhcClusterClient(namespace=robot_name, 
                                    cluster_size=cluster_size,
                                    isolated_cores_only = False, 
                                    use_only_physical_cores = False,
                                    core_ids_override_list = core_ids_override_list,
                                    verbose=verbose) # this blocks until connection with the client is established

control_cluster_srvr.pre_init() # pre-initialization steps

controllers = generate_controllers(robot_name)
    
for i in range(0, control_cluster_srvr.cluster_size):
    
    # we add the controllers

    result = control_cluster_srvr.add_controller(controllers[i])

control_cluster_srvr.run() # spawns the controllers on separate processes

try:

    while True:
        
        nsecs = int(0.1 * 1e9)
        perf_timer.clock_sleep(nsecs) # we don't want to drain all the CPU
        # with a busy wait

        pass

except KeyboardInterrupt:

    # This block will execute when Control-C is pressed
    print(f"[{script_name}]" + "[info]: KeyboardInterrupt detected. Cleaning up...")

    control_cluster_srvr.terminate() # closes all processes

    import sys
    sys.exit()
