import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonLRhcClusterClient
from kyonrlstepping.controllers.kyon_rhc.utils.sysutils import PathsGetter
kyonrhc_paths = PathsGetter

import torch

from perf_sleep.pyperfsleep import PerfSleep

verbose = False

debug = True
solver_deb_prints = False
profile_all = True
debug_solution = True

max_solver_iter = 1

perf_timer = PerfSleep()

robot_name = "kyon0"
cluster_size = 32

perf_timer = PerfSleep()

core_ids_override_list = None
core_ids_override_list = list(range(4, 35 + 1))
control_cluster_client = KyonLRhcClusterClient(namespace=robot_name, 
                                    cluster_size=cluster_size,
                                    isolated_cores_only = False, 
                                    use_only_physical_cores = False,
                                    core_ids_override_list = core_ids_override_list,
                                    verbose=verbose) # this blocks until connection with the client is established

control_cluster_client.pre_init() # pre-initialization steps
    
control_cluster_client.run() # spawns the controllers on separate processes

try:

    while True:
        
        nsecs = int(0.1 * 1e9)
        perf_timer.clock_sleep(nsecs) # we don't want to drain all the CPU
        # with a busy wait

        pass

except KeyboardInterrupt:

    # This block will execute when Control-C is pressed
    print(f"[{script_name}]" + "[info]: KeyboardInterrupt detected. Cleaning up...")

    control_cluster_client.terminate() # closes all processes
    
    import sys
    sys.exit()


