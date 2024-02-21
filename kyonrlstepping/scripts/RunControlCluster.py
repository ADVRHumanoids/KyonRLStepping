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
cluster_size = 8

core_ids_override_list = None
core_ids_override_list = list(range(8, 15 + 1))
control_cluster_client = KyonLRhcClusterClient(namespace=robot_name, 
                                    cluster_size=cluster_size,
                                    isolated_cores_only = False, 
                                    use_only_physical_cores = False,
                                    core_ids_override_list = core_ids_override_list,
                                    verbose=verbose) # this blocks until connection with the client is established

control_cluster_client.pre_init() # pre-initialization steps
    
control_cluster_client.run() # spawns the controllers on separate processes

control_cluster_client.terminate() # closes all processes

