import os
script_name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0]

from kyonrlstepping.controllers.kyon_rhc.kyonrhc_cluster_client import KyonLRhcClusterClient
from kyonrlstepping.controllers.kyon_rhc.utils.sysutils import PathsGetter
kyonrhc_paths = PathsGetter

import torch

from perf_sleep.pyperfsleep import PerfSleep

import os, argparse

# Function to set CPU affinity
def set_affinity(cores):
    try:
        os.sched_setaffinity(0, cores)
        print(f"Set CPU affinity to cores: {cores}")
    except Exception as e:
        print(f"Error setting CPU affinity: {e}")

if __name__ == "__main__":  

    # Parse command line arguments for CPU affinity
    parser = argparse.ArgumentParser(description="Set CPU affinity for the script.")
    parser.add_argument('--cores', nargs='+', type=int, help='List of CPU cores to set affinity to')
    
    args = parser.parse_args()

    # Set CPU affinity if cores are provided
    if args.cores:
        set_affinity(args.cores)

    verbose = False

    debug = True
    solver_deb_prints = False
    profile_all = True
    debug_solution = True

    max_solver_iter = 1

    perf_timer = PerfSleep()

    robot_name = "kyon0"
    cluster_size = 10

    perf_timer = PerfSleep()

    core_ids_override_list = None
    # core_ids_override_list = list(range(4, 15 + 1))
    control_cluster_client = KyonLRhcClusterClient(namespace=robot_name, 
                                        cluster_size=cluster_size,
                                        isolated_cores_only = True, 
                                        use_only_physical_cores = False,
                                        core_ids_override_list = core_ids_override_list,
                                        verbose=verbose) # this blocks until connection with the client is established

    control_cluster_client.pre_init() # pre-initialization steps
        
    control_cluster_client.run() # spawns the controllers on separate processes

    try:

        while True:
            
            nsecs = int(0.1 * 1e9)
            perf_timer.thread_sleep(nsecs) # we don't want to drain all the CPU
            # with a busy wait

            pass

    except KeyboardInterrupt:

        # This block will execute when Control-C is pressed
        print(f"[{script_name}]" + "[info]: KeyboardInterrupt detected. Cleaning up...")

        control_cluster_client.terminate() # closes all processes
        
        import sys
        sys.exit()


