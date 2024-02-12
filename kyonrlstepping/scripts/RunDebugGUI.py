from control_cluster_bridge.utilities.debugger_gui.cluster_debugger import RtClusterDebugger
from control_cluster_bridge.utilities.debugger_gui.gui_exts import JntImpMonitor

import os
import argparse

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

    data_update_dt = 0.01
    plot_update_dt = 0.1

    window_length = 10.0
    window_buffer_factor = 2

    namespace = "kyon0"
    cluster_debugger = RtClusterDebugger(data_update_dt=data_update_dt,
                            plot_update_dt=plot_update_dt,
                            window_length=window_length, 
                            window_buffer_factor=window_buffer_factor, 
                            verbose=True, 
                            namespace=namespace)
    
    # adding some of the available extensions
    gui_extensions = []
    gui_extensions.append(JntImpMonitor(update_data_dt = data_update_dt,
            update_plot_dt = plot_update_dt,
            window_duration = window_length,
            window_buffer_factor = window_buffer_factor,
            namespace = namespace,
            verbose = True))

    cluster_debugger.add_spawnable_tab(gui_extensions)

    cluster_debugger.run()
