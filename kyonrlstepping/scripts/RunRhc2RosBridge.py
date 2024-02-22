from lrhc_control.utils.rhc2viz import RhcToVizBridge

from perf_sleep.pyperfsleep import PerfSleep

from SharsorIPCpp.PySharsorIPC import LogType
from SharsorIPCpp.PySharsorIPC import Journal

import argparse
import os

import rospy

import time 

# Function to set CPU affinity
def set_affinity(cores):
    try:
        os.sched_setaffinity(0, cores)
        print(f"Set CPU affinity to cores: {cores}")
    except Exception as e:
        print(f"Error setting CPU affinity: {e}")

if __name__ == '__main__':

    # Parse command line arguments for CPU affinity
    parser = argparse.ArgumentParser(description="Set CPU affinity for the script.")
    parser.add_argument('--cores', nargs='+', type=int, help='List of CPU cores to set affinity to')
    parser.add_argument('--dt', type=float, default=0.01, help='Update interval in seconds, default is 0.01')
    parser.add_argument('robot_name', type=str, help='Name of the robot')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode, default is False')
    parser.add_argument('--verbose', type=bool, default=True, help='Enable verbose mode, default is True')
    args = parser.parse_args()

    # Set CPU affinity if cores are provided
    if args.cores:
        set_affinity(args.cores)

    # Use the provided robot name and update interval
    robot_name = args.robot_name
    update_dt = args.dt
    debug = args.debug
    verbose = args.verbose

    bridge = RhcToVizBridge(namespace=robot_name, 
                        verbose=verbose,
                        rhcviz_basename="RHCViz", 
                        robot_selector=[0, None])
    
    bridge.run()

    perf_timer = PerfSleep()
    
    info = f": starting bridge with update dt {update_dt} s"
    Journal.log("RHC2RosBridge",
        "",
        info,
        LogType.INFO,
        throw_when_excep = True)

    start_time = 0.0
    elapsed_time = 0.0
    actual_loop_dt = 0.0

    time_to_sleep_ns = 0

    while not rospy.is_shutdown():

        start_time = time.perf_counter() 

        success = bridge.update() # index is the controller 
        # position in the cluster

        elapsed_time = time.perf_counter() - start_time

        time_to_sleep_ns = int((update_dt - elapsed_time) * 1e+9) # [ns]

        if time_to_sleep_ns < 0:

            warning = f": Could not match desired update dt of {update_dt} s. " + \
                f"Elapsed time to update {elapsed_time}."
            Journal.log("RHC2RosBridge",
                "",
                warning,
                LogType.WARN,
                throw_when_excep = True)

        perf_timer.thread_sleep(time_to_sleep_ns) # nanoseconds (actually resolution is much
                    # poorer)

        actual_loop_dt = time.perf_counter() - start_time

        if debug:

            print(f"Actual loop dt {actual_loop_dt} s.")
