import signal
# Define a flag variable to control the loop
running = True

# Define a signal handler for the keyboard interrupt
def signal_handler(signal, frame):
    global running
    running = False

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

from kyon_rhc.kyonrhc import KyonRHC
from kyon_rhc.kyonrhc_cluster_srvr import KyonRHClusterSrvr
cmd_size = 2
verbose = True

control_cluster_srvr = KyonRHClusterSrvr(cmd_size = cmd_size)

for i in range(0, control_cluster_srvr.cluster_size):
    # we initialize the controllers and assign them the pipes for communication
    result = control_cluster_srvr.add_controller(KyonRHC(config_path = "", 
                                        trigger_pipename = control_cluster_srvr.trigger_pipes[i],
                                        success_pipename = control_cluster_srvr.success_pipes[i], 
                                        verbose = verbose, 
                                        name = "KyonRHController" + str(i)))

control_cluster_srvr.start() 

control_cluster_srvr.terminate() # closes all processes
