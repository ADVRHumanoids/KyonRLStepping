import signal
# Define a flag variable to control the loop
running = True

# Define a signal handler for the keyboard interrupt
def signal_handler(signal, frame):
    global running
    running = False

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

from kyon_rhc.kyonrhc import KyonClusterCmd, KyonRHClusterSrvr, KyonRHC
from kyon_rhc.interface.control_cluster import RobotClusterState
num_envs = 150
cmd_size = 2
device = "cuda"
verbose = False
cluster_cmds = KyonClusterCmd(cluster_size = num_envs, 
                            cmd_size = cmd_size, 
                            device = device)

control_cluster_srvr = KyonRHClusterSrvr(cluster_size = num_envs, 
                            cmd_size = cmd_size, 
                            device = device)


for i in range(0, num_envs):
    # we initialize the controllers and assign them the pipes for communication
    result = control_cluster_srvr.add_controller(KyonRHC(config_path = "", 
                                        trigger_pipe = control_cluster_srvr.trigger_pipes[i][0],
                                        success_pipe = control_cluster_srvr.success_pipes[i][1], 
                                        verbose = verbose, 
                                        name = "KyonRHController" + str(i)))

control_cluster_srvr.setup()

cluster_state = RobotClusterState(n_dofs = control_cluster_srvr.n_dofs, 
                                cluster_size = num_envs, 
                                device = device)

control_cluster_srvr.update(cluster_state)

while running:

    # rhc_cmds = rhc_get_cmds_fromjoy() or from agent

    control_cluster_srvr.set_commands(cluster_cmds)

    control_cluster_srvr.solve()

    print("Cluster solution time: " + str(control_cluster_srvr.solution_time))

    if not running:

        break

control_cluster_srvr.terminate() # closes all processes
