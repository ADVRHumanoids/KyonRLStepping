from control_cluster_bridge.utilities.debugger_gui.cluster_debugger import RtClusterDebugger
from kyonrlstepping.utils.gui_exts import JntImpMonitor

if __name__ == "__main__":  

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
                            namespace=namespace,
                            add_sim_data = True, 
                            add_cluster_data = True)
    
    gui_extensions = []
    gui_extensions.append(JntImpMonitor(update_data_dt = data_update_dt,
            update_plot_dt = plot_update_dt,
            window_duration = window_length,
            window_buffer_factor = window_buffer_factor,
            namespace = namespace,
            verbose = True))

    cluster_debugger.add_spawnable_tab(gui_extensions)

    cluster_debugger.run()
