from control_cluster_utils.utilities.cluster_debugger import RtClusterDebugger

if __name__ == "__main__":  

    update_dt = 0.05
    window_length = 4.0
    window_buffer_factor = 1
    main_window = RtClusterDebugger(update_dt=update_dt,
                            window_length=window_length, 
                            window_buffer_factor=window_buffer_factor, 
                            verbose=True)

    main_window.run()
