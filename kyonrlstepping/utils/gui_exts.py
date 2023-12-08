
from PyQt5.QtWidgets import QWidget

from control_cluster_bridge.utilities.debugger_gui.gui_exts import SharedDataWindowV2
from control_cluster_bridge.utilities.debugger_gui.plot_utils import RtPlotWindow

from control_cluster_bridge.utilities.rhc_defs import RhcTaskRefs
from control_cluster_bridge.utilities.shared_mem import SharedMemClient

import torch

from control_cluster_bridge.utilities.defs import jnt_names_client_name
from control_cluster_bridge.utilities.defs import additional_data_name
from control_cluster_bridge.utilities.defs import n_contacts_name

from control_cluster_bridge.utilities.control_cluster_defs import JntImpCntrlData

from SharsorIPCpp.PySharsorIPC import VLevel

import numpy as np

class JntImpMonitor(SharedDataWindowV2):

    def __init__(self, 
            update_data_dt: int,
            update_plot_dt: int,
            window_duration: int,
            window_buffer_factor: int = 2,
            namespace = "",
            parent: QWidget = None, 
            verbose = False):

        self.n_jnts = -1
        self.n_envs = -1
        self.jnt_names = []

        super().__init__(update_data_dt = update_data_dt,
            update_plot_dt = update_plot_dt,
            window_duration = window_duration,
            window_buffer_factor = window_buffer_factor,
            grid_n_rows = 3,
            grid_n_cols = 3,
            namespace = namespace,
            name = "JntImpMonitor",
            parent = parent, 
            verbose = verbose)

    def _initialize(self):
        
        self.rt_plotters.append(RtPlotWindow(n_data=2 * self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Pos VS Pos Ref.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names + \
                                [item + "_ref" for item in self.jnt_names]))
        
        self.rt_plotters.append(RtPlotWindow(n_data=2 * self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Vel VS Vel Ref.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names + \
                                [item + "_ref" for item in self.jnt_names]))

        self.rt_plotters.append(RtPlotWindow(n_data=2 * self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Meas. eff VS Imp Eff.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names + \
                                [item + "_imp" for item in self.jnt_names]))

        self.rt_plotters.append(RtPlotWindow(n_data=self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Pos Gains", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names))

        self.rt_plotters.append(RtPlotWindow(n_data=self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Vel Gains", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names))

        self.rt_plotters.append(RtPlotWindow(n_data=self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Pos Err.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names))

        self.rt_plotters.append(RtPlotWindow(n_data=self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Vel Err.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names))

        self.rt_plotters.append(RtPlotWindow(n_data=self.n_jnts, 
                    update_data_dt=self.update_data_dt, 
                    update_plot_dt=self.update_plot_dt,
                    window_duration=self.window_duration, 
                    parent=None, 
                    base_name="Eff Feedfor.", 
                    window_buffer_factor=self.window_buffer_factor, 
                    legend_list=self.jnt_names))

        self.grid.addFrame(self.rt_plotters[0].base_frame, 0, 0)
        self.grid.addFrame(self.rt_plotters[1].base_frame, 0, 1)
        self.grid.addFrame(self.rt_plotters[2].base_frame, 0, 2)
        self.grid.addFrame(self.rt_plotters[3].base_frame, 1, 0)
        self.grid.addFrame(self.rt_plotters[4].base_frame, 1, 1)
        self.grid.addFrame(self.rt_plotters[5].base_frame, 2, 0)
        self.grid.addFrame(self.rt_plotters[6].base_frame, 2, 1)
        self.grid.addFrame(self.rt_plotters[7].base_frame, 2, 2)

    def _init_shared_data(self):

        self.shared_data_clients.append(JntImpCntrlData(is_server = False, 
                                            namespace = self.namespace, 
                                            verbose = True, 
                                            vlevel = VLevel.V2))
        
        self.shared_data_clients[0].run()

        self.n_jnts = self.shared_data_clients[0].n_jnts
        self.n_envs = self.shared_data_clients[0].n_envs
        self.jnt_names = self.shared_data_clients[0].jnt_names

    def _post_shared_init(self):

        pass

    def update(self):

        if not self._terminated:
            
            # pos VS pos ref
            self.shared_data_clients[0].pos_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].pos_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].pos_view.n_cols) 
                                                       
            self.shared_data_clients[0].pos_ref_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].pos_ref_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].pos_ref_view.n_cols)

            pos_vs_pos_ref = np.concatenate((self.shared_data_clients[0].pos_view.numpy_view[self.cluster_idx, :],
                                            self.shared_data_clients[0].pos_ref_view.numpy_view[self.cluster_idx, :]), 
                                            axis=0) 

            self.rt_plotters[0].rt_plot_widget.update(pos_vs_pos_ref)

            # vel VS vel ref
            self.shared_data_clients[0].vel_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].vel_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].vel_view.n_cols) # synch data
            self.shared_data_clients[0].vel_ref_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].vel_ref_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].vel_ref_view.n_cols) # synch data

            vel_vs_vel_ref = np.concatenate((self.shared_data_clients[0].vel_view.numpy_view[self.cluster_idx, :],
                                            self.shared_data_clients[0].vel_ref_view.numpy_view[self.cluster_idx, :]), 
                                            axis=0) 

            self.rt_plotters[1].rt_plot_widget.update(vel_vs_vel_ref)

            # meas. eff VS imp. effort
            self.shared_data_clients[0].eff_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].eff_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].eff_view.n_cols) # synch data
            self.shared_data_clients[0].imp_eff_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].imp_eff_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].imp_eff_view.n_cols) # synch data

            eff_vs_imp_eff = np.concatenate((self.shared_data_clients[0].eff_view.numpy_view[self.cluster_idx, :],
                                            self.shared_data_clients[0].imp_eff_view.numpy_view[self.cluster_idx, :]), 
                                            axis=0) 

            self.rt_plotters[2].rt_plot_widget.update(eff_vs_imp_eff)

            # pos gains
            self.shared_data_clients[0].pos_gains_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].pos_gains_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].pos_gains_view.n_cols) # synch data
            self.rt_plotters[3].rt_plot_widget.update(self.shared_data_clients[0].pos_gains_view.numpy_view[self.cluster_idx, :])

            # vel gains
            self.shared_data_clients[0].vel_gains_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].vel_gains_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].vel_gains_view.n_cols) # synch data
            self.rt_plotters[4].rt_plot_widget.update(self.shared_data_clients[0].vel_gains_view.numpy_view[self.cluster_idx, :])

            # pos error
            self.shared_data_clients[0].pos_err_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].pos_err_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].pos_err_view.n_cols) # synch data
            self.rt_plotters[5].rt_plot_widget.update(self.shared_data_clients[0].pos_err_view.numpy_view[self.cluster_idx, :])

            # vel error
            self.shared_data_clients[0].vel_err_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].vel_err_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].vel_err_view.n_cols) # synch data
            self.rt_plotters[6].rt_plot_widget.update(self.shared_data_clients[0].vel_err_view.numpy_view[self.cluster_idx, :])

            # eff. feedforward  
            self.shared_data_clients[0].imp_eff_view.synch(read=True, row_index = 0, col_index = 0, 
                                            n_rows = self.shared_data_clients[0].imp_eff_view.n_rows, 
                                            n_cols = self.shared_data_clients[0].imp_eff_view.n_cols) # synch data
            self.rt_plotters[7].rt_plot_widget.update(self.shared_data_clients[0].imp_eff_view.numpy_view[self.cluster_idx, :])
