from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc_refs import HybridQuadRhcRefs

import numpy as np

class KyonRHCRefs(HybridQuadRhcRefs):
    # only overrides parent tasks getter and setter methods 
    def _get_tasks(self):
        # overrides parent
        self.base_lin_vel = self.gait_manager.task_interface.getTask('base_lin_vel')
        self.base_omega = self.gait_manager.task_interface.getTask('base_omega')
        self.base_height = self.gait_manager.task_interface.getTask('base_height')

    def _apply_refs_to_tasks(self):
        # overrides parent
        root_pose = self.rob_refs.root_state.get(data_type = "q_full", 
                            robot_idxs=self.robot_index_np).reshape(-1, 1)
        root_twist_ref = self.rob_refs.root_state.get(data_type="twist", 
                            robot_idxs=self.robot_index_np).reshape(-1, 1)
        self.base_lin_vel.setRef(root_twist_ref[0:2, :])
        self.base_omega.setRef(root_twist_ref[3:, :])
        self.base_height.setRef(root_pose) # why for cart pos we need the whole 
        # while for cart vel just the active components of the task=????
