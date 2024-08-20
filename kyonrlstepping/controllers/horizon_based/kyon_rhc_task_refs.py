from lrhc_control.controllers.rhc.horizon_based.hybrid_quad_rhc_refs import HybridQuadRhcRefs
from lrhc_control.controllers.rhc.horizon_based.utils.math_utils import hor2w_frame

import numpy as np

class KyonRHCRefs(HybridQuadRhcRefs):
    # only overrides parent tasks getter and setter methods 
    def _apply_refs_to_tasks(self, q_base = None):
        # overrides parent
        if q_base is not None: # rhc refs are assumed to be specified in the so called "horizontal" 
            # frame, i.e. a vertical frame, with the x axis aligned with the projection of the base x axis
            # onto the plane
            root_pose = self.rob_refs.root_state.get(data_type = "q_full", 
                                robot_idxs=self.robot_index_np).reshape(-1, 1) # this should also be
            # rotated into the horizontal frame (however, for now only the z componet is used, so it's ok)
            
            root_twist_ref = self.rob_refs.root_state.get(data_type="twist", 
                                robot_idxs=self.robot_index_np).reshape(-1, 1)
            root_twist_ref_h = root_twist_ref.copy() 

            hor2w_frame(root_twist_ref, q_base, root_twist_ref_h)

            self.base_lin_vel.setRef(root_twist_ref_h[0:3, :])
            self.base_omega.setRef(root_twist_ref_h[3:, :])
            # self.base_height.setRef(root_pose) # why for cart pos we need the whole 
            # while for cart vel just the active components of the task=????
        else:
            root_pose = self.rob_refs.root_state.get(data_type = "q_full", 
                                robot_idxs=self.robot_index_np).reshape(-1, 1)
            root_twist_ref = self.rob_refs.root_state.get(data_type="twist", 
                                robot_idxs=self.robot_index_np).reshape(-1, 1)

            self.base_lin_vel.setRef(root_twist_ref[0:3, :])
            self.base_omega.setRef(root_twist_ref[3:, :])
            # self.base_height.setRef(root_pose) # why for cart pos we need the whole 
            # while for cart vel just the active components of the task=????
