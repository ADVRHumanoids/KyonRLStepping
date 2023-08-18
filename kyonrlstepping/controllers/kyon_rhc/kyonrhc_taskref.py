from kyonrlstepping.controllers.kyon_rhc.gait_manager import GaitManager

from control_cluster_utils.utilities.rhc_defs import RhcTaskRefs

import torch

from typing import List, Dict

import numpy as np

class KyonRhcTaskRef(RhcTaskRefs):

    def __init__(self, 
            gait_manager: GaitManager, 
            cluster_size: int,
            n_contacts: int,
            index: int,
            q_remapping: List[int] = None,
            dtype = torch.float32, 
            verbose=False):
        
        super().__init__( 
                cluster_size=cluster_size,
                n_contacts=n_contacts,
                index=index,
                q_remapping=q_remapping,
                dtype=dtype, 
                verbose=verbose)

        # handles phase transitions
        self.gait_manager = gait_manager

        # task references
        self.final_base_xy = self.gait_manager.task_interface.getTask('final_base_xy')
        self.com_height = self.gait_manager.task_interface.getTask('com_height')
        self.base_orientation = self.gait_manager.task_interface.getTask('base_orientation')

    def update(self):
        
        # contact phases
        if self.phase_id.get_phase_id() < 0 and \
            self.gait_manager.contact_phases['ball_1'].getEmptyNodes() > 0:

            is_contact = (self.phase_id.get_contacts().numpy() > 0.5).flatten().tolist() 
            # contact if contact_flags[i] > 0.5
            
            self.gait_manager.cycle(is_contact)
                
        else:

            if self.phase_id.get_phase_id() == 0:

                self.gait_manager.stand()
            
            if self.phase_id.get_phase_id() == 1:

                self.gait_manager.walk()

            if self.phase_id.get_phase_id() == 2:

                self.gait_manager.crawl()

            if self.phase_id.get_phase_id() == 3:

                self.gait_manager.trot()

            if self.phase_id.get_phase_id() == 4:

                self.gait_manager.trot_jumped()

            if self.phase_id.get_phase_id() == 5:

                self.gait_manager.jump()

            if self.phase_id.get_phase_id() == 6:

                self.gait_manager.wheelie()

        # updated internal references with latest available ones
        self.final_base_xy.setRef(self.base_pose.get_pose().numpy().T)
        self.base_orientation.setRef(self.base_pose.get_pose().numpy().T)