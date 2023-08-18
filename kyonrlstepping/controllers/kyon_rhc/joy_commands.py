from control_cluster_utils.utilities.rhc_defs import RhcTaskRefs
from control_cluster_utils.utilities.shared_mem import SharedMemClient
from control_cluster_utils.utilities.defs import cluster_size_name, n_contacts_name

import torch

from typing import List

class JoyCmds():

    def __init__(self,
            window_size: int,
            q_remapping: List[int] = None,
            dtype = torch.float32, 
            verbose=False):
        
        self.verbose = verbose

        self.wait_amount = 0.05

        self.cluster_size = SharedMemClient(n_rows=1, n_cols=1, 
                                    name=cluster_size_name(), 
                                    dtype=torch.int64, 
                                    wait_amount=self.wait_amount, 
                                    verbose=self.verbose)
        
        self.n_contacts = SharedMemClient(n_rows=1, n_cols=1, 
                                    name=n_contacts_name(), 
                                    dtype=torch.int64, 
                                    wait_amount=self.wait_amount, 
                                    verbose=True)
        
        self.cluster_size.attach()
        self.n_contacts.attach()
        
        cluster_size = self.cluster_size.tensor_view[0, 0].item()
        n_contacts = self.n_contacts.tensor_view[0, 0].item()

        rhc_refs = []

        for i in range(0, cluster_size):

                rhc_refs.append(RhcTaskRefs( 
                    cluster_size=cluster_size,
                    n_contacts=n_contacts,
                    index=i,
                    q_remapping=q_remapping,
                    dtype=dtype, 
                    verbose=verbose))        
    
    def update(self):

        a = 1