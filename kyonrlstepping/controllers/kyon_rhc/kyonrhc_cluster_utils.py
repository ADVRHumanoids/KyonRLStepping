from control_cluster_utils.utilities.control_cluster_utils import Action
import torch 

class KyonClusterCmd(Action):

    def __init__(self, 
                cluster_size: int = 1, 
                cmd_size: int = 1,
                device: str = "cpu"):
        
        self.cluster_size = cluster_size

        self.cmd_size = cmd_size

        self.device = device

        self.cmd = torch.zeros((self.cluster_size, self.cmd_size))

class JntImpCntrlAction(Action):

    def __init__(self, 
                n_jnts: int, 
                cluster_size: int = 1, 
                device: str = "cpu"):

        self._device = device

        self.q_ref = torch.zeros((cluster_size, n_jnts), device = self._device) # joint positions references
        self.q_dot_ref = torch.zeros((cluster_size, n_jnts), device = self._device) # joint velocities references
        self.q_ddot_ref = torch.zeros((cluster_size, n_jnts), device = self._device) # joint accelerations references
        self.effort_ref = torch.zeros((cluster_size, n_jnts), device = self._device) # joint effort references