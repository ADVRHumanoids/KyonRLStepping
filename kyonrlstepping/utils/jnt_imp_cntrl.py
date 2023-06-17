import torch 

from typing import List
from enum import Enum

class JntImpCntrl:

    class IndxState(Enum):

        NONE = -1 
        VALID = 1
        INVALID = 0

    def __init__(self, 
                num_robots: int,
                dofs_names: List[str],
                default_pgain = 300.0, 
                default_vgain = 30.0, 
                backend = "torch", 
                device = "cpu"):
        
        self._valid_signal_types = ["pos_ref", "vel_ref", "eff_ref", 
                                    "pos", "vel", 
                                    "pgain", "vgain"] 
        self._device = device

        self.num_robots = num_robots

        self.n_dofs = len(dofs_names)
        
        self.dofs_names = dofs_names
        self.jnt_idxs = torch.tensor([i for i in range(0, self.n_dofs)], 
                                    device = self._device, 
                                    dtype=torch.int64)

        if (backend != "torch"):

            print("JntImpCntrl: forcing torch backend. Other backends are not yet supported.")
        
        self._backend = "torch"

        self.cntrl_action = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)
        
        # we assume diagonal gain matrices, so we can save on memory and only store the diagonal
        self._default_pgain = default_pgain
        self._default_vgain = default_vgain
        self._pos_gains =  torch.full((self.num_robots, self.n_dofs), 
                                    self._default_pgain, 
                                    device = self._device, 
                                    dtype=torch.float32)
        self._vel_gains = torch.full((self.num_robots, self.n_dofs), 
                                    self._default_vgain,
                                    device = self._device, 
                                    dtype=torch.float32)
        
        self._eff_ref = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)
        self._pos_ref = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)
        self._vel_ref = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)
        
        self._pos = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)
        self._vel = torch.zeros((self.num_robots, self.n_dofs), device = self._device, 
                                    dtype=torch.float32)

    def _validate_selectors(self, 
                robot_indxs: torch.Tensor = None, 
                jnt_indxs: torch.Tensor = None):

        check = [None] * 2 

        if robot_indxs is not None:

            robot_indxs_shape = robot_indxs.shape

            if (not (len(robot_indxs_shape) == 1 and \
                robot_indxs.dtype == torch.int64 and \
                robot_indxs.device.type == self._device and \
                bool(torch.min(robot_indxs) >= 0) and \
                bool(torch.max(robot_indxs) < self.n_dofs))): # sanity checks 

                check[0] = JntImpCntrl.IndxState.INVALID
            
            else:

                check[0] = JntImpCntrl.IndxState.VALID

        else:

            check[0] = JntImpCntrl.IndxState.NONE

        if jnt_indxs is not None:

            jnt_indxs_shape = jnt_indxs.shape
            if (not (len(jnt_indxs_shape) == 1 and \
                jnt_indxs.dtype == torch.int64 and \
                jnt_indxs.device.type == self._device and \
                bool(torch.min(jnt_indxs) >= 0) and \
                bool(torch.max(jnt_indxs) < self.n_dofs))): # sanity checks 

                check[1] = JntImpCntrl.IndxState.INVALID
            
            else:

                check[1] = JntImpCntrl.IndxState.VALID

        else:

            check[1] = JntImpCntrl.IndxState.NONE

        return check
    
    def _gen_selector(self, 
                robot_indxs: torch.Tensor, 
                jnt_indxs: torch.Tensor):
        
        selector = None

        indxs_check = self._validate_selectors(robot_indxs=robot_indxs, 
                            jnt_indxs=jnt_indxs)         

        if (indxs_check[0] == JntImpCntrl.IndxState.VALID and \
            indxs_check[1] == JntImpCntrl.IndxState.VALID):

            selector = torch.meshgrid((robot_indxs, jnt_indxs), 
                                    indexing="ij")
        
        if(indxs_check[0] == JntImpCntrl.IndxState.VALID and \
           indxs_check[1] == JntImpCntrl.IndxState.NONE):
            
            selector = torch.meshgrid((robot_indxs, 
                                       torch.tensor([i for i in range(0, self.n_dofs)], 
                                                    device = self._device, 
                                                    dtype=torch.int64)), 
                                    indexing="ij")
        
        if(indxs_check[0] == JntImpCntrl.IndxState.NONE and \
           indxs_check[1] == JntImpCntrl.IndxState.VALID):
            
            selector = torch.meshgrid((torch.tensor([i for i in range(0, self.num_robots)], 
                                                    device = self._device, 
                                                    dtype=torch.int64)), 
                                        jnt_indxs, 
                                    indexing="ij")
        
        return selector
            
    def _validate_signal(self, 
                        signal: torch.Tensor, 
                        selector: torch.Tensor = None):
        
        signal_shape = signal.shape
        if selector is None:

            if signal_shape[0] == self.num_robots and \
                signal_shape[1] == self.n_dofs and \
                signal.device.type == self._device:
                

                return True
            
            else:

                return False
            
        else:
            
            selector_shape = selector[0].shape

            if signal_shape[0] == selector_shape[0] and \
                signal_shape[1] == selector_shape[1] and \
                signal.device.type == self._device:

                return True
            
            else:

                return False
        
    def _assign_signal(self, 
                    signal_type: str,
                    signal: torch.Tensor = None, 
                    selector: torch.Tensor = None):

        if signal_type in self._valid_signal_types: 

            if signal_type == self._valid_signal_types[0]: # "pos_ref"
                
                if selector is not None:

                    self._pos_ref[selector] = signal

                else:
                    
                    self._pos_ref = signal

                return True

            if signal_type == self._valid_signal_types[1]: # "vel_ref"
                
                if selector is not None:

                    self._vel_ref[selector] = signal

                else:
                    
                    self._vel_ref = signal

                return True

            if signal_type == self._valid_signal_types[2]: # "eff_ref"

                if selector is not None:

                    self._eff_ref[selector] = signal

                else:
                    
                    self._eff_ref = signal
                
                return True

            if signal_type == self._valid_signal_types[3]: # "pos"
                
                if selector is not None:

                    self._pos[selector] = signal

                else:
                    
                    self._pos = signal
                
                return True

            if signal_type == self._valid_signal_types[4]: # "vel"
                
                if selector is not None:

                    self._vel[selector] = signal

                else:
                    
                    self._vel = signal

                return True

            if signal_type == self._valid_signal_types[5]: # "pgain"
                
                if selector is not None:
                    
                    self._pos_gains[selector] = signal

                else:
                    
                    self._pos_gains = signal

                return True

            if signal_type == self._valid_signal_types[6]: # "vgain"
                
                if selector is not None:

                    self._vel_gains[selector] = signal

                else:
                    
                    self._vel_gains = signal

                return True

        else:

            return False
        
    def set_gains(self, 
                pos_gains: torch.Tensor = None, 
                vel_gains: torch.Tensor = None, 
                robot_indxs: torch.Tensor = None, 
                jnt_indxs: torch.Tensor = None):

        success = [True] * 4 # error codes:
        # success[0] == False -> pos_gains error
        # success[1] == False -> vel_gains error
        # success[2] == False -> indexes error
        # success[3] == False -> assign error
        
        selector = self._gen_selector(robot_indxs=robot_indxs, 
                           jnt_indxs=jnt_indxs)
        
        if pos_gains is not None:

            pos_gains_valid = self._validate_signal(signal = pos_gains, 
                                                    selector = selector) 
            
            if (pos_gains_valid):
                
                if(not self._assign_signal(signal_type = "pgain", 
                                    signal = pos_gains, 
                                    selector=selector)):
                    
                    success[3] = False
            
            else:

                success[0] = False 

        if vel_gains is not None:

            vel_gains_valid = self._validate_signal(signal = pos_gains, 
                                                    selector = selector) 
            
            if (vel_gains_valid):
                
                if (not self._assign_signal(signal_type = "vgain", 
                                    signal = vel_gains, 
                                    selector=selector)):
                    
                    success[3] = False
            
            else:

                success[1] = False 

        return success
            
    def set_refs(self, 
                eff_ref: torch.Tensor = None, 
                pos_ref: torch.Tensor = None, 
                vel_ref: torch.Tensor = None, 
                robot_indxs: torch.Tensor = None, 
                jnt_indxs: torch.Tensor = None):
        
        success = [True] * 5 # error codes:
        # success[0] == False -> eff_ref error
        # success[1] == False -> pos_ref error
        # success[2] == False -> vel_ref error
        # success[3] == False -> indexes error
        # success[4] == False -> assign error

        selector = self._gen_selector(robot_indxs=robot_indxs, 
                           jnt_indxs=jnt_indxs)
        
        if eff_ref is not None:

            valid = self._validate_signal(signal = eff_ref, 
                                        selector = selector) 
            
            if (valid):
                
                if(not self._assign_signal(signal_type = "eff_ref", 
                                    signal = eff_ref, 
                                    selector = selector)):
                    
                    success[4] = False
            
            else:

                success[0] = False 

        if pos_ref is not None:

            valid = self._validate_signal(signal = pos_ref, 
                                        selector = selector) 
            
            if (valid):
                
                if(not self._assign_signal(signal_type = "pos_ref", 
                                    signal = pos_ref, 
                                    selector = selector)):
                    
                    success[4] = False
            
            else:

                success[1] = False 
            
        if vel_ref is not None:

            valid = self._validate_signal(signal = vel_ref, 
                                        selector = selector) 
            
            if (valid):
                
                if(not self._assign_signal(signal_type = "vel_ref", 
                                    signal = vel_ref, 
                                    selector = selector)):
                    
                    success[4] = False
            
            else:

                success[2] = False

        return success
        
    def set_state(self, 
            pos: torch.Tensor = None, 
            vel: torch.Tensor = None, 
            robot_indxs: torch.Tensor = None, 
            jnt_indxs: torch.Tensor = None):

        success = [True] * 3 # error codes:
        # success[0] == False -> pos error
        # success[1] == False -> vel error
        # success[2] == False -> indexes error
        # success[3] == False -> assign error

        selector = self._gen_selector(robot_indxs=robot_indxs, 
                           jnt_indxs=jnt_indxs)
        
        if pos is not None:

            valid = self._validate_signal(signal = pos, 
                                        selector = selector) 
            
            if (valid):
                
                if(not self._assign_signal(signal_type = "pos", 
                                    signal = pos, 
                                    selector = selector)):
                    
                    success[3] = False
            
            else:

                success[0] = False 

        if vel is not None:

            valid = self._validate_signal(signal = vel, 
                                        selector = selector) 
            
            if (valid):
                
                if(not self._assign_signal(signal_type = "vel", 
                                    signal = vel, 
                                    selector = selector)):
                    
                    success[3] = False
            
            else:

                success[1] = False

        return success
    
    def update(self, 
            robot_indxs: torch.Tensor = None, 
            jnt_indxs: torch.Tensor = None):

        success = True

        selector = self._gen_selector(robot_indxs=robot_indxs, 
                           jnt_indxs=jnt_indxs)
        
        if selector is not None:

            self.cntrl_action[selector] = torch.add(self._eff_ref[selector], 
                                                        torch.add(
                                                            torch.mul(self._pos_gains[selector], 
                                                                torch.sub(self._pos[selector], self._pos_ref[selector])),  \
                                                            torch.mul(self._vel_gains[selector], torch.sub(self._vel[selector], self._vel_ref[selector]))))
                                                        
        
        else:
            
            if robot_indxs is None and jnt_indxs is None:

                self.cntrl_action = torch.add(self._eff_ref, 
                                                torch.add(
                                                    torch.mul(self._pos_gains, 
                                                        torch.sub(self._pos, self._pos_ref)),  \
                                                    torch.mul(self._vel_gains, torch.sub(self._vel, self._vel_ref))))
            else:

                success = False
            
        return success 
    
    def get(self, 
            robot_indxs: torch.Tensor = None, 
            jnt_indxs: torch.Tensor = None):
        
        # returns a view of the internal control action

        selector = self._gen_selector(robot_indxs=robot_indxs, 
                           jnt_indxs=jnt_indxs)

        if selector is not None:
            
            return self.cntrl_action[selector]
                                                        
        else:
            
            return self.cntrl_action

    def get_pos_gains(self):

        return self._pos_gains
    
    def get_vel_gains(self):

        return self._vel_gains