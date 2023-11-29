from SharsorIPCpp.PySharsorIPC import ServerFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType, dtype
from SharsorIPCpp.PySharsorIPC import Journal , LogType

import numpy as np

class RHC2SharedNamings:

    def __init__(self, 
            basename: str, 
            namespace: str, 
            index: int):

        self.index = index

        self.basename = basename
        self.namespace = namespace

        self.global_ns = f"{basename}_{namespace}_{self.index}"

        self.ROBOT_Q_NAME = "robot_q"
        self.RHC_Q_NAME = "rhc_q"

    def global_ns(self, 
            basename: str, 
            namespace: str):

        return f"{basename}_{namespace}"
    
    def get_robot_q_name(self):
        
        return f"{self.global_ns}_{self.ROBOT_Q_NAME}"
    
    def get_rhc_q_name(self):

        return f"{self.global_ns}_{self.RHC_Q_NAME}"
    
class RHC2SharedInternal:

    # bridge from RHC controller to shared memory

    def __init__(self, 
            namespace: str, 
            index: int, 
            n_jnts: int, 
            n_rhc_nodes: int,
            verbose = False,
            basename: str = "RHC2SharedInternal"):
    
        self.verbose = verbose

        self.index = index

        self.basename = basename
        self.namespace = namespace

        self.names = RHC2SharedNamings(basename = self.basename, 
                        namespace = self.namespace, 
                        index = self.index)
        
        self.n_jnts = n_jnts

        self.floating_base_q_dim = 7 # orientation quat.
        self.n_rows = 7 + self.n_jnts
        self.n_cols = n_rhc_nodes

        self.dtype = np.float32
        self.layout = RowMajor
        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'
        
        self.server_factory_rhc_q = None
        self.server_factory_robot_q = None

        self._init_rhc_q_bridge()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.server_factory_rhc_q.getScalarType()),
                                order=self.order)

        self.rhc_q[6,:] = 1 # initialize to valid quaternion

        self.robot_q = np.zeros((self.n_rows, 1),
                                dtype=toNumpyDType(self.server_factory_robot_q.getScalarType()),
                                order='C' # to circumvent Numpy bug when slicing 2D matrices as 1D forcing Fortran ordering
                                )

        self.robot_q[6,:] = 1 # initialize to valid quaternion

    def run(self):
                        
        self.server_factory_rhc_q.run() # starts servers

        self.server_factory_rhc_q.write(self.rhc_q[:, :], 0, 0) # initialized to 
        #  null valid data (identity quaternion)
        #             
        self.server_factory_robot_q.run() # starts servers

        self.server_factory_robot_q.write(self.robot_q[:, :], 0, 0) # initialized to 
        #  null valid data (identity quaternion)

    def update(self, 
            q_opt: np.ndarray, 
            q_robot: np.ndarray):

        # update rhc_q matrix from Horizon solution dictionary
        self.rhc_q[:, :] = q_opt[0:self.rhc_q.shape[0], 
                                0:self.rhc_q.shape[1]]
        
        if q_robot.shape[0] != self.robot_q.shape[0]:
            
            message = f"provided q_robot n_rows {q_robot.shape[0]} does not match provided q_opt n_rows {self.robot_q.shape[0]}"

            Journal.log(self.__class__.__name__,
                    "update",
                    message,
                    LogType.EXCEP, 
                    throw_when_excep = True)
            
            return

        if q_robot.shape[1] != self.robot_q.shape[1]:
            
            message = f"provided q_robot n_cols {q_robot.shape[1]} does not match provided q_opt n_cols {self.robot_q.shape[1]}"

            Journal.log(self.__class__.__name__,
                    "update",
                    message,
                    LogType.EXCEP, 
                    throw_when_excep = True)

            return
        
        self.robot_q[:, :] = q_robot[0:self.robot_q.shape[0], 
                                    0:self.robot_q.shape[1]]

        # writing rhc q to shared memory
        self.server_factory_rhc_q.write(self.rhc_q[:, :], 0, 0)

        # writing robot q to shared memory
        
        self.server_factory_robot_q.write(self.robot_q[:, :], 0, 0)
    
    def close(self):

        self.server_factory_rhc_q.close() 

        self.server_factory_robot_q.close()

    def _init_rhc_q_bridge(self):

        # rhc internal state
        self.server_factory_rhc_q = ServerFactory(n_rows = self.n_rows, 
                                        n_cols = self.n_cols,
                                        basename = "",
                                        namespace = self.names.get_rhc_q_name(), 
                                        verbose = self.verbose, 
                                        vlevel = VLevel.V3, 
                                        force_reconnection = False, 
                                        dtype = dtype.Float,
                                        layout = self.layout)
        
        self.server_factory_robot_q = ServerFactory(n_rows = self.n_rows, 
                                        n_cols = 1,
                                        basename = "",
                                        namespace = self.names.get_robot_q_name(), 
                                        verbose = self.verbose, 
                                        vlevel = VLevel.V3, 
                                        force_reconnection = False, 
                                        dtype = dtype.Float,
                                        layout = ColMajor # ColMajor to circumvent Numpy bug when slicing 2D matrices as 1D
                                        )