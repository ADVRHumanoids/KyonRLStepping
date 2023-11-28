from SharsorIPCpp.PySharsorIPC import ServerFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType, dtype

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
        
        self.server_factories = []

        self._init_rhc_q_bridge()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.server_factories[0].getScalarType()),
                                order=self.order)

        self.rhc_q[6,:] = 1 # initialize to valid quaternion

    def run(self):

        for i in range(len(self.server_factories)):
                        
            self.server_factories[i].run() # starts servers

            self.server_factories[i].write(self.rhc_q[:, :], 0, 0) # initialized to 
            #  null valid data (identity quaternion)

    def update(self, 
            q_opt: np.ndarray):
        
        # print(f"AAAAAAAAAAA{q_opt.shape[0]}{q_opt.shape[1]}")
        # print(f"{q_opt}")

        # update rhc_q matrix from Horizon solution dictionary
        self.rhc_q[:, :] = q_opt[0:self.rhc_q.shape[0], 0:self.rhc_q.shape[1]]

        # writing q to shared memory
        self.server_factories[0].write(self.rhc_q[:, :], 0, 0)

    def close(self):

        for i in range(len(self.server_factories)):

            self.server_factories[i].close() # closes servers

    def _init_rhc_q_bridge(self):

        # rhc internal state
        self.server_factories.append(ServerFactory(n_rows = self.n_rows, 
                                        n_cols = self.n_cols,
                                        basename = "",
                                        namespace = self.names.get_rhc_q_name(), 
                                        verbose = self.verbose, 
                                        vlevel = VLevel.V3, 
                                        force_reconnection = False, 
                                        dtype = dtype.Float,
                                        layout = self.layout)
                                    )
        