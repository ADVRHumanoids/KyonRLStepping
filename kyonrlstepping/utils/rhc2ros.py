from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType, dtype

import numpy as np

import rospy

class RHC2ROSNamings:

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
        
class Shared2ROSInternal:

    # bridge from shared mem to ROS
    
    def __init__(self, 
            namespace: str, 
            index: int, 
            verbose = False,
            basename: str = "RHC2SharedInternal"):

        self.verbose = verbose

        self.index = index

        self.basename = basename
        self.namespace = namespace

        self.names = RHC2ROSNamings(basename = self.basename, 
                        namespace = self.namespace, 
                        index = self.index)
        
        self.floating_base_q_dim = 7 # orientation quat.
        
        self.dtype = np.float32
        self.layout = RowMajor
        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'
        
        self.client_factories = []

        self._init_rhc_q_bridge()

        self._initialized = False
    
    def run(self):

        for i in range(len(self.client_factories)):

            self.client_factories[i].attach() # starts clients

        self.n_rows = self.client_factories[0].getNRows()
        self.n_cols = self.client_factories[0].getNCols()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.client_factories[0].getScalarType()),
                                order=self.order)
        
    def update(self):
        
        if self._initialized:
            
            # first read from shared memory so that rhc_q is updated
            self.client_factories[0].read(self.rhc_q[:, :], 0, 0)

            # publish it on ROS topic

            self._publish()

    def close(self):

        for i in range(len(self.client_factories)):

            self.client_factories[i].close() # closes servers

    def _publish(self):

        a = 1

    def _init_rhc_q_bridge(self):

        # rhc internal state
        self.client_factories.append(ClientFactory(
                                        basename = "",
                                        namespace = self.names.get_rhc_q_name(), 
                                        verbose = self.verbose, 
                                        vlevel = VLevel.V1, 
                                        dtype = dtype.Float,
                                        layout = self.layout)
                                    )