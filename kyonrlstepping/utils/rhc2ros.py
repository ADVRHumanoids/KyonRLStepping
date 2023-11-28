from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType, dtype

import numpy as np
import torch

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions

from control_cluster_bridge.utilities.defs import Journal
from control_cluster_bridge.utilities.shared_mem import SharedMemClient
from control_cluster_bridge.utilities.defs import cluster_size_name

from kyonrlstepping.utils.rhc2shared import RHC2SharedNamings

import rospy
from std_msgs.msg import Float64MultiArray

class Shared2ROSInternal:

    # bridge from shared mem to ROS
    
    def __init__(self, 
            namespace: str, 
            verbose = False,
            shared_mem_basename: str = "RHC2SharedInternal",
            rhcviz_basename = "RHCViz"):

        self.journal = Journal() # for printing stuff

        self.verbose = verbose

        self.shared_mem_basename = shared_mem_basename
        self.namespace = namespace # defines uniquely the kind of controller 
        # (associated with a specific robot)
        
        # to retrieve the number of controllers (associated with namespace)
        self.cluster_size_clnt = SharedMemClient(name=cluster_size_name(), 
                                    namespace=self.namespace,
                                    dtype=torch.int64, 
                                    wait_amount=0.05, 
                                    verbose=self.verbose)
        self.cluster_size_clnt.attach()
        self.cluster_size = self.cluster_size_clnt.tensor_view[0, 0].item()

        # shared mem. namings
        self.names = []
        for i in range(self.cluster_size):

            self.names.append(RHC2SharedNamings(basename = self.shared_mem_basename, 
                            namespace = self.namespace, 
                            index = i))
        
        # ros stuff
        self.ros_names = NamingConventions()
        self.rhcviz_basename = rhcviz_basename

        self.handshaker = RHCVizHandshake(self.ros_names.handshake_topicname(basename=self.rhcviz_basename, 
                                            namespace=namespace), 
                            is_server=True)
        
        self.rhc_q_pub = rospy.Publisher(self.ros_names.rhc_q_topicname(basename=self.rhcviz_basename, 
                                        namespace=namespace), 
                            Float64MultiArray, 
                            queue_size=10)

        # other data
        self.floating_base_q_dim = 7 # orientation quat.
        
        self.dtype = np.float32
        self.layout = RowMajor
        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'
        
        self.client_factories = []

        self._init_rhc_q_bridge() # init. shared mem. clients

        self._initialized = False
    
    def run(self):
        
        # starts clients and runs ros bridge

        for i in range(len(self.client_factories)):

            self.client_factories[i].attach() 

        # we assume all clients to be of the same controller, for
        # the same robot
        self.n_rows = self.client_factories[0].getNRows()
        self.n_cols = self.client_factories[0].getNCols()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.client_factories[0].getScalarType()),
                                order=self.order)
        
        rospy.init_node('RHC2ROSBridge')

        self.handshaker.set_n_nodes(self.n_cols) # signal to RHViz client
        # the number of nodes of the RHC problem

        self._initialized = True

    def update(self, 
            index: int = 0):
        
        success = False

        if self._initialized:
            
            # first read from shared memory so that rhc_q is updated
            # we read from controller at index index
            success = self.client_factories[index].read(self.rhc_q[:, :], 0, 0)

            # publish it on ROS topic

            self._publish()
        
        if not success:

            warning = f"[{self.__class__.__name__}" + "]" + \
                f"[{self.journal.warning}]" + \
                ": failed to read rhc_q from shared memory"
            
            print(warning)

        return success

    def close(self):

        for i in range(len(self.client_factories)):

            self.client_factories[i].close() # closes servers

    def _publish(self):

        # Publish rhc_q
        self.rhc_q_pub.publish(Float64MultiArray(data=self.rhc_q.flatten()))

    def _init_rhc_q_bridge(self):
        
        for i in range(self.cluster_size):

            # we create a client for each controller in the cluster
            # at runtime no overhead, since we only update the data with one, 
            # depending on the requested index

            # rhc internal state
            self.client_factories.append(ClientFactory(
                                            basename = "",
                                            namespace = self.names[i].get_rhc_q_name(), 
                                            verbose = self.verbose, 
                                            vlevel = VLevel.V3, 
                                            dtype = dtype.Float,
                                            layout = self.layout)
                                        )