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

import rospy
from std_msgs.msg import Float64MultiArray

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
            verbose = False,
            shared_mem_basename: str = "RHC2SharedInternal"):

        self.journal = Journal()

        self.verbose = verbose

        self.shared_mem_basename = shared_mem_basename
        self.namespace = namespace # defines uniquely the kind of controller 
        # (associated with a specific robot)

        self.cluster_size_clnt = SharedMemClient(name=cluster_size_name(), 
                                    namespace=self.namespace,
                                    dtype=torch.int64, 
                                    wait_amount=0.05, 
                                    verbose=self.verbose)
        self.cluster_size_clnt.attach()
        self.cluster_size = self.cluster_size_clnt.tensor_view[0, 0].item()

        self.names = []

        for i in range(self.cluster_size):

            self.names.append(RHC2ROSNamings(basename = self.shared_mem_basename, 
                            namespace = self.namespace, 
                            index = i))
        
        self.ros_names = NamingConventions()

        ros_basename = "RHCViz_test"
        handshake_topicname = self.ros_names.handshake_topicname(basename=ros_basename, 
                                            namespace=namespace)

        rhc_q_topic_name = self.ros_names.rhc_q_topicname(basename=ros_basename, 
                                        namespace=namespace)
        
        self.handshaker = RHCVizHandshake(handshake_topicname, 
                            is_server=False)
        
        self.pub = rospy.Publisher(rhc_q_topic_name, 
                            Float64MultiArray, 
                            queue_size=10)
    
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

        # we assume all clients to be of the same controller, for
        # the same robot
        self.n_rows = self.client_factories[0].getNRows()
        self.n_cols = self.client_factories[0].getNCols()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.client_factories[0].getScalarType()),
                                order=self.order)
        
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

    def _ros_handshake(handshaker: RHCVizHandshake):
        
        # Wait for handshake to complete
        while not rospy.is_shutdown() and not handshaker.handshake_done():

            rospy.sleep(0.1)

        if handshaker.n_nodes is None:

            rospy.logerr("Handshake not completed. Exiting.")

            return
    
        return handshaker.n_nodes

    def _publish(self):

        a = 1

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