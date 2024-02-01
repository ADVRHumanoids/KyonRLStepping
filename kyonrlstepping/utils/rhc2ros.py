from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType, dtype

import numpy as np
import torch

from rhcviz.utils.handshake import RHCVizHandshake
from rhcviz.utils.namings import NamingConventions
from rhcviz.utils.string_list_encoding import StringArray

from control_cluster_bridge.utilities.defs import Journal

from control_cluster_bridge.utilities.shared_data.rhc_data import RhcStatus, RobotState

from SharsorIPCpp.PySharsorIPC import dtype

from SharsorIPCpp.PySharsor.wrappers.shared_data_view import SharedDataView
from SharsorIPCpp.PySharsorIPC import VLevel

from kyonrlstepping.utils.rhc2shared import RHC2SharedNamings

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

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
        
        # to retrieve the number of controllers
        self.rhc_status = RhcStatus(is_server=False,
                                    namespace=self.namespace, 
                                    verbose=True,
                                    vlevel=VLevel.V2)
        self.rhc_status.run()
        self.cluster_size = self.rhc_status.cluster_size
        self.rhc_status.close() # not needed anymore

        self.env_index = SharedDataView(namespace = self.namespace,
                basename = "EnvSelector",
                is_server = False, 
                verbose = True, 
                vlevel = VLevel.V2,
                safe = False,
                dtype=dtype.Int)
        
        self.env_index.run()

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
                                            namespace=self.namespace), 
                            is_server=True)
        
        self.rhc_q_pub = rospy.Publisher(self.ros_names.rhc_q_topicname(basename=self.rhcviz_basename, 
                                        namespace=self.namespace), 
                            Float64MultiArray, 
                            queue_size=10)

        self.robot_q_pub = rospy.Publisher(self.ros_names.robot_q_topicname(basename=self.rhcviz_basename, 
                                        namespace=self.namespace), 
                            Float64MultiArray, 
                            queue_size=10)
        
        self.robot_jntnames_pub = rospy.Publisher(self.ros_names.robot_jntnames(basename=self.rhcviz_basename, 
                                        namespace=self.namespace), 
                            String, 
                            queue_size=10)
        
        # other data
        self.floating_base_q_dim = 7 # orientation quat.
        
        self.dtype = np.float32
        self.layout = RowMajor
        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'
        
        self.client_factories_rhc_q = []
        self.client_factories_robot_q = []

        self.cluster_jnt_names = []

        self._init_rhc_q_bridge() # init. shared mem. clients for rhc q
        self._init_robot_q_bridge() # init. shared mem. clients for robot q

        self.rhc_q = None
        self.robot_q = None

        self._initialized = False
    
    def run(self):
        
        # starts clients and runs ros bridge

        for i in range(len(self.client_factories_rhc_q)):

            self.client_factories_rhc_q[i].attach() 

        for i in range(len(self.client_factories_robot_q)):

            self.client_factories_robot_q[i].attach()

        # jnt names from shared mem
        robot_state = RobotState(namespace=self.namespace,
                            is_server=False, 
                            with_gpu_mirror=False, 
                            safe=False,
                            verbose=self.verbose,
                            vlevel=VLevel.V2)
        robot_state.run()

        self.cluster_jnt_names = robot_state.jnt_names()
        robot_state.close() # not needed anymore
        
        rospy.init_node('RHC2ROSBridge')

        # publishing joint names on topic 
        string_array = StringArray()
        self.jnt_names_encoded = string_array.encode(self.cluster_jnt_names) # encoding 
        # jnt names in a ; separated string

        # we assume all clients to be of the same controller, for
        # the same robot

        # rhc q
        self.n_rows = self.client_factories_rhc_q[0].getNRows()
        self.n_cols = self.client_factories_rhc_q[0].getNCols()

        self.rhc_q = np.zeros((self.n_rows, self.n_cols),
                    dtype=toNumpyDType(self.client_factories_rhc_q[0].getScalarType()),
                    order=self.order)
        self.rhc_q[6, :] = 1 # initializing to valid identity quaternion

        # robot q
        if self.n_rows != self.client_factories_robot_q[0].getNRows():
            
            excpetion = f"[{self.__class__.__name__}" + "]" + \
                f"[{self.journal.exception}]" + \
                ": q dimension from RHC controllers does not math q dimension from robot state!!"
            
            raise Exception(excpetion)
        if self.client_factories_robot_q[0].getNCols() != 1:
            
            excpetion = f"[{self.__class__.__name__}" + "]" + \
                f"[{self.journal.exception}]" + \
                ": robot state should have only one column!!"
            
            raise Exception(excpetion)
        
        self.robot_q = np.zeros((self.n_rows, 1),
                    dtype=toNumpyDType(self.client_factories_robot_q[0].getScalarType()),
                    order=self.order
                    )
        self.robot_q[6, :] = 1 # initializing to valid identity quaternion

        self.handshaker.set_n_nodes(self.n_cols) # signal to RHViz client
        # the number of nodes of the RHC problem

        self._initialized = True

    def update(self):
        
        success = False

        self.env_index.synch_all(read=False, wait=True)

        cluster_idx = self.env_index.torch_view[0, 0].item()

        if self._initialized:
            
            # first read from shared memory so 

            # rhc q
            success = self.client_factories_rhc_q[cluster_idx].read(self.rhc_q[:, :], 0, 0)

            # robot state

            success = self.client_factories_robot_q[cluster_idx].read(self.robot_q[:, :], 0, 0)

            # publish it on ROS topic

            self._publish()
        
        if not success:

            warning = f"[{self.__class__.__name__}" + "]" + \
                f"[{self.journal.warning}]" + \
                ": failed to read either rhc_q or robot_q from shared memory"
            
            print(warning)

        return success

    def close(self):

        for i in range(len(self.client_factories_rhc_q)):

            self.client_factories_rhc_q[i].close() # closes servers

    def _publish(self):
        
        # continously publish also joint names 
        self.robot_jntnames_pub.publish(String(data=self.jnt_names_encoded))

        # publish rhc_q
        self.rhc_q_pub.publish(Float64MultiArray(data=self.rhc_q.flatten()))

        # publish robot_q
        self.robot_q_pub.publish(Float64MultiArray(data=self.robot_q.flatten()))

    def _init_rhc_q_bridge(self):
        
        for i in range(self.cluster_size):

            # we create a client for each controller in the cluster
            # at runtime no overhead, since we only update the data with one, 
            # depending on the requested index

            # rhc internal state
            self.client_factories_rhc_q.append(ClientFactory(
                                            basename = "",
                                            namespace = self.names[i].get_rhc_q_name(), 
                                            verbose = self.verbose, 
                                            vlevel = VLevel.V2, 
                                            dtype = dtype.Float,
                                            layout = self.layout)
                                        )
            
    def _init_robot_q_bridge(self):

        for i in range(self.cluster_size):

            # we create a client for each controller in the cluster
            # at runtime no overhead, since we only update the data with one, 
            # depending on the requested index

            # robot actual state (either measured or simulated)
            self.client_factories_robot_q.append(ClientFactory(
                                            basename = "",
                                            namespace = self.names[i].get_robot_q_name(), 
                                            verbose = self.verbose, 
                                            vlevel = VLevel.V2, 
                                            dtype = dtype.Float,
                                            layout = ColMajor # ColMajor to circumvent Numpy bug when slicing 2D matrices as 1D
                                            )
                                        )