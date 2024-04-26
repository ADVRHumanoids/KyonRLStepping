#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from rhcviz.utils.sys_utils import PathsGetter

from kyonrlstepping.utils.kyon_urdf_gen import KyonUrdfGen

import os
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('--ns', type=str, help='Namespace to be used for cluster shared memory',default="kyon0")
    parser.add_argument('--dpath', type=str,default="/root/ibrido_ws/src/iit-kyon-ros-pkg/kyon_urdf")
    parser.add_argument('--nodes_perc', type=int, default=30)
    parser.add_argument('--comment', type=str, help='Any useful comment associated with this run',default="")
    parser.add_argument('--wheels', action='store_true', help='add wheels')

    args = parser.parse_args()

    syspaths = PathsGetter()
        
    urdf_generator = KyonUrdfGen(robotname="kyon", 
                wheels=args.wheels,
                descr_path=args.dpath,
                name="kyonUrdf")
    
    rhcviz = RHCViz(urdf_file_path=urdf_generator.urdf_path, 
        rviz_config_path=syspaths.DEFAULT_RVIZ_CONFIG_PATH,
        namespace=args.ns, 
        basename="RHCViz", 
        rate = 100,
        use_only_collisions=False,
        nodes_perc = args.nodes_perc       
        )
    
    rhcviz.run()
