#!/usr/bin/env python
from mpc_viz.MPCViz import MPCViz
from mpc_viz.utils.sys_utils import PathsGetter

from kyonrlstepping.utils.kyon_urdf_gen import KyonUrdfGen
from kyonrlstepping.utils.kyon_real_urdf_gen import KyonRealUrdfGen

from kyonrlstepping.utils.b2w_urdf_gen import B2WUrdfGen

import os
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('--ns', type=str, help='Namespace to be used for cluster shared memory',default="kyon0")
    parser.add_argument('--dpath', type=str,default=None)
    parser.add_argument('--nodes_perc', type=int, default=30)
    parser.add_argument('--comment', type=str, help='Any useful comment associated with this run',default="")
    parser.add_argument('--wheels', action='store_true', help='add wheels')
    parser.add_argument('--b2w', action='store_true', help='use unitree b2s')
    parser.add_argument('--kyon_real', action='store_true', help='use real Kyon prototype')
    parser.add_argument('--blink_name', type=str,default=None)
    parser.add_argument('--show_heightmap', action='store_true', help='Visualize heightmap markers if available')

    args = parser.parse_args()

    syspaths = PathsGetter()

    blink_name=args.blink_name
    if blink_name is None:
        if not args.b2w:
            blink_name="base_link"
            if args.kyon_real:
                blink_name="pelvis"
        else:
            blink_name="base"

    dpath=args.dpath
    if dpath is None:
        if not args.b2w:
            dpath="/root/ibrido_ws/src/iit-kyon-ros-pkg/kyon_urdf" 
            if args.kyon_real:
                dpath="/root/ibrido_ws/src/iit-kyon-description/kyon_urdf" 
        else:
            dpath="/root/ibrido_ws/src/unitree_ros/robots/b2w_description" 

    urdf_generator=None
    if not args.b2w:
        if args.kyon_real:
            urdf_generator = KyonUrdfGen(robotname="kyon", 
                        wheels=args.wheels,
                        descr_path=dpath,
                        name="kyonUrdf")
        else:
            urdf_generator = KyonRealUrdfGen(robotname="kyon", 
                        wheels=args.wheels,
                        descr_path=dpath,
                        name="kyonUrdf")
    else:
        urdf_generator = B2WUrdfGen(robotname="robot", 
                    wheels=True,
                    descr_path=dpath,
                    name="B2WUrdf")
        
    mpc_viz= MPCViz(urdf_file_path=urdf_generator.urdf_path, 
        rviz_config_path=syspaths.DEFAULT_RVIZ_CONFIG_PATH,
        namespace=args.ns, 
        basename="MPCViz", 
        rate = 100,
        use_only_collisions=False,
        nodes_perc = args.nodes_perc,
        base_link_name=blink_name,
        show_heightmap=args.show_heightmap,
        )
    
    mpc_viz.run()
