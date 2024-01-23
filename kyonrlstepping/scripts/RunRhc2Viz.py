#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from kyonrlstepping.utils.kyon_urdf_gen import KyonUrdfGen
from rhcviz.utils.sys_utils import PathsGetter

import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('--robot_type', type=str, default='kyon')
    parser.add_argument('--robot_name', type=str, default='kyon')
    parser.add_argument('--nodes_perc', type=int, default=100)
    args = parser.parse_args()
    
    syspaths = PathsGetter()
    
    # generating urdf using rospkg (robot desc. need to be visible
    # to ROS)

    kyon_urdf_gen = KyonUrdfGen(robotname=args.robot_type, 
            name= args.robot_type + "Urdf")
    
    rhcviz = RHCViz(urdf_file_path=kyon_urdf_gen.urdf_path, 
        rviz_config_path=syspaths.DEFAULT_RVIZ_CONFIG_PATH,
        namespace=args.robot_name, 
        basename="RHCViz", 
        rate = 10,
        cpu_cores = [12],
        use_only_collisions=False,
        nodes_perc = args.nodes_perc       
        )
    
    rhcviz.run()