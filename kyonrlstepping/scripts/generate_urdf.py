#!/usr/bin/env python
from rhcviz.RHCViz import RHCViz
from kyonrlstepping.utils.kyon_urdf_gen import KyonUrdfGen

import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Multi Robot Visualizer")
    parser.add_argument('--dpath', type=str, help="Path where the descr. files will be dumped")
    args = parser.parse_args()
    
    # generating description files for Kyon
    
    robot_type = "kyon"
    kyon_urdf_gen = KyonUrdfGen(descr_path = args.dpath, 
            robotname=robot_type, 
            name=robot_type+"Urdf")