import os

class PathsGetter:

    def __init__(self):
        
        self.PACKAGE_ROOT_DIR = os.path.dirname(os.path.dirname(__file__))

        self.CONFIGPATH = os.path.join(self.PACKAGE_ROOT_DIR, 
                                        'config', 
                                        'kyon_horizon_wheel_config.yaml')