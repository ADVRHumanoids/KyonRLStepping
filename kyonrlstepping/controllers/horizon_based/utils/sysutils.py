import os

class PathsGetter:

    def __init__(self):
        
        self.CONTROLLER_ROOT_DIR = os.path.dirname(os.path.dirname(__file__))

        self.RHCCONFIGPATH_NO_WHEELS = os.path.join(self.CONTROLLER_ROOT_DIR, 
                                        'kyon_rhc_config_no_wheels.yaml')
        
        self.RHCCONFIGPATH_WHEELS = os.path.join(self.CONTROLLER_ROOT_DIR, 
                                        'kyon_rhc_wheels.yaml')
