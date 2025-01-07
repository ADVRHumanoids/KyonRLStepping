import os

class PathsGetter:

    def __init__(self):
        
        self.ROOT_DIR = os.path.dirname(os.path.dirname(__file__))

        self.CONTROLLER_ROOT_DIR = os.path.join(self.ROOT_DIR, 
                                        'controllers',
                                        'horizon_based')

        self.CONTROLLER_CFGS_DIR = os.path.join(self.CONTROLLER_ROOT_DIR, 
                                        'cfgs')
        
        self.RHCCONFIGPATH_NO_WHEELS = os.path.join(self.CONTROLLER_CFGS_DIR, 
                                        'kyon_rhc_config_no_wheels')
        
        self.RHCCONFIGPATH_WHEELS = os.path.join(self.CONTROLLER_CFGS_DIR, 
                                        'kyon_rhc_wheels')
        
        self.RHCCONFIGPATH_B2W = os.path.join(self.CONTROLLER_CFGS_DIR, 
                                        'b2w_rhc_continuous')
        
        self.RHCCONFIGPATH_WHEELS_CONTINUOUS = os.path.join(self.CONTROLLER_CFGS_DIR, 
                                        'kyon_rhc_wheels_continuous')

        self.JNT_IMP_CONFIG = os.path.join(self.ROOT_DIR, 
                                        'config',
                                        'jnt_imp_config')
        
        self.JNT_IMP_CONFIG_XBOT = os.path.join(self.ROOT_DIR, 
                                        'config',
                                        'xmj_env_files',
                                        'xbot2_basic')

if __name__ == "__main__":  

    paths = PathsGetter()
    print(paths.ROOT_DIR)
    print(paths.CONTROLLER_ROOT_DIR)
    print(paths.RHCCONFIGPATH_NO_WHEELS)
    print(paths.RHCCONFIGPATH_WHEELS)