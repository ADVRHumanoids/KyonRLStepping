from rhcviz.utils.xrdf_gen import UrdfGenerator

class B2WUrdfGen(UrdfGenerator):

    def __init__(self, 
            robotname: str,
            descr_path: str,
            wheels: bool = True,
            name: str = "B2WUrdfRHCViz"):
        
        super().__init__(
            robotname = robotname,
            descr_path = descr_path,
            name = name)

        self._wheels = wheels

        self.generate_urdf(folder_name="xacro") # actually generated urdf

    def _xrdf_cmds(self):
        
        # implements parent method 

        cmds = {} 
        
        cmds[self.robotname] = self._get_xrdf_cmds_b2w(root=self.descr_path)
        
        return cmds
    
    def _get_xrdf_cmds_b2w(self,
                root: str):
        
        cmds = []
        
        cmds.append("root:=" + root)
        if self._wheels:
            cmds.append("wheels:=true")
        else:
            cmds.append("wheels:=false")
        cmds.append("floating_joint:=false")
        cmds.append("use_abs_mesh_paths:=true")
        cmds.append("use_local_filesys_for_meshes:=true")

        return cmds
