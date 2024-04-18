from rhcviz.utils.xrdf_gen import UrdfGenerator

class KyonUrdfGen(UrdfGenerator):

    def __init__(self, 
            robotname: str,
            descr_path: str,
            wheels: bool = False,
            name: str = "KyonUrdfRHCViz"):
        
        super().__init__(
            robotname = robotname,
            descr_path = descr_path,
            name = name)

        self._wheels = wheels

        self.generate_urdf() # actually generated urdf

    def _xrdf_cmds(self):
        
        # implements parent method 

        cmds = {} 
        
        cmds[self.robotname] = self._get_xrdf_cmds_kyon(root=self.descr_path)
        
        return cmds
    
    def _get_xrdf_cmds_kyon(self,
                root: str):
        
        cmds = []
        
        cmds.append("kyon_root:=" + root)
        if self._wheels:
            cmds.append("wheels:=true")
        else:
            cmds.append("wheels:=false")
        cmds.append("upper_body:=false")
        cmds.append("dagana:=false")
        cmds.append("sensors:=false")
        cmds.append("floating_joint:=false")
        cmds.append("payload:=false")
        cmds.append("use_abs_mesh_paths:=true")
        cmds.append("use_local_filesys_for_meshes:=true")

        return cmds
