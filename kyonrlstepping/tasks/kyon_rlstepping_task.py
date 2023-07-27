from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.viewports import set_camera_view

from omni.isaac.core.utils.rotations import euler_angles_to_quat 

import omni.kit

import gymnasium as gym
from gym import spaces
import numpy as np
import torch
import math

from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.prims import move_prim
from omni.isaac.cloner import GridCloner
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.urdf._urdf import UrdfJointTargetType

from omni.isaac.core.utils.types import ArticulationActions

from omni.isaac.core.scenes.scene import Scene

from kyonrlstepping.utils.jnt_imp_cntrl import OmniJntImpCntrl

class KyonRlSteppingTask(BaseTask):
    def __init__(self, 
                name: str, 
                num_envs = 1,
                device = "cuda", 
                robot_offset: np.array = np.array([0.0, 0.0, 0.0]),
                replicate_physics: bool = True,
                offset=None, 
                env_spacing = 5.0) -> None:

        self.info = "info"
        self.status = "status"
        self.warning = "warning"
        self.exception = "exception"
        
        # cloning stuff
        self.num_envs = num_envs
        self._env_ns = "/World/envs"
        self._env_spacing = env_spacing # [m]
        self._template_env_ns = self._env_ns + "/env_0"

        self._cloner = GridCloner(spacing=self._env_spacing)
        self._cloner.define_base_env(self._env_ns)

        prim_utils.define_prim(self._template_env_ns)
        self._envs_prim_paths = self._cloner.generate_paths(self._env_ns + "/env", 
                                                self.num_envs)

        # task-specific parameters
        if len(robot_offset) != 3:
            robot_offset = np.array([0.0, 0.0, 0.0])
            print(f"[{self.__class__.__name__}]" + f"[{self.warning}]" + ":  the provided robot_offset is not of the correct shape. A null offset will be used instead.")

        self._robot_offset = robot_offset

        self._position = np.array([0, 0, 0.8])
        self._orientation = euler_angles_to_quat(np.array([0, 0, 0]), degrees = True)

        # values used for defining RL buffers
        self._num_observations = 4
        self._num_actions = 1
        self._device = device # defaults to "cuda" ("cpu" also valid)

        # a few class buffers to store RL-related states
        self.obs = torch.zeros((self.num_envs, self._num_observations))
        self.resets = torch.zeros((self.num_envs, 1))

        # set the action and observation space for RL
        self.action_space = spaces.Box(np.ones(self._num_actions) * -1.0, np.ones(self._num_actions) * 1.0)
        self.observation_space = spaces.Box(
            np.ones(self._num_observations) * -np.Inf, np.ones(self._num_observations) * np.Inf
        )

        self._replicate_physics = replicate_physics

        self._world_initialized = False

        self._robot_bodynames = []
        self._robot_n_links = -1
        self._robot_n_dofs = -1
        self._robot_dof_names = []

        self._ground_plane_prim_path = "/World/ground_plane"

        # trigger __init__ of parent class
        BaseTask.__init__(self,
                        name=name, 
                        offset=offset)
        
        self._jnt_imp_controller = None 
    
    def _generate_srdf(self,
                    wheels: bool = True, 
                    arms: bool = False, 
                    payload: bool = False):
        
        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        descr_path = rospackage.get_path("kyon_srdf")
        srdf_path = descr_path + "/srdf"
        kyon_xacro_name = "kyon"
        xacro_path = srdf_path + "/" + kyon_xacro_name + ".srdf.xacro"
        self._srdf_path = srdf_path + "/" + kyon_xacro_name + ".srdf"
        
        wheel_cmd_val = "true"
        arms_cmd_val = "false"
        payload_cmd_val = "false"
        if wheels:
            wheel_cmd_val = "true"
        else:
            wheel_cmd_val = "false"
        if arms:
            arms_cmd_val = "true"
        else:
            arms_cmd_val = "false"
        if payload:
            payload_cmd_val = "true"
        else:
            payload_cmd_val = "false"

        add_wheels = "wheels:=" + wheel_cmd_val
        add_arms = "upper_body:=" + arms_cmd_val
        remove_sensors = "sensors:=" + "false"
        remove_floating_joint = "floating_joint:=" + "false"
        no_payload = "payload:=" + payload_cmd_val
        
        import subprocess
        try:

            xacro_gen = subprocess.check_call(["xacro",\
                                            xacro_path, \
                                            add_wheels, \
                                            add_arms, \
                                            remove_sensors, \
                                            no_payload, \
                                            remove_floating_joint, \
                                            "-o", 
                                            self._srdf_path])

        except:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + ": failed to generate kyon\'S SRDF!!!")
        
    def _generate_urdf(self, 
                    wheels: bool = True, 
                    arms: bool = False):

        # we generate the URDF where the Kyon description package is located
        import rospkg
        rospackage = rospkg.RosPack()
        descr_path = rospackage.get_path("kyon_urdf")
        urdf_path = descr_path + "/urdf"
        kyon_xacro_name = "kyon"
        xacro_path = urdf_path + "/" + kyon_xacro_name + ".urdf.xacro"
        self._urdf_path = urdf_path + "/" + kyon_xacro_name + ".urdf"
        
        wheel_cmd_val = "true"
        arms_cmd_val = "false"
        if wheels:
            wheel_cmd_val = "true"
        else:
            wheel_cmd_val = "false"
        if arms:
            arms_cmd_val = "true"
        else:
            arms_cmd_val = "false"
        add_wheels = "wheels:=" + wheel_cmd_val
        add_arms = "upper_body:=" + arms_cmd_val

        remove_floating_joint = "floating_joint:=" + "false"
        remove_sensors = "sensors:=" + "false"
        no_payload = "payload:=" + "false"

        import subprocess
        try:

            xacro_gen = subprocess.check_call(["xacro",\
                                            xacro_path, \
                                            add_wheels, \
                                            add_arms, \
                                            remove_floating_joint, \
                                            remove_sensors, \
                                            no_payload, \
                                            "-o", 
                                            self._urdf_path])
            
            # we also generate an updated SRDF (used by controllers)

        except:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + ": failed to generate kyon\'S URDF!!!")

    def _generate_description(self):
        
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": generating URDF...")
        self._generate_urdf()
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": done")

        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": generating SRDF...")
        # we also generate SRDF files, which are useful for control
        self._generate_srdf()
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": done")

    def _import_urdf(self, 
                    import_config: omni.isaac.urdf._urdf.ImportConfig = _urdf.ImportConfig(), 
                    robot_prim_name: str = "Kyon"):

        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": importing robot URDF")

        self._urdf_import_config = import_config
        # we overwrite some settings which are bound to be fixed
        self._urdf_import_config.merge_fixed_joints = True # makes sim more stable
        # in case of fixed joints with light objects
        self._urdf_import_config.import_inertia_tensor = True
        self._urdf_import_config.fix_base = False
        self._urdf_import_config.self_collision = False

        self._urdf_interface = _urdf.acquire_urdf_interface()

        self._robot_prim_name = robot_prim_name
        
        # import URDF
        success, robot_prim_path_default = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=self._urdf_path,
            import_config=import_config, 
        )
        self._robot_base_prim_path = self._template_env_ns + "/" + self._robot_prim_name
        move_prim(robot_prim_path_default, self._robot_base_prim_path)# we move the prim
        # from the default one of the URDF importer to the prescribed one

        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": done")

        return success
    
    def _get_jnts_state(self):

        self.robot_jnt_positions = self._robots_art_view.get_joint_positions(indices = None, 
                                        joint_indices = None, 
                                        clone = True)
        
        self.robot_jnt_velocities = self._robots_art_view.get_joint_velocities(indices = None, 
                                        joint_indices = None, 
                                        clone = True)

    def world_was_initialized(self):

        self._world_initialized = True

    def set_robot_default_jnt_config(self, 
                                hip_roll = None, 
                                hip_pitch = None, 
                                knee_pitch = None, 
                                wheels = None):
        
        if hip_roll is None:
            hip_roll = torch.tensor([0.3, -0.3, 0.3, -0.3], device=self._device)
        if hip_pitch is None:
            hip_pitch = torch.tensor([-0.3, -0.3, 0.3, 0.3], device=self._device)
        if knee_pitch is None:
            knee_pitch = torch.tensor([0.3, 0.3, -0.3, -0.3], device=self._device)
        if wheels is None:
            wheels = torch.tensor([0.0, 0.0, 0.0, 0.0], device=self._device)

        if (self._world_initialized):

            self._default_jnt_positions = torch.zeros((self.num_envs, self._robot_n_dofs), 
                                                device=self._device)
            
            default_jnt_positions = torch.cat((hip_roll, 
                                                hip_pitch, 
                                                knee_pitch, 
                                                wheels),
                                                0)

            for i in range(0,  self.num_envs):

                self._default_jnt_positions[i, :] = default_jnt_positions

            self._robots_art_view.set_joints_default_state(positions= self._default_jnt_positions)
            
        else:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + \
                        "Before calling _set_robot_default_jnt_config(), you need to reset the World" + \
                        " at least once and call _world_was_initialized()")

    def set_robot_root_default_config(self):
        
        # To be implemented

        return True

    # def override_pd_controller_gains(self):
        
    #     # all gains set to 0 so that it's possible to 
    #     # attach to the articulation a custom joint controller (e.g. jnt impedance), 
    #     # on top of the default articulation pd controller

    #     self.joint_kps_envs = torch.zeros((self.num_envs, self._robot_n_dofs))
    #     self.joint_kds_envs = torch.zeros((self.num_envs, self._robot_n_dofs)) 

    #     self._robots_art_view.set_gains(kps= self.joint_kps_envs, 
    #                                     kds= self.joint_kds_envs)
        
    def apply_collision_filters(self, 
                                physicscene_path: str, 
                                coll_root_path: str):

        self._cloner.filter_collisions(physicsscene_path = physicscene_path,
                                collision_root_path = coll_root_path, 
                                prim_paths=self._envs_prim_paths, 
                                global_paths=[self._ground_plane_prim_path] # can collide with these prims
                            )

    def print_envs_info(self):
        
        if (self._world_initialized):

            print("TASK INFO:")
            print("Envs bodies: " + str(self._robots_art_view.body_names))
            print("n. prims: " + str(self._robots_art_view.count))
            print("prims names: " + str(self._robots_art_view.prim_paths))
            print("n. bodies: " + str(self._robots_art_view.num_bodies))
            print("n. dofs: " + str(self._robots_art_view.num_dof))
            print("dof names: " + str(self._robots_art_view.dof_names))
            # print("dof limits: " + str(self._robots_art_view.get_dof_limits()))
            # print("effort modes: " + str(self._robots_art_view.get_effort_modes()))
            # print("dof gains: " + str(self._robots_art_view.get_gains()))
            # print("dof max efforts: " + str(self._robots_art_view.get_max_efforts()))
            # print("dof gains: " + str(self._robots_art_view.get_gains()))
            # print("physics handle valid: " + str(self._robots_art_view.is_physics_handle_valid()))

        else:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + \
                            "Before calling _print_envs_info(), you need to reset the World at least once!")

    def fill_robot_info_from_world(self):

        if (self._world_initialized):

            self._robot_bodynames = self._robots_art_view.body_names
            self._robot_n_links = self._robots_art_view.num_bodies
            self._robot_n_dofs = self._robots_art_view.num_dof
            self._robot_dof_names = self._robots_art_view.dof_names
        
        else:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + \
                        "Before calling _get_robot_info_from_world(), you need to reset the World at least once!")

    def init_imp_control(self, 
                default_jnt_pgain = 300.0, 
                default_jnt_vgain = 30.0, 
                default_wheel_pgain = 0.0, 
                default_wheel_vgain = 10.0):
        
        if self.world_was_initialized:

            self._jnt_imp_controller = OmniJntImpCntrl(articulation=self._robots_art_view,
                                                default_pgain = default_jnt_pgain, 
                                                default_vgain = default_jnt_vgain,
                                                device= self._device)

            # we override internal default gains for the wheels, which are usually
            # velocity controlled
            wheels_indxs = self._jnt_imp_controller.get_jnt_idxs_matching(name_pattern="wheel")

            wheels_pos_gains = torch.full((self.num_envs, len(wheels_indxs)), 
                                        default_wheel_pgain, 
                                        device = self._device, 
                                        dtype=torch.float32)
            wheels_vel_gains = torch.full((self.num_envs, len(wheels_indxs)), 
                                        default_wheel_vgain, 
                                        device = self._device, 
                                        dtype=torch.float32)

            success_imp_gains = self._jnt_imp_controller.set_gains(pos_gains = wheels_pos_gains,
                                        vel_gains = wheels_vel_gains,
                                        jnt_indxs=wheels_indxs)

            success_set_refs = self._jnt_imp_controller.set_refs(pos_ref=self._default_jnt_positions)

            print(self._default_jnt_positions.shape)
            # we update the internal references on the imp. controller using 
            # measured states, for smoothness sake
            print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": set_ref success [eff, pos, vel, idx, assign] -> " + str(success_set_refs))
            print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": set_gains success [pos, vel, idx, assign] -> " + str(success_imp_gains))

        else:

            raise Exception(f"[{self.__class__.__name__}]" + f"[{self.exception}]" + ": you should reset the World at least once and call the " + \
                            "world_was_initialized() method before initializing the " + \
                            "joint impedance controller."
                            )
        
    def set_up_scene(self, 
                    scene: Scene) -> None:

        self._generate_description()

        self._import_urdf()
        
        pos_offsets = np.zeros((self.num_envs, 3))
        for i in range(0, self.num_envs):
            pos_offsets[i, :] = self._robot_offset
        
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": cloning environments")
        envs_positions = self._cloner.clone(
            source_prim_path=self._template_env_ns,
            prim_paths=self._envs_prim_paths,
            replicate_physics=self._replicate_physics,
            position_offsets = pos_offsets
        ) # robot is now at the default env prim --> we can clone the environment
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": done")

        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": finishing scene setup...")
        self._robots_art_view = ArticulationView(self._env_ns + "/env*"+ "/" + self._robot_prim_name, 
                                reset_xform_properties=False)

        self._robots_articulations = scene.add(self._robots_art_view)

        scene.add_default_ground_plane(z_position=0, 
                            name="ground_plane", 
                            prim_path= self._ground_plane_prim_path, 
                            static_friction=0.5, 
                            dynamic_friction=0.5, 
                            restitution=0.8)
        
        # delete_prim(self._ground_plane_prim_path + "/SphereLight") # we remove the default spherical light
        
        # set default camera viewport position and target
        self.set_initial_camera_params()
        print(f"[{self.__class__.__name__}]" + f"[{self.status}]" + ": done")

    def set_initial_camera_params(self, 
                                camera_position=[10, 10, 3], 
                                camera_target=[0, 0, 0]):
        
        set_camera_view(eye=camera_position, 
                        target=camera_target, 
                        camera_prim_path="/OmniverseKit_Persp")

    def post_reset(self):
        # self._cart_dof_idx = self._cartpoles.get_dof_index("cartJoint")
        # self._pole_dof_idx = self._cartpoles.get_dof_index("poleJoint")
        # # randomize all envs
        # indices = torch.arange(self._cartpoles.count, dtype=torch.int64, device=self._device)
        # self.reset(indices)

        a = 1

    def reset(self, env_ids=None):
        # if env_ids is None:
        #     env_ids = torch.arange(self.num_envs, device=self._device)
        # num_resets = len(env_ids)

        # # randomize DOF positions
        # dof_pos = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        # dof_pos[:, self._cart_dof_idx] = 1.0 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        # dof_pos[:, self._pole_dof_idx] = 0.125 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # # randomize DOF velocities
        # dof_vel = torch.zeros((num_resets, self._cartpoles.num_dof), device=self._device)
        # dof_vel[:, self._cart_dof_idx] = 0.5 * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))
        # dof_vel[:, self._pole_dof_idx] = 0.25 * math.pi * (1.0 - 2.0 * torch.rand(num_resets, device=self._device))

        # # apply resets
        # indices = env_ids.to(dtype=torch.int32)
        # self._cartpoles.set_joint_positions(dof_pos, indices=indices)
        # self._cartpoles.set_joint_velocities(dof_vel, indices=indices)

        # # bookkeeping
        # self.resets[env_ids] = 0

        a = 1

    def pre_physics_step(self, actions) -> None:
        
        wheel_idxs = self._jnt_imp_controller.get_jnt_idxs_matching("wheel")
        # pos_ref = torch.mul(torch.sub(torch.rand((1, len(knee_indxs)), device = self._device), 0.5), 2.0)
        eff_ref = torch.mul(torch.sub(torch.rand((1, len(wheel_idxs)), device = self._device), 0.5), 5.0)
        success = self._jnt_imp_controller.set_refs(eff_ref=eff_ref, 
                                        pos_ref=None, 
                                        vel_ref=None, 
                                        jnt_indxs=wheel_idxs, 
                                        robot_indxs=torch.tensor([0], device=self._device))

        self._jnt_imp_controller.apply_refs()

    def get_observations(self):

        self._get_jnts_state() # updates joints states

        return self.obs

    def calculate_metrics(self) -> None:

        # compute reward based on angle of pole and cart velocity
        reward = 0

        return reward

    def is_done(self) -> None:
        # cart_pos = self.obs[:, 0]
        # pole_pos = self.obs[:, 2]

        # # reset the robot if cart has reached reset_dist or pole is too far from upright
        # resets = torch.where(torch.abs(cart_pos) > self._reset_dist, 1, 0)
        # resets = torch.where(torch.abs(pole_pos) > math.pi / 2, 1, resets)
        # self.resets = resets

        return True
