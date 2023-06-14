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

class KyonRlSteppingTask(BaseTask):
    def __init__(self, 
                name: str, 
                num_envs = 1,
                device = "cuda", 
                robot_offset: np.array = np.array([0.0, 0.0, 0.0]),
                replicate_physics: bool = True,
                offset=None) -> None:

        # cloning stuff
        self.num_envs = num_envs
        self._env_ns = "/World/envs"
        self._env_spacing = 0.5 # [m]
        self._template_env_ns = self._env_ns + "/env_0"

        self._cloner = GridCloner(spacing=self._env_spacing)
        self._cloner.define_base_env(self._env_ns)

        prim_utils.define_prim(self._template_env_ns)
        self._envs_prim_paths = self._cloner.generate_paths(self._env_ns + "/env", 
                                                self.num_envs)

        # task-specific parameters
        if len(robot_offset) != 3:
            robot_offset = np.array([0.0, 0.0, 0.0])
            print("KyonRlSteppingTask:  the provided robot_offset is not of the correct shape. A null offset will be used instead.")

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

        import subprocess
        try:

            xacro_gen = subprocess.check_call(["xacro",\
                                            xacro_path, \
                                            add_wheels, \
                                            add_arms, \
                                            remove_floating_joint, \
                                            remove_sensors, \
                                            "-o", 
                                            self._urdf_path])

        except:

            raise Exception('\nFAILED TO GENERATE KYON\'S URDF!!!.\n')

    def _import_urdf(self, 
                    import_config: omni.isaac.urdf._urdf.ImportConfig = _urdf.ImportConfig(), 
                    robot_prim_name: str = "Kyon"):

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

        return success

    def world_was_initialized(self):

        self._world_initialized = True

    def set_robot_default_jnt_config(self, 
                                hip_roll = torch.tensor([0.3, -0.3, 0.3, -0.3]), 
                                hip_pitch = torch.tensor([-0.3, -0.3, 0.3, 0.3]), 
                                knee_pitch = torch.tensor([0.3, 0.3, -0.3, -0.3]), 
                                wheels = torch.tensor([0.0, 0.0, 0.0, 0.0])):
        
        if (self._world_initialized):

            self._default_jnt_positions = torch.zeros((self.num_envs, self._robot_n_dofs))
            
            default_jnt_positions = torch.cat((hip_roll, 
                                                hip_pitch, 
                                                knee_pitch, 
                                                wheels),
                                                0)

            for i in range(0,  self.num_envs):

                self._default_jnt_positions[i, :] = default_jnt_positions

            self._default_jnt_positions = torch.cat((hip_roll, 
                                                hip_pitch, 
                                                knee_pitch, 
                                                wheels),
                                                0)

            self._robots_art_view.set_joints_default_state(positions= self._default_jnt_positions)
            
        else:

            raise Exception("Before calling _set_robot_default_jnt_config(), you need to reset the World at least once and call _world_was_initialized()")

    def set_robot_root_default_config(self):
        
        # To be implemented

        return True

    def set_robot_imp_gains(self, 
                            hip_roll_kp = 100.0, 
                            hip_pitch_kp = 100.0, 
                            knee_pitch_kp = 100.0,
                            wheels_kp = 0.0, 
                            hip_roll_kd = 10.0, 
                            hip_pitch_kd = 10.0, 
                            knee_pitch_kd = 10.0, 
                            wheels_kd = 10.0):

        hip_roll_kps = torch.tensor([hip_roll_kp, hip_roll_kp, hip_roll_kp, hip_roll_kp])
        hip_roll_kds = torch.tensor([hip_roll_kd, hip_roll_kd, hip_roll_kd, hip_roll_kd])
        hip_pitch_kps = torch.tensor([hip_pitch_kp, hip_pitch_kp, hip_pitch_kp, hip_pitch_kp])
        hip_pitch_kds = torch.tensor([hip_pitch_kd, hip_pitch_kd, hip_pitch_kd, hip_pitch_kd])
        knee_pitch_kps = torch.tensor([knee_pitch_kp, knee_pitch_kp, knee_pitch_kp, knee_pitch_kp])
        knee_pitch_kds = torch.tensor([knee_pitch_kd, knee_pitch_kd, knee_pitch_kd, knee_pitch_kd])
        wheels_kps = torch.tensor([wheels_kp, wheels_kp, wheels_kp, wheels_kp])
        wheels_kds = torch.tensor([wheels_kd, wheels_kd, wheels_kd, wheels_kd])

        joint_kps = torch.cat((hip_roll_kps, 
                                hip_pitch_kps, 
                                knee_pitch_kps, 
                                wheels_kps),
                                0)
        joint_kds = torch.cat((hip_roll_kds, 
                                hip_pitch_kds, 
                                knee_pitch_kds, 
                                wheels_kds),
                                0)

        self.joint_kps_envs = torch.zeros((self.num_envs, self._robot_n_dofs))
        self.joint_kds_envs = torch.zeros((self.num_envs, self._robot_n_dofs)) 

        for i in range(0, self.num_envs):

            self.joint_kps_envs[i, :] = joint_kps
            self.joint_kds_envs[i, :] = joint_kds

        self._robots_art_view.set_gains(kps= self.joint_kps_envs, 
                                        kds= self.joint_kds_envs, 
                                        # indices= 
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

            raise Exception("Before calling _print_envs_info(), you need to reset the World at least once!")

    def fill_robot_info_from_world(self):

        if (self._world_initialized):

            self._robot_bodynames = self._robots_art_view.body_names
            self._robot_n_links = self._robots_art_view.num_bodies
            self._robot_n_dofs = self._robots_art_view.num_dof
            self._robot_dof_names = self._robots_art_view.dof_names
        
        else:

            raise Exception("Before calling _get_robot_info_from_world(), you need to reset the World at least once!")

    def set_up_scene(self, 
                    scene) -> None:
        
        self._generate_urdf()

        self._import_urdf()
        
        pos_offsets = np.zeros((self.num_envs, 3))
        for i in range(0, self.num_envs):
            pos_offsets[i, :] = self._robot_offset
        
        envs_positions = self._cloner.clone(
            source_prim_path=self._template_env_ns,
            prim_paths=self._envs_prim_paths,
            replicate_physics=self._replicate_physics,
            position_offsets = pos_offsets
        ) # robot is now at the default env prim --> we can clone the environment

        self._robots_art_view = ArticulationView(self._env_ns + "/env*"+ "/" + self._robot_prim_name, 
                                reset_xform_properties=False)

        self._robots_articulations = scene.add(self._robots_art_view)

        scene.add_default_ground_plane(z_position=0, 
                            name="ground_plane", 
                            prim_path= self._ground_plane_prim_path, 
                            static_friction=0.5, 
                            dynamic_friction=0.5, 
                            restitution=0.8)

        # set default camera viewport position and target
        self.set_initial_camera_params()

    def set_initial_camera_params(self, camera_position=[10, 10, 3], camera_target=[0, 0, 0]):
        set_camera_view(eye=camera_position, target=camera_target, camera_prim_path="/OmniverseKit_Persp")

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
        # reset_env_ids = self.resets.nonzero(as_tuple=False).squeeze(-1)
        # if len(reset_env_ids) > 0:
        #     self.reset(reset_env_ids)

        # actions = torch.tensor(actions)

        # forces = torch.zeros((self._cartpoles.count, self._cartpoles.num_dof), dtype=torch.float32, device=self._device)
        # forces[:, self._cart_dof_idx] = self._max_push_effort * actions[0]

        # indices = torch.arange(self._cartpoles.count, dtype=torch.int32, device=self._device)
        # self._cartpoles.set_joint_efforts(forces, indices=indices)

        a = 1

    def get_observations(self):

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
