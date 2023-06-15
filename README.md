# KyonRLStepping package

The preferred way of using KyonRLStepping package is to employ the provided [mamba](https://mamba.readthedocs.io/en/latest/user_guide/mamba.html) environment. 

Installation instructions:
- First install Mamba by running ```curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh"``` and then ```bash Mambaforge-$(uname)-$(uname -m).sh```.

- Create the mamba environment by running ```create_mamba_env.sh```. This will properly setup a Python 3.7 mamba environment named ```kyonrlstepping``` with (almost) all necessary dependencies

- Activate the environment with ```mamba activate kyonrlstepping```

- From the root folder install the package with ```pip install -e .```.

- Test the Lunar Lander example from StableBaselines3 v2.0 with ```python kyonrlstepping/tests/test_lunar_lander_stable_bs3.py```.

- Download [Omniverse Launcer](https://www.nvidia.com/en-us/omniverse/download/), go to the "exchange" tab and install ``` Omniverse Cache``` and  ```Isaac Sim 2022.2.1```  (might take a while). You can then launch it from the Launcher GUI or by navigating to ```${HOME}/.local/share/ov/pkg/isaac_sim-2022.2.1``` and running the ```isaac-sim.sh``` script. When launching IsaacSim for the first time, compilation of ray tracing shaders will take place and may take a while. If the resources of the workstation/pc are limited (e.g. RAM < 16GB), the compilation may abort after a while. You can still manage to compile them by adding sufficient SWAP memory to the system. Before trying to recompile the shaders, remember however to first delete the cache at ```.cache/ov/Kit/*```.

- To be able to run any script with dependencies on Omniverse packages, it's necessary to first source ```${HOME}/.local/share/ov/pkg/isaac_sim-*/setup_conda_env.sh```.

- You can now test a simple simulation with Kyon by running ```python kyonrlstepping/tests/spawn_kyon_isaac_sim.py``` or ```python kyonrlstepping/tests/test_kyon_cloning.py```.

- To be able to use the controllers, you need to install also the remaining external dependencies (horizon, phase_manager).

External dependencies to be installed separately: 
- [horizon-casadi](https://github.com/ADVRHumanoids/horizon), T.O. tool tailored to robotics, based on [Casadi](https://web.casadi.org/). Branch to be used: ```add_nodes_py37```. Clone this repo at your preferred location and, from its root, run ```pip install --no-deps -e .```. This will install the package in editable mode without its dependencies (this is necessary to avoid circumvent current issues with horizon's pip distribution).
<!-- - [casadi_kin_dyn](https://github.com/ADVRHumanoids/horizon), generation of symbolic expressions for robot kinematics and dynamics, based on [http://wiki.ros.org/urdf](URDF) and [https://github.com/stack-of-tasks/pinocchio](Pinocchio). This library is automatically installed through mamba config file. -->
- [phase_manager](https://github.com/FrancescoRuscelli/phase_manager/tree/master). Currently stable branch: ```add_nodes```. Build this CMake package in you workspace (after activating the ```kyonrlstepping``` environment) and set the ```CMAKE_INSTALL_PREFIX``` to ```${HOME}/mambaforge/envs/kyonrlstepping```. 
<!-- - [Cartesian Interface](https://github.com/ADVRHumanoids/CartesianInterface/tree/2.0-devel) -->
- [Omniverse Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html), photo-realistic GPU accelerated simulatorfrom NVIDIA.

### Short-term ToDo list:

- [x] Test Stable Baselines3 with Gymnasium to understand exactly the interfaces needed to set up a custom environment and use it

- [x] Decide exactly for simulation environment to use. Options:
    - :x: IsaacGym Preview4 &rarr; this is a standalone distribution. It won't be maintained nor developed by NVIDIA. All development will focus on Omniverse ecosystem.
    - :heavy_check_mark: Omniverse isaac_sim-2022.2.1 &rarr; this is maintained and actively developed by NVIDIA team. Modular structure and different API from IsaacGym. More complicated but definitely more complete software environment.
    - :x: No external simulator or Gazebo/Mujoco/Pybullet(CPU) &rarr; might be doable in an initial stage, but requires to setup a custom rendering for visualizing and debugging training, doesn't have realistic dynamics simulation (we should use Horizon's integration) and requires a CPU-based dynamics integration for each environment.

- [x] Decide which gym library to use (defines the environment the learning algorithm runs on). Viable options might be:

    - :x: OpenAI’s [Gym](https://github.com/openai/gym) library (no longer maintained). Currently used by IsaacGym and also Omniverse's ```VecEnvBase```.

    - :heavy_check_mark: [Gymnasium](https://gymnasium.farama.org/), a maintained fork of OpenAI’s [Gym ](https://github.com/openai/gym) library (no longer maintained). Most notably, the library already provides environments for OpenAI's [MuJoCo](https://gymnasium.farama.org/environments/mujoco/) simulator. The user can implement their custom environments if needed. This library has been improvements wrt the old Gym and migration from Gym should be trivial.
    

- [x] Decide which library/algorithms to use for learning. Options:
    - :heavy_check_mark: [Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/), version 2.0.* with support for [Gymnasium](https://gymnasium.farama.org/) and [PyTorch >=1.11](https://pytorch.org/). An advantage of Baselines3 being that it provides a collection of many SoA and learning algorithms (useful for debugging) and is reliable. This should be the starting point.
    - :wavy_dash: [SKRL](https://skrl.readthedocs.io/en/latest/), an open-source modular library for Reinforcement Learning written in PyTorch. Out-of-the-box support of environment interfaces provided by [Isaac Orbit](https://isaac-orbit.github.io/), [Omniverse Isaac Gym](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs),  [IsaacGym](https://developer.nvidia.com/isaac-gym), OpenAI [Gymnasium](https://gymnasium.farama.org/) and [Gym](https://github.com/openai/gym). This might be considered in the future.

    - :x: [rl_games](https://github.com/Denys88/rl_games). Used by both [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs) and [IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs) benchmark repos, with support for PyTorch (1.9+) and CUDA 11.1+. Algorithms supported with PyTorch are PPO and asymmetric actor-critic variant of PPO. Apparently, this library is tailored to GPU and shows considerably higher performance than, e.g. stable-baselines3. 

- [x] Code implementation options:
    - :x: directly develop on top of [Isaac Orbit](https://isaac-orbit.github.io/), a framework for creating robot learning environments, based on [Omniverse Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html). It allows to create custom wrappers for interfacing with learning libraries, but it already provides interfaces for [Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/), [SKRL](https://skrl.readthedocs.io/en/latest/), [rl_games](https://github.com/Denys88/rl_games), [rsl_rl](https://github.com/leggedrobotics/rsl_rl) (see [here](https://isaac-orbit.github.io/orbit/source/api/orbit_envs.utils.wrappers.html)). [Here](https://github.com/NVIDIA-Omniverse/Orbit/blob/main/source/tools/convert_urdf.py) is also shows an example of converting a URDF to USD (used by Isaac Sim). Orbit's github [repo](https://github.com/NVIDIA-Omniverse/Orbit/tree/main) hosts a custom ```IsaacEnv``` which inherits from ```gym.Env```, same as ``` omni.isaac.gym.VecEnvBase``` and implementations of actuators models, controller (e.g. joint impedance), static and dynamic markers, sensors and a base class for robots ```RobotBase```, which wraps ```omni.isaac.core.articulations.Articulations``` class
    - :heavy_check_mark: develop an as-much-as-possible self-contained package with simple environment, tasks, etc.. exploiting and possibly modifying currently available code in Omniverse, [Isaac Orbit](https://isaac-orbit.github.io/) framework and [Omniverse Isaac Gym](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs) &rarr; this is the preferred option as of now. Code reimplementation should however be kept as low as possible.

- [x] First simple test of IsaacSim simulators: What was done:
    - [x] Kyon ```prim``` is spawned in the scene at a user defined ```prim_path``` calling the ``URDFParseAndImportFile`` script.
    - [x] A simple joint-space impedance controller is attached to the robot. This allows to send position, velocity, effort commands.
    - [x] Checked simulation stability and collision behavior.

- [x] Integration and testing of casadi_kin_dyn and horizon in Conda 

- [] Fist proof-of-concept integration of Horizon-based [kyon mpc controller](https://github.com/ADVRHumanoids/kyon_controller) within the simulation:

    - [] Horizon's abstract class for RHC controller
    - [] Kyon's specific implementation of RHC controller
    - [] Testing of joystick-based MPC control in IsaacSim
    - [] Implementation of a joint space impedance controller for simultaneous application of control actions through all environments?

- [] Setting up the RL task:
    - [x] Testing vectorization of Kyon's simulation
    - [x] Test first skeleton of Kyon's Gymnasium environment and task (without rewards, observations...)
    - [] Implementing observations, rewards, envs reset, etc..