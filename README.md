# KyonRLStepping package

Create the conda environment with ```conda env create -f kyon_stepping_conda_env.yml```

Activate the environment with ```conda activate kyonrlstepping```

Test the Lunar Lander example from StableBaselines3 v2.0 with ```python kyonrlstepping/tests/test_lunar_lander_stable_bs3.py```

External dependencies (to be pruned/refined/changed in the future): 
- [horizon](https://github.com/ADVRHumanoids/horizon), T.O. tool tailored to robotics, based on [Casadi](https://web.casadi.org/)
- [casadi_kin_dyn](https://github.com/ADVRHumanoids/horizon), generation of symbolic expressions for robot kinematics and dynamics, based on [http://wiki.ros.org/urdf](URDF) and [https://github.com/stack-of-tasks/pinocchio](Pinocchio)
- [IsaacGym Preview4](https://developer.nvidia.com/isaac-gym) ([Omniverse Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html)), GPU accelerated simulator for RL applications from NVIDIA

### Short-term ToDo list:

- [] Test Stable Baselines3 with Gymnasium to understand exactly the interfaces needed to set up a custom environment and use it

- [] Decide exactly for simulation environment to use. Options:
    - IsaacGym Preview4 &rarr; this is a standalone distribution. Will not be maintained nor developed by NVIDIA. All development will focus on Omniverse ecosystem.
    - Omniverse isaac_sim-2022.2.1 &rarr; this is maintained and actively developed by NVIDIA team. Modular structure and different API from IsaacGym. More complicated but definitely more complete software environment.
    - No external simulator or Gazebo/Mujoco/Pybullet(CPU) &rarr; not very advantageous

- [] Decide which gym library to use (defines the environment the learning algorithm runs on). Viable options might be:
    - [Isaac Orbit](https://isaac-orbit.github.io/), a framework (in Preview) for creating robot learning environments, based on [Omniverse Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html). It allows to create custom wrappers for interfacing with learning libraries, but it already provides interfaces for [Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/), [SKRL](https://skrl.readthedocs.io/en/latest/), [rl_games](https://github.com/Denys88/rl_games), [rsl_rl](https://github.com/leggedrobotics/rsl_rl) (see [here](https://isaac-orbit.github.io/orbit/source/api/orbit_envs.utils.wrappers.html)). [Here](https://github.com/NVIDIA-Omniverse/Orbit/blob/main/source/tools/convert_urdf.py) is also shows an example of converting a URDF to USD (used by Isaac Sim). Using IsaacSim's URDF extension might also be an option.

    - [Gymnasium](https://gymnasium.farama.org/), a maintained fork of OpenAI’s [Gym ](https://github.com/openai/gym) library (no longer maintained). Most notably, the library already provides environments for OpenAI's [MuJoCo](https://gymnasium.farama.org/environments/mujoco/) simulator. The user can implement their custom environments if needed.
    

- [] Decide which library/algorithms to use for learning. Options may include:
    - [Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/), version 2.0.* with support for [Gymnasium](https://gymnasium.farama.org/) and [PyTorch 2.0](https://pytorch.org/). An advantage of Baselines3 being that it provides a collection of many SoA and learning algorithms and is reliable.
    - [SKRL](https://skrl.readthedocs.io/en/latest/), an open-source modular library for Reinforcement Learning written in PyTorch. Out-of-the-box support of environment interfaces provided by [Isaac Orbit](https://isaac-orbit.github.io/), [Omniverse Isaac Gym](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs),  [IsaacGym](https://developer.nvidia.com/isaac-gym), OpenAI [Gymnasium](https://gymnasium.farama.org/) and [Gym](https://github.com/openai/gym)

    - [rl_games](https://github.com/Denys88/rl_games). Used by both [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs) and [IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs) benchmark repos, with support for PyTorch (1.9+) and CUDA 11.1+. Algorithms supported with PyTorch are PPO and asymmetric actor-critic variant of PPO.
    
    