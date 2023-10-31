# KyonRLStepping package

The preferred way of using KyonRLStepping package is to employ the provided [mamba](https://mamba.readthedocs.io/en/latest/user_guide/mamba.html) environment. 

Installation instructions:
- First install Mamba by running ```curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh"``` and then ```bash Mambaforge-$(uname)-$(uname -m).sh```.

- Create the mamba environment by running ```create_mamba_env.sh```. This will properly setup a Python 3.7 mamba environment named ```kyonrlstepping``` with (almost) all necessary dependencies

- Activate the environment with ```mamba activate kyonrlstepping```

- From the root folder install the package with ```pip install -e .```.

- Debug the controllers with ```python scripts/run_control_cluster.py```.

- To be able to use the controllers, you need to install also the remaining external dependencies (horizon, phase_manager).

External dependencies to be installed separately: 
- [horizon-casadi](https://github.com/ADVRHumanoids/horizon), T.O. tool tailored to robotics, based on [Casadi](https://web.casadi.org/). Branch to be used: ```add_nodes_rl```. Clone this repo at your preferred location and, from its root, run ```pip install --no-deps -e .```. 
- [phase_manager](https://github.com/AndrePatri/phase_manager).
- [CoClusterBridge](https://github.com/AndPatr/CoClusterBridge), branch ```francesco_debug```: utilities to create a CPU-based controllers cluster to be interfaced with GPU-based simulators 