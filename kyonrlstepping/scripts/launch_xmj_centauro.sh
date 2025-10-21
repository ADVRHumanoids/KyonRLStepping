source /root/ibrido_utils/mamba_utils/bin/_activate_current_env.sh
micromamba activate  ibrido

source /opt/ros/noetic/setup.bash
source /opt/xbot/setup.sh
source /root/ibrido_ws/setup.bash

python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/centauro.urdf \
    --simopt_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sim_opt.xml \
    --world_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/world.xml \
    --sites_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sites.xml \
    --xbot_config_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/xbot2_basic.yaml \
    --pub_rostime --blink_name base_link
