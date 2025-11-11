source /root/ibrido_utils/mamba_utils/bin/_activate_current_env.sh
micromamba activate  ibrido

source /opt/ros/noetic/setup.bash
source /opt/xbot/setup.sh
source /root/ibrido_ws/setup.bash

python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path /tmp/RtDeploymentEnv/kyon_real_wheels_no_yaw.urdf \
    --simopt_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/sim_opt.xml \
    --world_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/world.xml \
    --sites_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/sites.xml \
    --xbot_config_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/xbot2_basic_wheels_no_yaw_real.yaml \
     --pub_rostime --blink_name pelvis

# python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/centauro.urdf \
#     --simopt_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sim_opt.xml \
#     --world_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/world.xml \
#     --sites_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sites.xml \
#     --xbot_config_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/xbot2_basic.yaml \
#     --pub_rostime --blink_name base_link
# python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path --simopt_path --world_path --sites_path --xbot_config_path --pub_rostime --blink_name --fullspeed --rt_factor 1.0 --render_to_file
