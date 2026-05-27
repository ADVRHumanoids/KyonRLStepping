usage() {
  echo "Usage: $0 [--rt_factor <value>] [--ros-version <ros2|ros1>]"
  exit 1
}

RT_FACTOR=1.0
XMJ_ROS_VERSION="${XMJ_ROS_VERSION:-ros2}"
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-jazzy}"
URDF_PATH="/tmp/RtDeploymentEnv/kyon_real_wheels_no_yaw.urdf"

require_valid_urdf() {
  local urdf_path="$1"

  if [ ! -s "$urdf_path" ]; then
    echo "URDF not found or empty: $urdf_path"
    echo "Start the RtDeploymentEnv world interface first so it can generate the RT deployment URDF."
    exit 2
  fi

  if ! grep -Eq '<robot([[:space:]>])' "$urdf_path"; then
    echo "URDF does not contain a <robot> root: $urdf_path"
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rt_factor) RT_FACTOR="$2"; shift ;;
    --ros-version|--ros_version) XMJ_ROS_VERSION="$2"; shift ;;
    *) echo "Unknown arg: $1"; usage ;;
  esac
  shift
done

source /root/ibrido_utils/mamba_utils/bin/_activate_current_env.sh
micromamba activate  ibrido

case "$XMJ_ROS_VERSION" in
  1) XMJ_ROS_VERSION="ros1"; source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
  2) XMJ_ROS_VERSION="ros2"; source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
  ros1) source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
  ros2) source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
  *) echo "Unsupported ROS version: ${XMJ_ROS_VERSION}"; usage ;;
esac
source /opt/xbot/setup.sh
source /root/ibrido_ws/setup.bash

require_valid_urdf "$URDF_PATH"

python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path "$URDF_PATH" \
    --simopt_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/sim_opt_wheels.xml \
    --world_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/world.xml \
    --sites_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/sites.xml \
    --xbot_config_path /root/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real/xbot2_basic_wheels_no_yaw_real.yaml \
     --pub_rostime --blink_name pelvis \
    --ros-version "$XMJ_ROS_VERSION" \
    --rt_factor "$RT_FACTOR"

# python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/centauro.urdf \
#     --simopt_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sim_opt.xml \
#     --world_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/world.xml \
#     --sites_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/sites.xml \
#     --xbot_config_path /root/ibrido_ws/src/xbot2_mujoco/tests/files/centauro/xbot2_basic.yaml \
#     --pub_rostime --blink_name base_link
# python /root/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py --urdf_path --simopt_path --world_path --sites_path --xbot_config_path --pub_rostime --blink_name --fullspeed --rt_factor 1.0 --render_to_file
