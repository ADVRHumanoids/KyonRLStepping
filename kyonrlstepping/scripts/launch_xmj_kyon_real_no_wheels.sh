#!/bin/bash

usage() {
  echo "Usage: $0 [--rt_factor <value>] [--ros-version <ros2|ros1>] [--urdf_path <path>] [--srdf_path <path>] [--runtime_dir <path>] [--headless] [--no_manual_stepping] [--pub-rostime]"
  exit 1
}

RT_FACTOR=1.0
XMJ_ROS_VERSION="${XMJ_ROS_VERSION:-ros2}"
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-jazzy}"
HEADLESS=false
NO_MANUAL_STEPPING=false
PUB_ROSTIME=false

VARIANT_NAME="kyon_real_no_wheels"
RUNTIME_DIR="${XMJ_RUNTIME_DIR:-/tmp/KyonRLStepping/${VARIANT_NAME}}"
URDF_PATH="${XMJ_URDF_PATH:-${RUNTIME_DIR}/${VARIANT_NAME}.urdf}"
SRDF_PATH="${XMJ_SRDF_PATH:-${RUNTIME_DIR}/${VARIANT_NAME}.srdf}"
XBOT_CONFIG_PATH="${XMJ_XBOT_CONFIG_PATH:-${RUNTIME_DIR}/xbot2_basic.yaml}"

KYON_DESCRIPTION_ROOT="${KYON_DESCRIPTION_ROOT:-${HOME}/ibrido_ws/src/iit-kyon-description}"
KYON_URDF_XACRO="${KYON_URDF_XACRO:-${KYON_DESCRIPTION_ROOT}/kyon_urdf/urdf/kyon.urdf.xacro}"
KYON_SRDF_XACRO="${KYON_SRDF_XACRO:-${KYON_DESCRIPTION_ROOT}/kyon_srdf/srdf/kyon.srdf.xacro}"
KYON_XMJ_DIR="${KYON_XMJ_DIR:-${HOME}/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/xmj_env_files/kyon_real}"
XBOT_CONFIG_SRC="${KYON_XBOT_CONFIG_SRC:-${KYON_XMJ_DIR}/xbot2_basic_real.yaml}"
SIMOPT_PATH="${KYON_SIMOPT_PATH:-${KYON_XMJ_DIR}/sim_opt.xml}"
XBOT_CONFIG_BUILDER="${IBRIDO_XBOT_CONFIG_BUILDER:-${HOME}/ibrido_utils/ibrido_xbot_config_builder.py}"
KYON_JNT_IMP_CONFIG_PATH="${KYON_JNT_IMP_CONFIG_PATH:-${HOME}/ibrido_ws/src/KyonRLStepping/kyonrlstepping/config/jnt_imp_config_kyon_real.yaml}"

require_valid_xml() {
  local path="$1"
  local root_tag="$2"

  if [ ! -s "$path" ]; then
    echo "XML file not found or empty: $path"
    exit 2
  fi

  if ! grep -Eq "<${root_tag}([[:space:]>])" "$path"; then
    echo "XML file does not contain a <${root_tag}> root: $path"
    exit 2
  fi
}

generate_xrdf() {
  local xacro_path="$1"
  local output_path="$2"

  if ! command -v xacro >/dev/null 2>&1; then
    echo "Cannot generate XRDF: xacro is not available on PATH."
    exit 2
  fi

  if [ ! -f "$xacro_path" ]; then
    echo "Cannot generate XRDF: xacro file not found: $xacro_path"
    exit 2
  fi

  mkdir -p "$(dirname "$output_path")"
  xacro "$xacro_path" \
    root:="${KYON_DESCRIPTION_ROOT}/kyon_urdf" \
    wheels:=false \
    upper_body:=false \
    dagana:=false \
    cameras:=false \
    velodyne:=false \
    payload:=false \
    floating_joint:=true \
    use_abs_mesh_paths:=true \
    use_local_filesys_for_meshes:=false \
    -o "$output_path"
}

apply_runtime_impedance_config() {
  if [ ! -f "$XBOT_CONFIG_BUILDER" ]; then
    echo "XBot config builder not found: $XBOT_CONFIG_BUILDER"
    exit 2
  fi
  if [ ! -f "$KYON_JNT_IMP_CONFIG_PATH" ]; then
    echo "Joint impedance config not found: $KYON_JNT_IMP_CONFIG_PATH"
    exit 2
  fi

  XBOT_CONFIG_PATH="$(
    python3 "$XBOT_CONFIG_BUILDER" \
      --xbot-config "$XBOT_CONFIG_PATH" \
      --impedance-config "$KYON_JNT_IMP_CONFIG_PATH" \
      --urdf-path "$URDF_PATH" \
      --srdf-path "$SRDF_PATH" \
      --output-dir "${RUNTIME_DIR}/xbot_runtime"
  )"
}

prepare_runtime_files() {
  mkdir -p "$RUNTIME_DIR"

  echo "Generating Kyon URDF at $URDF_PATH"
  generate_xrdf "$KYON_URDF_XACRO" "$URDF_PATH"
  require_valid_xml "$URDF_PATH" robot

  echo "Generating Kyon SRDF at $SRDF_PATH"
  generate_xrdf "$KYON_SRDF_XACRO" "$SRDF_PATH"
  require_valid_xml "$SRDF_PATH" robot

  cp "$XBOT_CONFIG_SRC" "$XBOT_CONFIG_PATH"
  rm -rf "$RUNTIME_DIR/hal"
  cp -r "$KYON_XMJ_DIR/hal" "$RUNTIME_DIR/hal"
  apply_runtime_impedance_config
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rt_factor) RT_FACTOR="$2"; shift ;;
    --ros-version|--ros_version) XMJ_ROS_VERSION="$2"; shift ;;
    --urdf_path|--urdf-path) URDF_PATH="$2"; shift ;;
    --srdf_path|--srdf-path) SRDF_PATH="$2"; shift ;;
    --runtime_dir|--runtime-dir) RUNTIME_DIR="$2"; shift ;;
    --headless) HEADLESS=true ;;
    --no_manual_stepping|--no-manual-stepping) NO_MANUAL_STEPPING=true ;;
    --pub-rostime|--pub_rostime) PUB_ROSTIME=true ;;
    *) echo "Unknown arg: $1"; usage ;;
  esac
  shift
done

source "${HOME}/ibrido_utils/mamba_utils/bin/_activate_current_env.sh"
micromamba activate ibrido
if [ -f /opt/xbot/setup.sh ]; then
  source /opt/xbot/setup.sh
fi

extra_args=()
if [ "$HEADLESS" = true ]; then
  extra_args+=(--headless)
fi
if [ "$NO_MANUAL_STEPPING" = true ]; then
  extra_args+=(--no_manual_stepping)
fi
if [ "$PUB_ROSTIME" = true ]; then
  case "$XMJ_ROS_VERSION" in
    1) XMJ_ROS_VERSION="ros1"; source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
    2) XMJ_ROS_VERSION="ros2"; source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
    ros1) source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
    ros2) source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
    *) echo "Unsupported ROS version: ${XMJ_ROS_VERSION}"; usage ;;
  esac
  extra_args+=(--pub_rostime --ros-version "$XMJ_ROS_VERSION")
fi
source "${HOME}/ibrido_ws/setup.bash"

prepare_runtime_files

python "${HOME}/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py" --urdf_path "$URDF_PATH" \
    --simopt_path "$SIMOPT_PATH" \
    --world_path "${KYON_XMJ_DIR}/world.xml" \
    --sites_path "${KYON_XMJ_DIR}/sites.xml" \
    --xbot_config_path "$XBOT_CONFIG_PATH" \
    --blink_name pelvis \
    --rt_factor "$RT_FACTOR" \
    "${extra_args[@]}"
