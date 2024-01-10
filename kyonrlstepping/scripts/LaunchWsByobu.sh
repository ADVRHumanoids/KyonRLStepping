#!/bin/bash

# Set Italian keyboard layout
export XMODIFIERS=@im=ibus
export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus

SLEEP_FOR=0.5
WS_NAME="RlWorkspace"
WORKING_DIR="$HOME/RL_ws/hhcm/src/KyonRLStepping/kyonrlstepping/scripts"
MAMBAENVNAME="KyonRLSteppingIsaac2023.1.0"

# Function to execute common commands in Terminator terminal
execute_command() {
    xdotool type --delay 1 "$1"
    xdotool key Return
    sleep $SLEEP_FOR
    xdotool key Ctrl+L
}

source_mamba_env() {

    execute_command "mamba activate ${MAMBAENVNAME}"

}

split_h() {

    xdotool key Ctrl+F2

}

split_v() {

    xdotool key Ctrl+Shift+e

}

new_tab() {

    xdotool key Ctrl+Shift+T

}

# launch terminator window
byobu kill-session -t ${WS_NAME}
byobu new-session -s ${WS_NAME} 
execute_command "cd ${WORKING_DIR}"
split_h

# tab 0
# source_mamba_env

# split_v

# source_mamba_env

# split_h

# source_mamba_env

# split_h

# source_mamba_env

# xdotool key Alt+Left

# split_h

# execute_command "source /opt/ros/noetic/setup.bash"
# execute_command "source ~/RL_ws/hhcm/setup.bash"
# source_mamba_env

# # tab 1

# new_tab
# execute_command "source /opt/ros/noetic/setup.bash"
# execute_command "roscore"

# split_h

# execute_command "source /opt/ros/noetic/setup.bash"
# execute_command "source ~/RL_ws/hhcm/setup.bash"

# # tab 2

# new_tab

# execute_command "htop"

# split_h

# execute_command "nvtop"
# xdotool key Return

