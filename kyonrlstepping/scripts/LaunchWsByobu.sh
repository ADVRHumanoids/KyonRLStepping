#!/bin/bash

# Set Italian keyboard layout
export XMODIFIERS=@im=ibus
export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus

SLEEP_FOR=0.1
WS_NAME="RlWorkspace"
WORKING_DIR="$HOME/RL_ws/hhcm/src/KyonRLStepping/kyonrlstepping/scripts"
MAMBAENVNAME="KyonRLSteppingIsaac2023.1.0"

# Array of directories
directories=(
    "$HOME/RL_ws/hhcm/src/KyonRLStepping"
    "$HOME/RL_ws/hhcm/src/CoClusterBridge"
    "$HOME/RL_ws/hhcm/src/OmniRoboGym"
    "$HOME/RL_ws/hhcm/src/horizon"
    "$HOME/RL_ws/hhcm/src/RHCViz"
    "$HOME/RL_ws/hhcm/build/horizon"
    "$HOME/RL_ws/hhcm/src/phase_manager"
    "$HOME/RL_ws/hhcm/build/phase_manager"
    "$HOME/RL_ws/hhcm/src/SharsorIPCpp"
    "$HOME/RL_ws/hhcm/build/SharsorIPCpp"
    # Add more directories as needed
)

press_enter() {

    byobu send-keys Enter
    sleep $SLEEP_FOR
}

# Function to execute common commands in Terminator terminal
execute_command() {
    byobu send-keys "$1"
    press_enter
    sleep $SLEEP_FOR
}

prepare_command() {
    byobu send-keys "$1"
    sleep $SLEEP_FOR
}

go_to_pane() {

    byobu select-pane -t $1

}

go_to_window() {

    byobu select-window -t $1

}

attach_to_session() {

    byobu attach-session -t ${WS_NAME} 

}

source_mamba_env() {

    execute_command "mamba activate ${MAMBAENVNAME}"

}

split_h() {

    byobu split-window -p 50 -v

}

split_v() {

    byobu split-window -p 50 -h

}

new_tab() {

    byobu new-window

}

# Function to navigate to a directory and split Terminator horizontally
cd_and_split() {

    execute_command "cd $1"
    
    # Check if it's the last directory before splitting
    if [ "$1" != "${directories[-1]}" ]; then
    
        split_h

    fi
}

# launch terminator window
byobu kill-session -t ${WS_NAME}

byobu new-session -d -s ${WS_NAME} -c ${WORKING_DIR} -n ${WS_NAME} # -d "detached" session


# tab 0
execute_command "cd ${WORKING_DIR}"
source_mamba_env
execute_command "source ~/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/setup_conda_env.sh"
prepare_command "reset && python KyonPlay.py"

split_v
execute_command "cd ${WORKING_DIR}"
source_mamba_env
prepare_command "reset && python RunControlCluster.py"

split_h
execute_command "cd ${WORKING_DIR}"
source_mamba_env
prepare_command "reset && python RunDebugGUI.py"

split_h
execute_command "cd ${WORKING_DIR}"
source_mamba_env
prepare_command "reset && python RunKeyboardCmds.py"

go_to_pane 0 

split_h
execute_command "cd ${WORKING_DIR}"
execute_command "source /opt/ros/noetic/setup.bash"
execute_command "source ~/RL_ws/hhcm/setup.bash"
source_mamba_env
prepare_command "reset && python RunRhc2RosBridge.py kyon0"

# tab 1
new_tab

execute_command "source /opt/ros/noetic/setup.bash"
prepare_command "roscore"

split_h
execute_command "cd ${WORKING_DIR}"
execute_command "source /opt/ros/noetic/setup.bash"
execute_command "source ~/RL_ws/hhcm/setup.bash"
prepare_command "reset && python3 RunRhc2Viz.py --nodes_perc 10 --robot_type kyon --robot_name kyon0"

# tab2
new_tab
execute_command "htop"

split_h
execute_command "cd ${WORKING_DIR}"
execute_command "nvtop"
press_enter

# tab 3

new_tab

# Loop through directories and navigate to each one
for dir in "${directories[@]}"; do
    cd_and_split "$dir"
done

# we attach to the detached session
attach_to_session