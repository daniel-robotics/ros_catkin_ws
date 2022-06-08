#!/usr/bin/env bash
if [ "$EUID" -eq 0 ]; then
    echo "ERROR: root privilege detected."
    echo "Catkin workspaces should never be modified as root."
    echo "Please rerun script without sudo."
    exit 1
fi

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
CONF_DIR="$ROOT_DIR/config"
SRC_DIR="$ROOT_DIR/src"
WORKSPACE_NAME="$(basename "$ROOT_DIR")"
source "$SCRIPT_DIR/script-utils"


# Mark all scripts in this folder as executable
cd "$SCRIPT_DIR"
chmod +x *.sh


# Download ROS utility scripts
cd "$SCRIPT_DIR"
git_reset_pull https://github.com/daniel-scripts/ros-noetic-scripts.git main
cd "./ros-noetic-scripts"
chmod +x *.sh


# Check if ROS is installed correctly
if [ ! -f "$HOME/.rosconfig" ]; then
    echo "ERROR: ~/.rosconfig not found."
    echo "Please reinstall ROS with 'scripts/ros-noetic-scripts/install.sh'."
    exit 1
fi
if [ -z "$ROS_INSTALL_PATH" ]; then
    echo "ERROR: ROS_INSTALL_PATH not set."
    echo "This could be due to an invalid ~/.rosconfig, or if ~/.rosconfig is not being automatically sourced."
    echo "Please reinstall ROS with 'scripts/ros-noetic-scripts/install.sh'."
    exit 1
fi


echo
echo "Initializing Catkin workspace: $WORKSPACE_NAME"
echo


# Import Catkin configuration file
if [ -f "$CONF_DIR/catkin.conf" ]; then
    source "$CONF_DIR/catkin.conf"
    echo -e "Using Catkin configuration options:\n$CATKIN_CONFIG_ARGS"
    echo
    CATKIN_CONFIG_ARGS="$(newlines_to_spaces "$CATKIN_CONFIG_ARGS")"
else
    echo "WARNING: $CONF_DIR/catkin.conf does not exist."
    echo
    CATKIN_CONFIG_ARGS=""
fi


# Initialize this folder as a catkin_tools workspace
cd "$ROOT_DIR"
mkdir -p "./src"
catkin init
if [[ "$CATKIN_CONFIG_ARGS" != "" ]]; then
    catkin config $CATKIN_CONFIG_ARGS 
fi


# Set as default Catkin workspace
echo
echo "The default Catkin workspace is sourced automatically by each shell."
echo "If not default, you will need to run 'source \"$ROOT_DIR/devel/setup.bash\"' before each 'catkin build'."
echo
echo "The default Catkin workspace is defined in ~/.rosconfig."
echo "Check which workspace is default using 'echo \$DEFAULT_CATKIN_WS'"
echo
read -r -p "Set $WORKSPACE_NAME as default? (Y/N): " 
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    "$SCRIPT_DIR/set-default-workspace.sh"
fi

echo
echo "Initialization complete."
echo