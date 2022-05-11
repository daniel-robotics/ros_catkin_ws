#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_NAME="$(basename $SCRIPT_DIR)"

# Download additional github projects into source directory
#   cd "$SCRIPT_DIR/src"
#   if [ -d "simontest" ]; then
#       cd "simontest" && git pull origin master
#   else
#       git clone git@github.com:SimonBirrell/simontest.git
#   fi

# Install ROS dependencies for each package
rosdep update
if [ -z ${ROS_OS_OVERRIDE+x} ]; then
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO --os=$ROS_OS_OVERRIDE -y
else
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO -y
fi

# Initialize this folder as a catkin_tools workspace and build it
cd "$SCRIPT_DIR"
catkin init
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
catkin build

# Set this as the default catkin project and source it
APPEND="source \"$SCRIPT_DIR/devel/setup.bash\""
FILE="$HOME/.bashrc"
grep -qxF "$APPEND" "$FILE" || echo "$APPEND" | tee -a "$FILE" > /dev/null

source ~/.bashrc
