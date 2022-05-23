#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_NAME="$(basename "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

if [ "$EUID" -eq 0 ]; then
    echo "DO NOT run this script with sudo!"
    echo "Catkin workspaces should never be run as root."
    exit
fi

# Download additional github projects into source directory
cd "$SCRIPT_DIR/src"
if [ -d "template_rospy_pkg" ]; then
    cd "template_rospy_pkg" && git pull origin main
else
    git clone https://github.com/danielk-98/template_rospy_pkg.git
fi
cd "$SCRIPT_DIR/src"
if [ -d "template_msg_pkg" ]; then
    cd "template_msg_pkg" && git pull origin main
else
    git clone https://github.com/danielk-98/template_msg_pkg.git
fi

# Install ROS dependencies for each package
cd "$SCRIPT_DIR"
rosdep update
if [ -z ${ROS_OS_OVERRIDE+x} ]; then
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO --os=$ROS_OS_OVERRIDE -y
else
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO -y
fi

# Execute install.sh in each package (if it exists)
cd "$SCRIPT_DIR/src"
packages=(*)
for dir in "${packages[@]}"; do 
    cd "$SCRIPT_DIR/src/$dir"
    if [ -f "./install.sh" ]; then
        echo "Executing $dir/install.sh..."
        chmod +x ./install.sh
        ./install.sh
    else
        echo "No install script for $dir"
    fi
done

# Initialize this folder as a catkin_tools workspace and build it
cd "$SCRIPT_DIR"
catkin init
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug

# Build the workspace
cd "$SCRIPT_DIR"
chmod +x ./build.sh
./build.sh

# Set this as the default catkin project and source it
SEARCH="export DEFAULT_CATKIN_WS=.*"
REPLACE="export DEFAULT_CATKIN_WS=\"$SCRIPT_DIR\""
FILE="$HOME/.bashrc"
sed -i "s#$SEARCH#$REPLACE#" "$FILE"

APPEND="source \"\$DEFAULT_CATKIN_WS/devel/setup.bash\""
FILE="$HOME/.bashrc"
grep -qxF "$APPEND" "$FILE" || echo "$APPEND" | tee -a "$FILE" > /dev/null

source ~/.bashrc
