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

echo
echo "Installing dependencies for Catkin workspace: $WORKSPACE_NAME."
echo

# Import package dependencies config file
if [ -f "$CONF_DIR/depends.conf" ]; then
    source "$CONF_DIR/depends.conf"
    echo -e "Will download the following APT packages: $APT_PACKAGES"
    echo -e "Will download the following ROS packages to ./src: $ROS_PACKAGES"
    echo -e "Will download the following Git repositories to ./src: $GIT_REPOSITORIES"
    echo
    APT_PACKAGES="$(newlines_to_spaces "$APT_PACKAGES")" 
    ROS_PACKAGES="$(newlines_to_spaces "$ROS_PACKAGES")"
    split(){ GIT_REPOSITORIES=( $GIT_REPOSITORIES ); }
    IFS=$'\n' split
else
    echo "WARNING: $CONF_DIR/depends.conf does not exist."
    echo
    APT_PACKAGES=""
    ROS_PACKAGES=""
    GIT_REPOSITORIES=""
fi

echo
read -r -p "Continue to download? (Y/N): " 
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit
fi

cd "$ROOT_DIR"
mkdir -p "./src"

# Download APT packages
cd "$ROOT_DIR"
sudo apt-get update
sudo apt-get install $APT_PACKAGES

# Download ROS packages (and dependencies) into source directory
cd "$ROOT_DIR"
"$SCRIPT_DIR/ros-noetic-scripts/rosinstall-generator.sh" "$ROS_PACKAGES"
rm "./ros-noetic.rosinstall"

# Download Git repos into source directory
cd "$SRC_DIR"
for pkg in "${GIT_REPOSITORIES[@]}"; do
    git_reset_pull $pkg
done

echo
read -r -p "Continue to rosdep? (Y/N): " 
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit
fi

# Install missing system dependencies for each package in src
cd "$ROOT_DIR"
"$SCRIPT_DIR/ros-noetic-scripts/rosdep-update.sh"
"$SCRIPT_DIR/ros-noetic-scripts/rosdep-install.sh"

# Execute install.sh in each package (if it exists)
cd "$SRC_DIR"
packages=(*)
for pkg in "${packages[@]}"; do 
    cd "$SRC_DIR/$pkg"
    if [ -f "./install.sh" ]; then
        echo "Executing $pkg/install.sh..."
        chmod +x ./install.sh
        ./install.sh
    else
        echo "No install script for $pkg"
    fi
done


echo
echo "Complete."
echo