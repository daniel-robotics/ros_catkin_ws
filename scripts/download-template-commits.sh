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


if [[ "$WORKSPACE_NAME" == "ros_catkin_ws" ]]; then
    echo "ERROR: Cannot add upstream daniel-robotics/ros_catkin_ws because this repository has the same name!"
    exit
fi

cd "$ROOT_DIR"
git remote add upstream https://github.com/daniel-robotics/ros_catkin_ws.git
git pull upstream main --allow-unrelated-histories

echo
echo "If merge conflicts occurred, you may resolve file-by-file using:"
echo "    git mergetool"
echo "    git checkout --ours filename.c"
echo " OR git checkout --theirs filename.c"
echo "    git add filename.c"
echo "    git commit -m \"updated from ros_catkin_ws template\""
echo
