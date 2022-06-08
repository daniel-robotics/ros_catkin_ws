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


# Source workspace then build all packages in the src directory
cd "$ROOT_DIR"
catkin build

echo
echo "Catkin build complete."
echo "Please run: source \"$ROOT_DIR/devel/setup.bash\""
echo
