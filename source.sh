#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_NAME="$(basename "$SCRIPT_DIR")"

cd "$SCRIPT_DIR"
if [ -f "./devel/setup.bash" ]; then
    source "$SCRIPT_DIR/devel/setup.bash"
else
    echo "Error: Catkin workspace $SCRIPT_DIR not initialized"
    echo "Please run $SCRIPT_DIR/init.sh"
fi
