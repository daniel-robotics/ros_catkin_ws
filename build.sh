#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_NAME="$(basename "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

if [ "$EUID" -eq 0 ]; then
    echo "DO NOT run this script with sudo!"
    echo "Catkin workspaces should never be run as root."
    exit
fi


catkin build
source ~/.bashrc
