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


# Set this workspace as the default catkin project
SEARCH="export DEFAULT_CATKIN_WS=.*"
REPLACE="export DEFAULT_CATKIN_WS=\"$ROOT_DIR\""
FILE="$HOME/.rosconfig"
search_replace_line_in_file "$SEARCH" "$REPLACE" "$FILE"
