#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_NAME="$(basename "$SCRIPT_DIR")"

source "$SCRIPT_DIR/devel/setup.bash"
