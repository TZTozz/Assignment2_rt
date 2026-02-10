#!/bin/bash

PERCORSO="/home/tozz/Documents/ros_ws"
COMANDO="source install/local_setup.sh"

gnome-terminal --working-directory="$PERCORSO" -- bash -c "$COMANDO; exec bash"
