#!/bin/bash
set -e

# Sorgente ambiente ROS 2
source /opt/ros/humble/setup.bash

# Esegui il comando passato al container (es. bash, ros2 run, ecc.)
exec "$@"