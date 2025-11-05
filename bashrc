# ROS 2 Humble setup
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /usr/share/gazebo/setup.sh

# Colcon autocomplete
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# (Opzionale) workspace locale
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# Alias utili
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
