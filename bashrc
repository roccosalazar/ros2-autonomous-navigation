# ROS 2 Humble setup
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic
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
