# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

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
