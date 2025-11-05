FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    psmisc \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \ 
    python3-rosdep \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-generate-parameter-library-py \
    ros-${ROS_DISTRO}-pal-urdf-utils \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-launch-param-builder \ 
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    && rm -rf /var/lib/apt/lists/*

    

# Copia i file header di lightsfm in /usr/local/include/lightsfm
COPY deps_ws/src/lightsfm/include /usr/local/include/lightsfm

# Crea utente non-root "ros"
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}

# Copia la configurazione della shell e l'entrypoint
COPY bashrc /home/${USERNAME}/.bashrc
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Imposta utente e cartella di lavoro
USER ${USERNAME}
WORKDIR /workspace

# Abilita GUI/X11
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
