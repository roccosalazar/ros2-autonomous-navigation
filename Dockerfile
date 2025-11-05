FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*


# Crea utente non-root "ros"
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}

# Copia la configurazione della shell e l'entrypoint
COPY .bashrc /home/${USERNAME}/.bashrc
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Imposta utente e cartella di lavoro
USER ${USERNAME}
WORKDIR /workspace

# Abilita GUI/X11
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
