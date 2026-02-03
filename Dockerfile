FROM osrf/ros:humble-desktop

ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo locales ca-certificates \
  && locale-gen en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    python3-pip \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Add OSRF Gazebo repository for Gazebo Classic (11)
RUN curl -fsSL http://packages.osrfoundation.org/gazebo.key | apt-key add - \
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  && rm -rf /var/lib/apt/lists/*

# Install SLAM and Nav2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user to use inside the container
RUN groupadd --gid ${USER_GID} ${USERNAME} || true && \
    useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} || true && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Create workspace directory
RUN mkdir -p /home/${USERNAME}/workspace && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/workspace

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

ENV HOME /home/${USERNAME}
WORKDIR /home/${USERNAME}

COPY ./entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
