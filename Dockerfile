# ros2 humble + webots_ros2 for macOS Apple Silicon
# webots runs natively on mac, ros2 runs in this container
# communication via TCP + shared folder

FROM ros:humble

# avoid interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# install webots_ros2 and navigation stack
RUN apt-get update && apt-get install -y \
    # webots ros2 interface
    ros-humble-webots-ros2 \
    # navigation stack (includes DWA/DWB controller)
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # slam
    ros-humble-slam-toolbox \
    # turtlebot3 packages
    ros-humble-turtlebot3* \
    # teleop keyboard control
    ros-humble-teleop-twist-keyboard \
    # tf2 tools for debugging transforms
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    # visualization (for foxglove bridge)
    ros-humble-foxglove-bridge \
    # build tools for custom packages
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    # utilities
    vim \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# set turtlebot3 model
ENV TURTLEBOT3_MODEL=burger

# source ros2 automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# workspace for custom packages (mount point)
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# default command
CMD ["bash"]
