# Ubuntu Jammy Jellyfish as base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

# Update the system
RUN apt-get update && apt-get upgrade -y && \
    # The `install ros-humble-desktop` command below requests a timezone in an interactive way, 
    # which halts the creation of the docker image. By installing `tzdata` before that, this is averted.
    apt-get install -y tzdata

# Optionally set time zone
ENV TZ="Europe/Athens"
RUN date

# The rest of these lines follow (mostly) the documentation for the installation of ROS2 Humble Hawksbill:
# see: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

# Setup locale
RUN locale  # check for UTF-8 && \
    apt-get update && apt-get install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    echo "LANG=en_US.UTF-8" >> /root/.bashrc && \
    locale  # verify settings

# Setup sources
RUN export LANG=en_US.UTF-8 && \
    apt-get install software-properties-common -y && \
    add-apt-repository universe && \
    apt-get update && apt-get install curl -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install /tmp/ros2-apt-source.deb -y && \
    apt-get update -y && \
    apt-get install -y /tmp/ros2-apt-source.deb

# Install ROS2 packages
RUN export LANG=en_US.UTF-8 && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install ros-humble-desktop ros-dev-tools -y && \
    # Source the ROS2 underlay
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    source /root/.bashrc && \
    # Create the ROS2 workspace path
    mkdir -p /ros2_ws/src

# Install Eigen, YAML-cpp and RVIZ visual tools dependencies
RUN export LANG=en_US.UTF-8 && apt-get install libeigen3-dev libyaml-cpp* ros-humble-rviz-visual-tools -y && \
    # Install ROS2 Joint State publisher, its GUI and XACRO
    apt-get update && apt-get install ros-humble-joint-state-publisher* -y && apt-get install -y ros-humble-xacro
