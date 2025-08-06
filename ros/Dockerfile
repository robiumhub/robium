FROM ubuntu:22.04

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop

# Install development tools
RUN apt-get install -y \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    python3-rosdep

# Clean up
RUN apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create a non-root user
RUN useradd -ms /bin/bash ros
USER ros
WORKDIR /home/ros

# Copy the workspace
COPY . /home/ros/ros_ws/src

# Install dependencies
RUN rosdep install -i --from-path src --rosdistro humble -y

# Build the workspace
RUN . /opt/ros/humble/setup.bash && colcon build

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros/ros_ws/install/setup.bash" >> ~/.bashrc 