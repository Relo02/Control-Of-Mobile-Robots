FROM osrf/ros:humble-desktop-full

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set timezone to UTC
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    apt-get install -y tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    vim \
    nano \
    htop && \
    apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# Update rosdep (don't re-init)
RUN rosdep update || true

# Setup ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
