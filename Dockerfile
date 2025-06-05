FROM ros:noetic-ros-core

# Set to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Setup workspace
WORKDIR /root/catkin_ws/src

# Fix temp key issue
RUN apt-get update || true && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && rm -f /etc/apt/sources.list.d/ros1-latest.list \
             /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
    && echo \
       "deb [signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg] \
       http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
       > /etc/apt/sources.list.d/ros-latest.list

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
      ros-noetic-tf \
      ros-noetic-pcl-ros \
      ros-noetic-eigen-conversions \
      ros-noetic-cv-bridge \
      ros-noetic-image-transport \
      ros-noetic-rviz \
      ros-noetic-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Copy FAST-LIVO2 into src/
COPY . FAST-LIVO2/

# Clone dependencies
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git
RUN git clone https://github.com/mohammad-intramotev/Sophus.git

# Build Sophus
WORKDIR /root/catkin_ws/src/Sophus
RUN mkdir build && cd build && cmake .. && make && make install

# Build the entire catkin workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source ROS and the workspace setup on container start
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default command
CMD ["tail", "-f", "/dev/null"]