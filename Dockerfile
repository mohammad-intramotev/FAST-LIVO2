FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    DISABLE_ROS1_EOL_WARNINGS=true

# Setup workspace
WORKDIR /root/catkin_ws/src

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