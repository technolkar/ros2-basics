# Start from the official ROS 2 Jazzy image
FROM osrf/ros:jazzy-desktop

# Work inside /ros2_ws in the container
WORKDIR /ros2_ws

# Copy only the source code (build/install/log stay out)
COPY src ./src

# Build the workspace inside the image
RUN . /opt/ros/jazzy/setup.sh && colcon build

# Every interactive shell in the container sources the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default: open a shell
CMD ["bash"]