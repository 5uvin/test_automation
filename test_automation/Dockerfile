FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# Install deps
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
# Set workspace
WORKDIR /ros2_ws
COPY ./test_automation_pkg ./src/test_automation_pkg
COPY ./ros2_bt_utils ./src/ros2_bt_utils
COPY ./Groot ./src/Groot
# Build your code
RUN . /opt/ros/humble/setup.bash && \
    apt update && apt upgrade -y && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]