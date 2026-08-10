# fs_autonomy runtime image.
# ros:humble-ros-base is multi-arch: builds x86_64 on the dev VM and arm64 on
# a Jetson (JetPack 6 / Ubuntu 22.04) from this same file.
#
#   docker build -t fs_autonomy -f docker/Dockerfile .
#   docker run --rm -it --net=host --ipc=host fs_autonomy \
#       ros2 launch fs_autonomy accel.launch.py

FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# eufs_msgs is a pure message package -- the only thing shared with the sim.
RUN git clone --depth 1 https://gitlab.com/eufs/eufs_msgs.git src/eufs_msgs

COPY src/fs_autonomy src/fs_autonomy

RUN . /opt/ros/humble/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build

# source the overlay instead of only the base install
RUN sed -i 's|source "/opt/ros/$ROS_DISTRO/setup.bash"|source "/ws/install/setup.bash"|' /ros_entrypoint.sh
