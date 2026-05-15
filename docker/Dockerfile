FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy \
    WS=/opt/ros_ws \
    PARAMS_FILE=/config/params.yaml \
    RVIZ=false

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        build-essential \
        git \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${WS}/src
COPY . ${WS}/src/elevation_traversability/

RUN rosdep update --rosdistro=${ROS_DISTRO} && \
    apt-get update && \
    rosdep install --from-paths ${WS}/src --ignore-src -r -y \
        --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select elevation_traversability

COPY docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD []
