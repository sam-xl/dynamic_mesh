FROM ros:jazzy

SHELL ["bash", "-c"]

ARG DEBIAN_FRONTEND noninteractive

ARG WORKSPACE_DIR=/workspace
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/dynamic_mesh \
    apt update && rosdep install \
        --from-paths ${WORKSPACE_DIR}/src \
        -iry \
    && rm -rf /var/lib/apt/lists/* \
    && cd ${WORKSPACE_DIR} \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --packages-up-to dynamic_mesh --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log
