# Fixed PX4 interfaces are built separately from the editable AS2 submodules.
ARG BASE_IMAGE=companion_as2_dependencies:humble-px4-1.18
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble \
    AEROSTACK2_WORKSPACE=/root/aerostack2_ws \
    AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2 \
    ROS_DOMAIN_ID=81 \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git patch python3-colcon-common-extensions \
    python3-rosdep python3-vcstool python3-yaml \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /root/aerostack2_ws/src
COPY thirdparty/aerostack2/ aerostack2/
COPY thirdparty/as2_platform_pixhawk/ as2_platform_pixhawk/
WORKDIR /root/aerostack2_ws
# Install only dependencies of the selected flight components, not unrelated
# Gazebo/platform packages that happen to be present in the upstream monorepo.
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && colcon list --packages-up-to as2_platform_pixhawk as2_state_estimator \
       as2_motion_controller as2_behaviors_motion --paths-only > /tmp/as2-paths \
    && apt-get update \
    && (test -e /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) \
    && rosdep update --rosdistro humble
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && rosdep install --from-paths $(cat /tmp/as2-paths) --ignore-src \
       --rosdistro humble -t build -t build_export -t buildtool -t buildtool_export -t exec -y \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && MAKEFLAGS=-j2 colcon build --parallel-workers 2 --merge-install \
       --packages-up-to as2_platform_pixhawk as2_state_estimator \
       as2_motion_controller as2_behaviors_motion \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && source install/setup.bash \
    && ros2 pkg prefix as2_platform_pixhawk \
    && ros2 pkg prefix as2_state_estimator \
    && ros2 pkg prefix as2_motion_controller \
    && ros2 pkg prefix as2_behaviors_motion
COPY deployment/companion_as2/entrypoint.sh /usr/local/bin/companion-as2
COPY runtime/as2/run.py /opt/companion_as2/run.py
COPY config/as2/config.yaml /opt/companion_as2/config.yaml
COPY config/as2/fleet.yaml /opt/companion_as2/fleet.yaml
COPY config/as2/pid_speed_controller.yaml /opt/companion_as2/pid_speed_controller.yaml
RUN chmod +x /usr/local/bin/companion-as2
ENTRYPOINT ["/usr/local/bin/companion-as2"]
CMD ["run"]

COPY config/dds/fastdds_udp.xml /opt/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_udp.xml
