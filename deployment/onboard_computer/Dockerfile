# Fixed PX4 interfaces are built separately from the editable AS2 submodules.
ARG BASE_IMAGE=onboard_computer_dependencies:humble-px4-1.18
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG IMAGE_VERSION=v0.1.0
ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.version=$IMAGE_VERSION \
      org.opencontainers.image.revision=$SOURCE_REVISION \
      org.opencontainers.image.source="https://github.com/Air-Suck/aerohub"
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
COPY as2/airsim/aerostack2/ aerostack2/
COPY as2/as2_platform_pixhawk/ as2_platform_pixhawk/
COPY deployment/platform/patches/as2-platform-px4-v1.18.patch /tmp/as2-platform-px4-v1.18.patch
RUN cd as2_platform_pixhawk && patch --batch --forward -p1 < /tmp/as2-platform-px4-v1.18.patch
COPY src/perception/navigation_inputs/ as2_navigation_inputs/
WORKDIR /root/aerostack2_ws
# Install only dependencies of the selected flight components, not unrelated
# Gazebo/platform packages that happen to be present in the upstream monorepo.
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && colcon list --packages-up-to as2_platform_pixhawk as2_state_estimator \
       as2_motion_controller as2_behaviors_motion as2_navigation_inputs --paths-only > /tmp/as2-paths \
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
       as2_motion_controller as2_behaviors_motion as2_navigation_inputs \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && source install/setup.bash \
    && ros2 pkg prefix as2_platform_pixhawk \
    && ros2 pkg prefix as2_state_estimator \
    && ros2 pkg prefix as2_motion_controller \
    && ros2 pkg prefix as2_behaviors_motion \
    && ros2 pkg prefix as2_navigation_inputs
# Project execution overlay; the flight-board image and PX4 dependencies are unchanged.
COPY src/planner/interfaces/ /opt/uav_execution_ws/src/interfaces/
COPY src/behavior/follow_path/ /opt/uav_execution_ws/src/follow_path/
WORKDIR /opt/uav_execution_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/px4_ws/install/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --parallel-workers 2 \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY deployment/onboard_computer/entrypoint.sh /usr/local/bin/onboard-computer
COPY deployment/airsim/as2/run.py /opt/onboard_computer/run.py
COPY deployment/airsim/as2/selected_behaviors.launch.py /opt/onboard_computer/selected_behaviors.launch.py
COPY deployment/airsim/as2/scoped_tf.launch.py /opt/onboard_computer/scoped_tf.launch.py
COPY deployment/airsim/config/as2/config.yaml /opt/onboard_computer/config.yaml
COPY deployment/airsim/config/as2/fleet.yaml /opt/onboard_computer/fleet.yaml
COPY deployment/airsim/config/as2/pid_speed_controller.yaml /opt/onboard_computer/pid_speed_controller.yaml
RUN chmod +x /usr/local/bin/onboard-computer
ENTRYPOINT ["/usr/local/bin/onboard-computer"]
CMD ["run"]

COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_udp.xml
