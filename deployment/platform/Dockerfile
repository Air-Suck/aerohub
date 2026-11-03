FROM voxelsky/ros-humble-desktop-classic:v0.0.2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG AEROSTACK2_COMMIT=19c397309c6a41b279e6b551818ec12605cfcecc
ARG AS2_PLATFORM_COMMIT=fcba062219935acef69dfa009c8be376c707ee8a
ARG PX4_MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db
ARG MICRO_XRCE_AGENT_TAG=v2.4.2

ENV ROS_DISTRO=humble \
    AEROSTACK2_WORKSPACE=/root/aerostack2_ws \
    AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential cmake git patch libasio-dev libtinyxml2-dev \
       python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/aerostack2_ws/src
WORKDIR /root/aerostack2_ws/src
COPY aerostack2/ ./aerostack2/
COPY px4_msgs/ ./px4_msgs/
COPY as2_platform_pixhawk/ ./as2_platform_pixhawk/
COPY deployment/platform/patches/as2-platform-px4-v1.17.patch /tmp/
RUN test "$(cat aerostack2/.source-commit)" = "${AEROSTACK2_COMMIT}" \
    && test "$(cat px4_msgs/.source-commit)" = "${PX4_MSGS_COMMIT}" \
    && test "$(cat as2_platform_pixhawk/.source-commit)" = "${AS2_PLATFORM_COMMIT}" \
    && patch --batch --forward -p1 -d as2_platform_pixhawk \
         < /tmp/as2-platform-px4-v1.17.patch

WORKDIR /root/aerostack2_ws
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
       --packages-up-to \
         as2_platform_pixhawk \
         as2_state_estimator \
         as2_motion_controller \
         as2_behaviors_motion \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /opt
# v2.4.2 points at the now-deleted Fast-DDS branch 2.12.x. Pin the
# corresponding immutable upstream release tag instead.
COPY vendor/Fast-DDS-v2.12.2.tar.gz /opt/vendor/Fast-DDS-v2.12.2.tar.gz
COPY vendor/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz /opt/vendor/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz
COPY vendor/Fast-CDR-v1.1.1.tar.gz /opt/vendor/Fast-CDR-v1.1.1.tar.gz
COPY vendor/Micro-CDR-v2.0.1.tar.gz /opt/vendor/Micro-CDR-v2.0.1.tar.gz
COPY vendor/Micro-XRCE-DDS-Client-v2.4.2.tar.gz /opt/vendor/Micro-XRCE-DDS-Client-v2.4.2.tar.gz
COPY vendor/foonathan-memory-v0.7-3.tar.gz /opt/vendor/foonathan-memory-v0.7-3.tar.gz
COPY vendor/tinyxml2-8c8293ba.tar.gz /opt/vendor/tinyxml2-8c8293ba.tar.gz
COPY deployment/platform/patches/microxrce-fastdds-v2.12.2.patch /tmp/
RUN echo 'b31ea2d39e9af1a7e28af5809d48cfcc8d634bc1081e76fdee67f2f166ff9d7f  /opt/vendor/Fast-DDS-v2.12.2.tar.gz' \
      | sha256sum -c - \
    && echo '586b7a021cbec2b5a93fafcdab8a6f3a520952c2c8f5253d217a1e49bfed497a  /opt/vendor/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz' \
      | sha256sum -c - \
    && echo '5b5c1fc138a0d3092010158be0306c587da56bf406b0bfd45c20835e53839caf  /opt/vendor/Fast-CDR-v1.1.1.tar.gz' \
      | sha256sum -c - \
    && echo 'bcc16517acc55a58fad6d39cfa42ba3658e2c7d0f408af5b1a27b634072d64b6  /opt/vendor/Micro-CDR-v2.0.1.tar.gz' \
      | sha256sum -c - \
    && echo '4a2d4016ebfb3895c3e217fadbd3a3a03f0259c5280a83b1e85e993323eddb67  /opt/vendor/Micro-XRCE-DDS-Client-v2.4.2.tar.gz' \
      | sha256sum -c - \
    && echo '4203d15db22a94a3978eeb1afb59a37d35c57c0f148733f0f1a53a6281cb74dd  /opt/vendor/foonathan-memory-v0.7-3.tar.gz' \
      | sha256sum -c - \
    && echo '17995f6508c54c296f07c8fa661cc3f25927a2b0855cf9b0d50b8c6993a63acb  /opt/vendor/tinyxml2-8c8293ba.tar.gz' \
      | sha256sum -c - \
    && mkdir Micro-CDR \
    && tar -xzf /opt/vendor/Micro-CDR-v2.0.1.tar.gz \
       --strip-components=1 -C Micro-CDR \
    && cmake -S Micro-CDR -B Micro-CDR/build \
       -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Micro-CDR/build --parallel \
    && cmake --install Micro-CDR/build \
    && ldconfig \
    && rm -rf Micro-CDR \
    && mkdir Micro-XRCE-DDS-Agent \
    && tar -xzf /opt/vendor/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz \
       --strip-components=1 -C Micro-XRCE-DDS-Agent \
    && patch -p1 -d Micro-XRCE-DDS-Agent \
       < /tmp/microxrce-fastdds-v2.12.2.patch \
    && cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build \
       -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Micro-XRCE-DDS-Agent/build --parallel \
    && cmake --install Micro-XRCE-DDS-Agent/build \
    && ldconfig \
    && rm -rf Micro-XRCE-DDS-Agent /opt/vendor /tmp/microxrce-fastdds-v2.12.2.patch

COPY deployment/platform/config.yaml /opt/aerostack2-integration/config.yaml
COPY deployment/platform/multi/ /opt/aerostack2-integration/platform-multi/
COPY deployment/platform/pid_speed_controller.yaml /opt/aerostack2-integration/pid_speed_controller.yaml
COPY deployment/platform/run_single_as2.sh /usr/local/bin/run_single_as2.sh
COPY deployment/platform/run_multi_as2.sh /usr/local/bin/run_multi_as2.sh
RUN chmod +x /usr/local/bin/run_single_as2.sh /usr/local/bin/run_multi_as2.sh

ENV ROS_DOMAIN_ID=0 \
    AEROSTACK2_SIMULATION_DRONE_ID=drone0

CMD ["bash", "/usr/local/bin/run_single_as2.sh"]
