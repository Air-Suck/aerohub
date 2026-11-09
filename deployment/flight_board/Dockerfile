# syntax=docker/dockerfile:1
# Source recipe: documents additions to px4-sitl-gazebo.
# Runtime delivery uses the frozen tar archive; see README.md and image.sh.
# Rebuild-only inputs: provide vendor/*.tar.gz matching the hashes below.
ARG PX4_IMAGE=px4io/px4-sitl-gazebo@sha256:54a7a15ba35b3f4786256296d89edd144e000f40084cb6ed692fcbf74c9168f2
FROM ${PX4_IMAGE} AS agent-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
# Fast DDS 2.12 relies on transitive cstdint includes removed by GCC 13.
ENV CXXFLAGS="-include cstdint"
RUN rm -f /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       build-essential cmake patch libasio-dev libtinyxml2-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
COPY vendor/ /opt/vendor/
COPY deployment/flight_board/patches/microxrce-fastdds-v2.12.2.patch /tmp/
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
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce \
    && cmake --build Micro-CDR/build --parallel 4 \
    && cmake --install Micro-CDR/build \
    && ldconfig \
    && rm -rf Micro-CDR \
    && mkdir Micro-XRCE-DDS-Agent \
    && tar -xzf /opt/vendor/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz \
       --strip-components=1 -C Micro-XRCE-DDS-Agent \
    && patch -p1 -d Micro-XRCE-DDS-Agent \
       < /tmp/microxrce-fastdds-v2.12.2.patch \
    && cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce -DCMAKE_PREFIX_PATH=/opt/xrce -DUAGENT_LOGGER_PROFILE=OFF \
    && cmake --build Micro-XRCE-DDS-Agent/build --parallel 4 \
    && cmake --install Micro-XRCE-DDS-Agent/build \
    && ldconfig \
    && rm -rf Micro-XRCE-DDS-Agent /opt/vendor /tmp/microxrce-fastdds-v2.12.2.patch


# Preserve the existing Noble PX4 binary and Agent with their own dynamic loader.
FROM ${PX4_IMAGE} AS payload
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
RUN rm -f /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends patchelf \
    && rm -rf /var/lib/apt/lists/*
COPY --from=agent-builder /opt/xrce/ /opt/xrce/
COPY deployment/flight_board/collect_runtime.py /tmp/collect_runtime.py
RUN python3 /tmp/collect_runtime.py

# ROS publishers use the same Humble baseline as the OHOS AS2 subscriber.
FROM osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0 AS runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-msgpack ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
ENV ROS_DISTRO=humble LANG=C.UTF-8
COPY --from=payload /opt/px4-gazebo/ /opt/px4-gazebo/
COPY --from=payload /opt/xrce/ /opt/xrce/
COPY --from=payload /opt/px4-runtime/ /opt/px4-runtime/
WORKDIR /opt/platform_sensors
COPY simulator/airsim/sensors/ src/as2_sensor_adapter/
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --packages-select uav_as2_sensor_adapter \
    && PYTHONPATH=src/as2_sensor_adapter python3 -m unittest discover -s src/as2_sensor_adapter/tests -v \
    && PYTHONPATH=src/as2_sensor_adapter:$PYTHONPATH python3 -c 'import uav_as2_sensor_adapter.clock; import uav_as2_sensor_adapter.node'
COPY deployment/flight_board/entrypoint.sh /usr/local/bin/flight-board
COPY deployment/flight_board/supervisor.py /opt/flight_board/supervisor.py
COPY deployment/airsim/config/airsim/settings/px4-sensors.json /opt/flight_board/settings.example.json
RUN chmod +x /usr/local/bin/flight-board
ENV RUNS_IN_DOCKER=true PX4_SIM_MODEL=none_iris PX4_SYS_AUTOSTART=10016 \
    ROS_DOMAIN_ID=81 VEHICLE_NAME=PX4 VEHICLE_NAMESPACE=drone0 \
    PX4_INSTANCE=0 PX4_SYSTEM_ID=1 XRCE_AGENT_PORT=8888 \
    AIRSIM_HOST=127.0.0.1 AIRSIM_RPC_PORT=41471 \
    AIRSIM_SETTINGS=/config/settings.json
WORKDIR /var/lib/flight_board
ENTRYPOINT ["/usr/local/bin/flight-board"]
CMD ["vehicle"]
COPY deployment/airsim/config/dds/fastdds_udp.xml /opt/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_udp.xml
HEALTHCHECK --interval=5s --timeout=3s --start-period=15s --retries=3 CMD ["python3", "/opt/flight_board/supervisor.py", "healthcheck"]
