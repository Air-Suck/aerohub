# Fixed PX4 interfaces; independent of the AS2 source submodules.
ARG BASE_IMAGE=osrf/ros@sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db
ARG UBUNTU_APT_MIRROR=https://archive.ubuntu.com/ubuntu
ARG UBUNTU_SECURITY_MIRROR=https://security.ubuntu.com/ubuntu
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
RUN find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) -print0 \
    | xargs -0 sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_APT_MIRROR}|g" \
      -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_SECURITY_MIRROR}|g" \
      -e "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g"
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/px4_ws
RUN git init src/px4_msgs \
    && git -C src/px4_msgs fetch --depth 1 https://github.com/PX4/px4_msgs.git "$PX4_MSGS_COMMIT" \
    && git -C src/px4_msgs checkout --detach FETCH_HEAD \
    && test "$(git -C src/px4_msgs rev-parse HEAD)" = "$PX4_MSGS_COMMIT"
COPY deployment/onboard_computer/px4_msgs/overrides/ /tmp/px4-1.18-msgs/
RUN cd /tmp/px4-1.18-msgs && sha256sum -c SHA256SUMS \
    && cp *.msg /opt/px4_ws/src/px4_msgs/msg/
RUN source /opt/ros/humble/setup.bash \
    && MAKEFLAGS=-j2 colcon build --merge-install --packages-select px4_msgs \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && source install/setup.bash \
    && python3 -c 'from px4_msgs.msg import VehicleStatus; assert VehicleStatus.MESSAGE_VERSION == 4' \
    && printf '%s\n' "$PX4_MSGS_COMMIT" > install/px4_msgs-source-commit \
    && rm -rf build log src /tmp/px4-1.18-msgs
