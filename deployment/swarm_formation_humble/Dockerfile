FROM osrf/ros:humble-desktop-full-jammy
SHELL ["/bin/bash", "-c"]
COPY src/planner/swarm_formation/ros2_adapter/native /opt/swarm_ws/src
COPY src/planner/swarm_formation/upstream/LICENSE /opt/swarm_ws/LICENSE
RUN source /opt/ros/humble/setup.bash && cd /opt/swarm_ws && \
    MAKEFLAGS=-j2 colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash", "-lc", "source /opt/swarm_ws/install/setup.bash; exec ros2 launch ego_planner seven_planners.launch.py input_mode:=navigation fleet_config:=/config/fleet.yaml"]
