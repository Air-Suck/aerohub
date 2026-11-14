FROM uav/aerostack2-px4:humble-v1.17

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /opt/uav_planner_ws
COPY src/tool/navigation/interfaces/ src/tool/navigation/interfaces/
COPY src/behavior/follow_path/ src/behavior/follow_path/
COPY src/tool/navigation/framework/ src/tool/navigation/framework/
COPY src/tool/navigation/planner/cat_ora/ros2_adapter/bridge/ src/tool/navigation/planner/cat_ora/ros2_adapter/bridge/
COPY src/tool/navigation/planner/swarm_formation/ros2_adapter/bridge/ src/tool/navigation/planner/swarm_formation/ros2_adapter/bridge/
COPY src/behavior/legacy_setpoint/ src/behavior/legacy_setpoint/
COPY src/tool/navigation/planner/super/ros2_adapter/native/mars_uav_sim/mars_quadrotor_msgs/ src/dependencies/mars_quadrotor_msgs/
COPY deployment/platform/fleet_takeoff.py /usr/local/bin/fleet_takeoff.py
COPY deployment/platform/run_single_as2.sh /usr/local/bin/run_single_as2.sh
COPY deployment/platform/run_multi_as2.sh /usr/local/bin/run_multi_as2.sh
COPY deployment/platform/pid_speed_controller.yaml /opt/aerostack2-integration/pid_speed_controller.yaml

RUN chmod +x /usr/local/bin/fleet_takeoff.py

RUN source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash \
    && colcon test --merge-install --packages-select uav_follow_path_execution \
    && colcon test-result --verbose \
    && python3 -m unittest discover -s src/tool/navigation/framework/tests -v \
    && python3 -m unittest discover -s src/tool/navigation/planner/cat_ora/ros2_adapter/bridge/tests -v \
    && python3 -m unittest discover -s src/tool/navigation/planner/swarm_formation/ros2_adapter/bridge/tests -v \
    && python3 -m unittest discover -s src/behavior/legacy_setpoint/tests -v

ENV ROS_DOMAIN_ID=80 \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash; source /root/aerostack2_ws/install/setup.bash; source /opt/uav_planner_ws/install/setup.bash; exec ros2 run uav_planner_framework planner_framework_node --ros-args -p registry_root:=/opt/uav_planner_ws/install/share/uav_planner_framework/registry -p resolved_dir:=/var/lib/uav-planner/resolved -p use_sim_time:=true"]
