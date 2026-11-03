FROM uav/aerostack2-px4:humble-v1.17

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /opt/uav_planner_ws
COPY planner_interfaces/ src/planner_interfaces/
COPY planner_framework/ src/planner_framework/
COPY cat_ora_adapter/ src/cat_ora_adapter/
COPY swarm_formation_adapter/ src/swarm_formation_adapter/
COPY as2_setpoint_adapter/ src/as2_setpoint_adapter/
COPY deployment/platform/fleet_takeoff.py /usr/local/bin/fleet_takeoff.py

RUN chmod +x /usr/local/bin/fleet_takeoff.py

RUN source /opt/ros/humble/setup.bash \
    && source /root/aerostack2_ws/install/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash \
    && python3 -m unittest discover -s src/planner_framework/tests -v \
    && python3 -m unittest discover -s src/cat_ora_adapter/tests -v \
    && python3 -m unittest discover -s src/swarm_formation_adapter/tests -v \
    && python3 -m unittest discover -s src/as2_setpoint_adapter/tests -v

ENV ROS_DOMAIN_ID=80 \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash; source /root/aerostack2_ws/install/setup.bash; source /opt/uav_planner_ws/install/setup.bash; exec ros2 run uav_planner_framework planner_framework_node --ros-args -p registry_root:=/opt/uav_planner_ws/install/share/uav_planner_framework/registry -p resolved_dir:=/var/lib/uav-planner/resolved -p use_sim_time:=true"]
