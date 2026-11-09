ARG BASE_IMAGE=uav/catora-hybrid-compute:local
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY src/planner/swarm_formation/ros2_adapter/native /opt/swarm_ws/src/swarm
COPY src/perception/navigation_inputs /opt/swarm_ws/src/navigation_inputs
RUN source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
 source /root/aerostack2_ws/install/setup.bash && cd /opt/swarm_ws && \
 MAKEFLAGS=-j2 colcon build --parallel-workers 2 --base-paths src/swarm src/navigation_inputs \
 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY src/planner/swarm_formation/ros2_adapter/bridge /opt/swarm_ws/src/adapter
COPY src/planner/interfaces /opt/swarm_ws/src/interfaces
COPY src/planner/framework /opt/swarm_ws/src/framework
COPY src/behavior/legacy_setpoint /opt/swarm_ws/src/legacy_setpoint
COPY src/behavior/follow_path /opt/swarm_ws/src/follow_path
RUN source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
 source /root/aerostack2_ws/install/setup.bash && source /opt/swarm_ws/install/setup.bash && cd /opt/swarm_ws && \
 colcon build --parallel-workers 2 --paths src/adapter src/interfaces src/framework src/legacy_setpoint src/follow_path
COPY deployment/airsim/as2 /opt/swarm_app/as2
COPY deployment/airsim/config/as2 /opt/swarm_app/config
COPY deployment/airsim/demos/catora_open_field/mission.py /opt/swarm_app/common/flight_mission.py
COPY deployment/airsim/demos/catora_open_field/hybrid/metrics.py /opt/swarm_app/metrics.py
COPY deployment/airsim/demos/swarm_open_field /opt/swarm_app
COPY simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py /opt/swarm_app/airsim_rpc.py
COPY --from=flight_board:v0.1.0 /usr/lib/python3/dist-packages/msgpack /usr/lib/python3/dist-packages/msgpack
ENTRYPOINT ["bash", "/opt/swarm_app/entrypoint.sh"]
CMD ["worker"]
