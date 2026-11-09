FROM uav/catora-hybrid-compute:local
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
RUN source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
 source /root/aerostack2_ws/install/setup.bash && source /opt/swarm_ws/install/setup.bash && cd /opt/swarm_ws && \
 colcon build --parallel-workers 2 --paths src/adapter src/interfaces src/framework src/legacy_setpoint
COPY deployment/airsim/as2 /opt/swarm_app/as2
COPY deployment/airsim/config/as2 /opt/swarm_app/config
COPY deployment/airsim/demos/catora_open_field/hybrid/metrics.py /opt/swarm_app/metrics.py
COPY deployment/airsim/demos/swarm_open_field /opt/swarm_app
ENTRYPOINT ["bash", "/opt/swarm_app/entrypoint.sh"]
CMD ["worker"]
