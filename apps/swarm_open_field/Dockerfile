FROM uav/catora-hybrid-compute:local
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY libraries/swarm_formation/humble /opt/swarm_ws/src/swarm
COPY runtime/navigation_inputs /opt/swarm_ws/src/navigation_inputs
RUN source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
 source /root/aerostack2_ws/install/setup.bash && cd /opt/swarm_ws && \
 MAKEFLAGS=-j2 colcon build --parallel-workers 2 --base-paths src/swarm src/navigation_inputs \
 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY libraries/swarm_formation/ros2_adapter /opt/swarm_ws/src/adapter
COPY planner_interfaces /opt/swarm_ws/src/interfaces
COPY planner_framework /opt/swarm_ws/src/framework
RUN source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
 source /root/aerostack2_ws/install/setup.bash && source /opt/swarm_ws/install/setup.bash && cd /opt/swarm_ws && \
 colcon build --parallel-workers 2 --paths src/adapter src/interfaces src/framework src/framework/bridges/as2_setpoint
COPY runtime/as2 /opt/swarm_app/as2
COPY config/as2 /opt/swarm_app/config
COPY apps/catora_open_field/hybrid/metrics.py /opt/swarm_app/metrics.py
COPY apps/swarm_open_field /opt/swarm_app
ENTRYPOINT ["bash", "/opt/swarm_app/entrypoint.sh"]
CMD ["worker"]
