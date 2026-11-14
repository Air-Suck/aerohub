FROM uav/swarm-formation:noetic-px4
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Rebuild actual checked-out algorithm; retain dependency packages from base.
COPY src/tool/navigation/planner/swarm_formation/upstream/src/ /workspace/src/src/
COPY deployment/swarm_formation_ros1/upstream-integration.patch /tmp/upstream-integration.patch
RUN cd /workspace/src && patch --batch --forward -p1 < /tmp/upstream-integration.patch \
    && source /opt/ros/noetic/setup.bash && cd /workspace && catkin_make -j2
COPY src/tool/navigation/planner/swarm_formation/ros1_proxy/ros1_proxy.py /usr/local/bin/swarm_formation_ros1_proxy.py
COPY deployment/swarm_formation_ros1/four_planner.launch /opt/uav/four_planner.launch
COPY deployment/swarm_formation_ros1/configure_waypoints.py /opt/uav/configure_waypoints.py
RUN python3 /opt/uav/configure_waypoints.py /workspace/src/src/planner/plan_manage/launch/run_in_sim.launch \
    && chmod +x /usr/local/bin/swarm_formation_ros1_proxy.py
ENV ROS_MASTER_PORT=11322 ROS_MASTER_URI=http://127.0.0.1:11322 ROS_IP=127.0.0.1
CMD ["bash", "-lc", "source /workspace/devel/setup.bash; roslaunch -p ${ROS_MASTER_PORT} /opt/uav/four_planner.launch & planner=$!; trap 'kill $planner 2>/dev/null || true' EXIT INT TERM; python3 /usr/local/bin/swarm_formation_ros1_proxy.py"]
