FROM uav/swarm-formation:noetic-px4

COPY libraries/swarm_formation/ros1_proxy/ros1_proxy.py /usr/local/bin/swarm_formation_ros1_proxy.py
COPY libraries/swarm_formation/deployment/four_planner.launch /opt/uav/four_planner.launch
RUN planner_launch=/workspace/src/src/planner/plan_manage/launch/run_in_sim.launch && \
    sed -i 's#<arg name="point_num" value="4" />#<arg name="point_num" value="1" />#' "$planner_launch" && \
    sed -i 's#<arg name="point0_x" value="-12.0" />#<arg name="point0_x" value="$(arg target_x)" />#' "$planner_launch" && \
    sed -i 's#<arg name="point0_y" value="3.5" />#<arg name="point0_y" value="$(arg target_y)" />#' "$planner_launch" && \
    sed -i 's#<arg name="point0_z" value="1.2" />#<arg name="point0_z" value="$(arg target_z)" />#' "$planner_launch" && \
    chmod +x /usr/local/bin/swarm_formation_ros1_proxy.py

ENV ROS_MASTER_PORT=11322 \
    ROS_MASTER_URI=http://127.0.0.1:11322 \
    ROS_IP=127.0.0.1

CMD ["bash", "-lc", "source /workspace/devel/setup.bash; roslaunch -p ${ROS_MASTER_PORT} /opt/uav/four_planner.launch & planner=$!; trap 'kill $planner 2>/dev/null || true' EXIT INT TERM; python3 /usr/local/bin/swarm_formation_ros1_proxy.py"]
