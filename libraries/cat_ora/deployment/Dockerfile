FROM uav/catora:noetic-upstream

COPY libraries/cat_ora/ros1_proxy/ros1_proxy.py /usr/local/bin/cat_ora_ros1_proxy.py
RUN chmod +x /usr/local/bin/cat_ora_ros1_proxy.py

CMD ["bash", "-lc", "source /opt/catora_ws/devel/setup.bash; roslaunch -p ${ROS_MASTER_PORT:-11321} mrs_formation_reshaping catora.launch rviz:=false & planner=$!; trap 'kill $planner 2>/dev/null || true' EXIT INT TERM; python3 /usr/local/bin/cat_ora_ros1_proxy.py"]
