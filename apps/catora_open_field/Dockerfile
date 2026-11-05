FROM companion_as2:airsim-px4
SHELL ["/bin/bash", "-c"]
COPY libraries/cat_ora/humble /opt/catora_app_ws/src/cat_ora
COPY libraries/cat_ora/ros2_adapter /opt/catora_app_ws/src/cat_ora_adapter
COPY planner_interfaces /opt/catora_app_ws/src/planner_interfaces
COPY planner_framework /opt/catora_app_ws/src/planner_framework
RUN source /root/aerostack2_ws/install/setup.bash && cd /opt/catora_app_ws && \
    colcon build --base-paths src/cat_ora src/cat_ora_adapter src/planner_interfaces \
      src/planner_framework src/planner_framework/bridges/as2_setpoint \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY apps/catora_open_field /opt/catora_app
COPY runtime/airsim_sensors/uav_as2_sensor_adapter/rpc.py /opt/catora_app/airsim_rpc.py
COPY --from=flight_board:airsim-px4-humble-20261105-43632de22490 /usr/lib/python3/dist-packages/msgpack /usr/lib/python3/dist-packages/msgpack
ENV AS2_FLEET_FILE=/opt/catora_app/fleet.yaml
CMD ["bash", "-lc", "source /opt/catora_app_ws/install/setup.bash; exec python3 /opt/catora_app/mission.py"]
