ARG BASE_IMAGE=onboard_computer:v0.1.0
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
COPY src/planner/cat_ora/ros2_adapter/native /opt/catora_app_ws/src/cat_ora
COPY src/planner/cat_ora/ros2_adapter/bridge /opt/catora_app_ws/src/cat_ora_adapter
COPY src/planner/interfaces /opt/catora_app_ws/src/interfaces
COPY src/planner/framework /opt/catora_app_ws/src/framework
COPY src/behavior/follow_path /opt/catora_app_ws/src/follow_path
COPY deployment/airsim/as2/ /opt/onboard_computer/
COPY src/behavior/legacy_setpoint /opt/catora_app_ws/src/legacy_setpoint
RUN source /root/aerostack2_ws/install/setup.bash && cd /opt/catora_app_ws && \
    colcon build --base-paths src/cat_ora src/cat_ora_adapter src/interfaces \
      src/framework src/legacy_setpoint src/follow_path \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY deployment/airsim/demos/catora_open_field /opt/catora_app
COPY simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py /opt/catora_app/airsim_rpc.py
COPY --from=flight_board:v0.1.0 /usr/lib/python3/dist-packages/msgpack /usr/lib/python3/dist-packages/msgpack
ENV AS2_FLEET_FILE=/opt/catora_app/fleet.yaml
CMD ["bash", "-lc", "source /opt/catora_app_ws/install/setup.bash; exec python3 /opt/catora_app/mission.py"]
