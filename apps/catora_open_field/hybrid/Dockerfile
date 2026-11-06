# Existing local image supplies all build dependencies; no downloads in this recipe.
ARG BASE_IMAGE=uav/apps-catora-open-field:humble
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY thirdparty/aerostack2/ /root/aerostack2_ws/src/aerostack2/
COPY thirdparty/as2_platform_pixhawk/ /root/aerostack2_ws/src/as2_platform_pixhawk/
RUN rm -rf /root/aerostack2_ws/build /root/aerostack2_ws/install /root/aerostack2_ws/log && \
    source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
    cd /root/aerostack2_ws && MAKEFLAGS=-j2 colcon build --merge-install --parallel-workers 2 \
      --packages-up-to as2_platform_pixhawk as2_state_estimator as2_motion_controller as2_behaviors_motion \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY libraries/cat_ora/humble /opt/catora_app_ws/src/cat_ora
COPY libraries/cat_ora/ros2_adapter /opt/catora_app_ws/src/cat_ora_adapter
COPY planner_interfaces /opt/catora_app_ws/src/planner_interfaces
COPY planner_framework /opt/catora_app_ws/src/planner_framework
RUN rm -rf /opt/catora_app_ws/build /opt/catora_app_ws/install /opt/catora_app_ws/log && \
    source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && \
    source /root/aerostack2_ws/install/setup.bash && cd /opt/catora_app_ws && \
    MAKEFLAGS=-j2 colcon build --parallel-workers 2 --paths src/cat_ora src/cat_ora_adapter \
      src/planner_interfaces src/planner_framework src/planner_framework/bridges/as2_setpoint \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY runtime/as2/ /opt/catora_hybrid/as2/
COPY config/as2/ /opt/catora_hybrid/config/
COPY apps/catora_open_field/hybrid/ /opt/catora_hybrid/
ENTRYPOINT ["bash", "/opt/catora_hybrid/entrypoint.sh"]
CMD ["worker"]
