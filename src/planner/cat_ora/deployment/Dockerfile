FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-c"]
COPY src/planner/cat_ora/humble /opt/catora_ws/src/mrs_formation_reshaping
RUN source /opt/ros/humble/setup.bash && cd /opt/catora_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY src/planner/cat_ora/deployment/start_humble.sh /usr/local/bin/start_catora_humble
RUN chmod +x /usr/local/bin/start_catora_humble

CMD ["/usr/local/bin/start_catora_humble"]
