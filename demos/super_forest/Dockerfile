# Compile native SUPER unchanged, offline, against the existing Humble toolchain.
FROM osrf/ros:humble-desktop-full-jammy AS native
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY thirdparty/SUPER-sim/mars_uav_sim/mars_quadrotor_msgs /opt/super_ws/src/mars_quadrotor_msgs
COPY thirdparty/SUPER-sim/rog_map /opt/super_ws/src/rog_map
COPY thirdparty/SUPER-sim/super_planner /opt/super_ws/src/super_planner
WORKDIR /opt/super_ws
RUN source /opt/ros/humble/setup.bash && MAKEFLAGS=-j2 colcon build \
    --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DSUPER_ENABLE_STACKTRACE=OFF -DBUILD_TESTING=OFF
# Collect native non-ROS runtime dependencies; keep the onboard image's ROS/glibc.
RUN source install/setup.bash && mkdir -p /opt/super_deps/lib && \
    ldd install/super_planner/lib/super_planner/fsm_node > /tmp/super-ldd && \
    python3 -c 'import pathlib,shutil; lines=pathlib.Path("/tmp/super-ldd").read_text().splitlines(); core=("libc.so", "libm.so", "libpthread.so", "libdl.so", "librt.so", "libstdc++", "libgcc_s", "ld-linux"); paths=[line.split("=>",1)[1].strip().split()[0] for line in lines if "=> /" in line]; [shutil.copy2(p,"/opt/super_deps/lib/"+pathlib.Path(p).name,follow_symlinks=True) for p in paths if not p.startswith(("/opt/ros/","/opt/super_ws/")) and not pathlib.Path(p).name.startswith(core)]'

FROM onboard_computer:v0.1.0
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=native /opt/super_ws/install /opt/super_ws/install
COPY --from=native /opt/super_deps /opt/super_deps
COPY src/planner/interfaces /opt/super_app_ws/src/interfaces
COPY src/planner/framework /opt/super_app_ws/src/framework
COPY src/behavior/follow_path /opt/super_app_ws/src/follow_path
COPY src/planner/super/ros2_adapter /opt/super_app_ws/src/super_adapter
WORKDIR /opt/super_app_ws
RUN source /root/aerostack2_ws/install/setup.bash && source /opt/super_ws/install/setup.bash && \
    colcon build --merge-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY runtime/as2 /opt/onboard_computer
COPY runtime/navigation_inputs/config/drone0.yaml /opt/super_app/navigation.yaml
COPY demos/super_forest /opt/super_app
COPY runtime/airsim_sensors/uav_as2_sensor_adapter/rpc.py /opt/super_app/airsim_rpc.py
COPY --from=flight_board:v0.1.0 /usr/lib/python3/dist-packages/msgpack /usr/lib/python3/dist-packages/msgpack
ENV LD_LIBRARY_PATH=/opt/super_deps/lib
CMD ["bash", "-lc", "source /opt/super_ws/install/setup.bash && source /opt/super_app_ws/install/setup.bash && exec python3 /opt/super_app/mission.py"]
