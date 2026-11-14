# DPV-SLAM integration

This component runs the official Deep Patch Visual SLAM implementation with
`LOOP_CLOSURE=True`. The upstream source is locked at
`0ac95b656d1fda91c271d2a106460d19ad966fc7` and remains MIT licensed.

DPV-SLAM is monocular: its native translation and sparse reconstruction do not
have an observable metric scale. The ROS 2 adapter therefore performs a
one-time Sim(3) alignment from the first 30 initialized visual poses to
time-paired PX4 local odometry. AirSim truth is never an algorithm input. After
the alignment freezes, subsequent visual motion and loop-closure corrections
produce `dpv_slam/odometry`; the native AS2 plugin composes that low-rate global
correction with high-rate local odometry.

The same algorithm publishes `dpv_slam/sparse_points`. These are DPVO patch
landmarks for visualization and diagnostics, not an occupancy map, ESDF, dense
surface, or independently restartable mapping process. See
`src/basic_function/perception/dpv_slam/README.md` for the perception contract and
`deployment/dpv_slam/README.md` for build and validation commands.

The integration deliberately does not enable `CLASSIC_LOOP_CLOSURE`: that is
the DPV-SLAM++ variant with DBoW2/DPRetrieval, not the selected DPV-SLAM work.
