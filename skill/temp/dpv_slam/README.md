# DPV-SLAM shared-environment acceptance, 2026-11-14

## Scope and Skills

Used `uav-ros-adaptation` for timestamp, deterministic runtime and Adapter
coverage contracts, and `uav-sim-e2e-validation` for shadow, authority,
failure recovery and evidence rules. The platform remained visible AirSim
1.8.1 (`DISPLAY=:3`) + locked flight-board PX4 SITL + shared
`onboard_computer:v0.1.0`; no algorithm-specific image was built.

## Changes and diagnosis

- Preserved DPVO observation timestamps through removed-frame delta chains.
- Fixed the runtime seed to upstream's official evaluation seed, `1234`.
- Used an AirSim profile based on upstream `fast.yaml` and official
  `BACKEND_THRESH=18`; loop closure remains enabled.
- Separated loop-closing correction and high-rate canonical freshness. Schema
  0.2 uses 2.5 seconds globally and retains 0.1 seconds for control state.
- Matched candidate output to source stamps reported by the Adapter.
- Fault tests fly an observable square, stop the owned container, verify
  canonical continuity, then Land and disarm.

Historical failures remain under `log/dpv_slam/`, including old 1.5-second
gate failures and the initial missing fault-ready implementation.

## Final matrix

| Scenario | Evidence | Result |
| --- | --- | --- |
| Shadow | `log/dpv_slam/shadow-schema-0.2-current-frame-20261114-01` | Pass: 0.178 m RMSE, 2.274 s global gap, full coverage |
| Authority | `log/dpv_slam/authority-schema-0.2-current-frame-20261114-01` | Pass: 0.162 m RMSE; canonical gaps 0.024/0.033 s |
| RGB loss | `log/dpv_slam/sensor-fault-schema-0.2-final-20261114-01` | Pass: 2.03 s detection, 50 hold samples, Land/disarm |
| Algorithm exit | `log/dpv_slam/algorithm-fault-schema-0.2-final-20261114-01` | Pass: 2.05 s detection, 50 hold samples, Land/disarm |

The result is reproduced for locked AirSim + PX4 SITL. Real flight, PX4 local
estimator failure and sparse-landmark conversion remain outside this claim.
