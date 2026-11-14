# ROG-Map deployment

ROG-Map runs as an independent compute-side Perception capability. It reuses
the project-fixed `onboard_computer:v0.1.0` image and an external overlay; no
ROG-Map-specific Docker image is created. PX4 remains the image-embedded SITL
from `flight_board:v0.1.0`.

## Roles

| Role | Runtime | Processes and interfaces |
| --- | --- | --- |
| Simulator | visible AirSim host process | scene and LiDAR measurements |
| Flight board | `flight_board:v0.1.0` | image-embedded PX4 SITL, clock and sensor bridge |
| Compute | `onboard_computer:v0.1.0` plus external overlay | `navigation_inputs`, ROG-Map core and Adapter |
| Workstation | fixed ROS environment | query consumer, scoring and evidence collection |

## Build

From the feature worktree on a Linux host:

```bash
ROG_MAP_CLEAN_BUILD=1 bash deployment/rog_map/prepare_workspace.sh
```

The script copies the repository source inputs into
`log/workspaces/rog_map/src`, builds them sequentially in the fixed image, runs
their tests, and records the image and source state. Build/install/log remain
outside the image.

## Run

Start the existing AirSim + PX4 SITL chain in visible mode first. The required
environment is:

```bash
export DISPLAY=:3
export AIRSIM_RENDER_MODE=windowed
```

Start `as2_navigation_inputs` with its existing `drone0` configuration, then
start the map in a separate terminal:

```bash
ROS_DOMAIN_ID=<same-domain> bash deployment/rog_map/run.sh
```

The map becomes ready only after it receives a fresh registered cloud and the
LiDAR pose at the same measurement stamp. Query services return `ready=false`
when input is stale. A backward ROS clock jump invalidates the map epoch; stop
and restart the component instead of reusing the old map.

Functional interfaces under `/<namespace>/map/` are `query_voxel` and
`check_line`. `occupied`, `inflated_occupied` and `esdf_slice` are rolling
point snapshots for inspection and RViz, not interchangeable generic map
formats.

## Stop and recovery

The foreground runtime stops with `Ctrl-C`. If an orchestrator owns it, stop
the exact `aerohub-rog-map-<namespace>` container and wait for removal. Never
kill processes by a generic ROG-Map name because another planner may embed its
own map. Restart creates an empty map epoch; consumers must wait for fresh
status before querying.

## Acceptance

Build the two overlays before running the acceptance matrix. Set
`AIRSIM_BINARY` to the locally installed AirSim 1.8.1 executable; do not put a
machine-specific path in shared configuration.

```bash
ROG_MAP_CLEAN_BUILD=1 bash deployment/rog_map/prepare_workspace.sh

export DISPLAY=:3
export AIRSIM_RENDER_MODE=windowed
export AIRSIM_BINARY=/path/to/AirSimNH

python3 deployment/rog_map/run_airsim.py run \
  --scenario normal --output log/airsim/rog-map/normal
python3 deployment/rog_map/run_airsim.py run \
  --scenario sensor_fault --output log/airsim/rog-map/sensor-fault
python3 deployment/rog_map/run_airsim.py run \
  --scenario algorithm_fault --output log/airsim/rog-map/algorithm-fault
```

Each output directory is immutable: use a fresh directory for every retry.
The runner owns only its named AirSim process and containers, records the
visible window and source/image manifests, and verifies that its containers
and ports are released during cleanup.

The accepted 2026-11-14 evidence is indexed in
`skill/temp/rog_map/README.md`. The reproduced scope is visible AirSim 1.8.1,
the image-embedded PX4 SITL, real AirSim LiDAR, AS2-controlled motion and an
independent query consumer. It does not include a physical flight or a generic
persistent-map interchange format.
