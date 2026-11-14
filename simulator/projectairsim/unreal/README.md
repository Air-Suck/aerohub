# UE5 observation display

`AerohubDisplay` is an independent UE 5.2 plugin. It uses public UE actor,
scene-capture and HUD interfaces, without modifying ProjectAirSim core code.
Its source lives here; generated binaries are installed in the external Blocks
project. The project must be rebuilt/repackaged to include it in a packaged sim.

Build and install (restart the editor once after installing a new module):

```bash
bash simulator/projectairsim/build_display.sh \
  ~/UE_installed_build ~/ProjectAirSim/unreal/Blocks/Blocks.uproject \
  log/projectairsim/display-plugin
```

Configuration is in `config/simulator/*.yaml`, under `display`:

```yaml
display:
  enabled: true
  vehicle: Drone1
  camera_enabled: false
  camera: DownCamera
  camera_width: 320
  speed_max_mps: 20
  port: 8991
```

The default layout keeps mission state and wall time in a compact top panel and
puts observed speed in a dedicated lower-right dial. `speed_max_mps` sets the
dial scale and can be changed without recompiling. The dial borrows the visual
idea of a large numeric readout, radial ticks and a moving needle from
XXLiu-HNU/Speed-Visualization, but is independently implemented with UE Canvas;
no source code or graphics are copied from that project.

`attach` writes the resolved display settings to the selected UE project's
`Saved/Aerohub/display.json`. The HUD reloads settings every 0.5 game seconds;
port changes require restarting Play. Set `projectairsim.project_resource` when
using a project other than Blocks. A manager-started packaged host gets its
configuration path through `AEROHUB_DISPLAY_CONFIG`, without changing the map.

The camera inset is not drawn in the current v5 layout. Camera capture settings
remain available for the simulator sensor bridge.

The HUD exposes a local HTTP interface (UE HTTPServer defaults to loopback):

| Request | Meaning |
| --- | --- |
| GET `/aerohub/status` | Vehicle presence, observed speed, timer and camera target state |
| POST `/aerohub/telemetry` | Manager-owned speed reader supplies vehicle and speed_mps |
| POST `/aerohub/mission/start` | Start wall-clock timer from idle/finished |
| POST `/aerohub/mission/finish` | Freeze a running timer |
| POST `/aerohub/mission/reset` | Reset display timer to idle |

The manager wraps this interface:

```bash
bash simulator/simulator.sh display --event status
bash simulator/simulator.sh display --event start
bash simulator/simulator.sh display --event finish
bash simulator/simulator.sh display --event reset
```

These events change the observation display only. They do not start/cancel a
planner, arm/disarm PX4, teleport a vehicle, or reset the simulator. A mission
client can emit them alongside its actual lifecycle operations. Invalid timer
transitions return HTTP 400. Repeated start while running does not reset time.
This interface supplements the existing ProjectAirSim RPC; world/object/camera
operations continue to use upstream APIs.

Current limits:

- Timer measures monotonic wall time, including pauses; the compact label is
  TIME. It is not a simulator-time benchmark.
- Speed is the magnitude of upstream ProjectAirSim ground-truth `twist.linear`
  in m/s, forwarded by `display_telemetry.py` using service-only RPC. It is not
  PX4 telemetry. Data older than one wall second displays `--`.
- Peak is accumulated since startup/reset. READY/TRACKING/FINISHED reflects
  display timer state, not planner health. The review viewport is 1280×720.
- Presence does not mean PX4 is connected or ready.
- One local player/world per port; simultaneous PIE worlds need distinct ports.
- The plugin replaces the default HUD; projects with an existing custom HUD
  should integrate the drawing rather than enabling both independently.
- Editor, Development and Shipping compilation was checked on UE 5.2.1. Actual
  runtime checks used a standalone editor game window. The old packaged Blocks
  binary does not acquire this plugin without rebuilding it.

## Runtime validation (2026-11-12)

UE 5.2.1 standalone editor game, separate ports 9989/9990 and display 8992:

- Actual Blocks rendering, vehicle discovery and native RGB render-target display
  checked in a saved window capture.
- Native PX4 scene load and camera target discovery checked without PX4 flight.
- Actual image rendering checked with the upstream SimpleFlight sensor scene and
  native C++ ROS bridge, on separate ROS domain 189.
- Timer start/finish/reset, elapsed time increase, frozen finish, invalid transition
  HTTP 400, missing-camera handling and live camera selection checked.
- Final CLI-to-UE status query and active camera flag checked.
- Compact top status panel and lower-right speed dial visually checked at
  1280x720 in both idle and running display states; the camera inset was off.
- These initial checks preceded the moving-flight check documented below.
  Packaged Blocks runtime acceptance was outstanding at that point; see the 2026-11-13 package check below.

Local evidence: `log/projectairsim/display-validation/` (`hud-gauge.png`,
`hud-gauge-running.png`, `control.json`, `lifecycle.json`, `final-status.json`,
UE and bridge logs).

Latest flight check: native AS2 Takeoff 2 m → GoTo +3 m at 1 m/s → hold 6 s →
Land/disarm passed. Ground-truth HUD speed median during GoTo was 1.001 m/s,
peak 1.113 m/s, hold final 0.037 m/s and landing final zero. Telemetry shutdown
returned null current speed. Evidence: `log/projectairsim/display-flight/`.
This was a separate game instance; user-visible dynamic review in the intended
window remains outstanding. Earlier actor-difference spikes and a failed
connection attempt are retained in that evidence directory. Manager automatic
telemetry ownership has unit coverage; the successful flight used a manually
started reader. The old camera and actor-speed validation above is historical.


## Packaged course runtime check (2026-11-13)

The Linux Development package now includes the reviewed two-stage Blocks course v7
and this HUD, including both Nimbus Mono fonts. The separate runtime map is
`/Game/BlocksPlannerCourse`; reviewed editor maps remain preserved.
The static scale-reference drone and stale original-building reflection captures
are editor-only in the runtime copy.

Actual packaged Vulkan rendering at 1280×720, 24 obstacle objects, one unarmed
SimpleFlight robot at the green start, real ground-truth speed forwarding,
and HUD start/finish/reset plus invalid-transition HTTP 400 were checked.
No flight, PX4, ROS/AS2 or planner run was performed for this package check.
The launcher supplies `-noraytracing` to avoid a UE 5.2 Vulkan extension/device
creation failure observed with NVIDIA 580.178.04; accepted SunSky/SSGI lighting remains.

Build, launch and interface instructions: [course package](../course/PACKAGE_README.md).
Artifacts and validation evidence: `log/projectairsim/package-course-v7/`.
