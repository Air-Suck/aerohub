# GCOPTER private ROG-Map copy

Copied from `src/planner/super/ros2_adapter/native/rog_map/{include,src}` on
2026-11-12, with its source copyright and license headers preserved. Original
ROG-Map files declare LGPL-3.0-or-later; bundled fmt and utility files retain
their own notices. SUPER import provenance is recorded in its native SOURCE.json
(reference SUPER-sim commit 3bfa4033361bbdc7cd814c94aff826e6e1679018).

This is deliberately a private copy, not a new common map abstraction. GCOPTER
uses the core map API with registered LiDAR and the matching sensor pose.
The ROS1 wrapper is retained as source but is not compiled. The local debug
directory is changed to `/tmp/gcopter-local-map/`. No SUPER planner, trajectory
optimizer, exploration trajectory or backup trajectory is used.

GCOPTER-specific adaptation: the original first-frame-only clearing of the
sensor blind volume is repeated at each measured pose so that takeoff and later
movement do not leave the sensor origin unknown. Already occupied cells are
preserved. This uses the configured minimum ray range (0.5m), not remote scene
geometry. The original map resolution, raycasting and inflation mechanisms are
unchanged; GCOPTER owns a separate configuration. Local planning explicitly
uses probabilistic `isKnownFree` because `isKnownFreeInflate` treats unknown as
nonoccupied when unknown inflation is disabled.
