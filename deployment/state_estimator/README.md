# State Estimator validation utilities

This directory contains validation helpers shared by state-estimation
deployments. It is not a runtime estimator and does not publish canonical
state or TF.

`canonical_freshness.py` records canonical pose source stamps and local arrival
times only while the platform reports `FLYING`. Start it before arming, stop it
after landing, and pass its JSON output to the selected estimator's authority
gate. A valid authority run must contain a closed airborne window; a process
that is stopped early leaves the window open and fails the gate.

`sensor_gap_probe.py` is a ground-safe preflight check for a `PointCloud2`
input. It reports unique source stamps, non-monotonic stamps, and maximum source
and local-arrival gaps without publishing or changing the sensor stream. Run it
for longer than the intended acceptance interval before repeating a flight.
