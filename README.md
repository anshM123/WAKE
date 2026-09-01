# WAKE v0.2

WAKE is a research platform for testing whether a single microdrone can infer
nearby geometry from disturbances in its own propeller wake. AprilTag tracking
provides global 6-DoF localization only; it is **not** a mapping sensor. No
LiDAR, sonar, depth camera, radar, or direct range measurement enters the map.

The scientific pipeline is deliberately inspectable:

```text
raw telemetry + timestamped global pose
  -> interpolation at the same timestamp
  -> calibration/filtering and correct specific-force handling
  -> expected free-air dynamics
  -> aerodynamic residual (observed - expected)
  -> surface estimate with uncertainty
  -> probabilistic evidence fusion
  -> sparse voxel map and supported surfaces
```

Raw observations are immutable. Changing a filter or model means replaying the
session, never rewriting measurements. The baseline estimators are experimental
debug tools and are not evidence that wake-based ranging works.

## Safety state

The Matrix flight controller remains responsible for stabilization. Autonomous
hardware control is fail-closed: `MatrixAdapter` raises an error because no
verified, documented high-level Matrix setpoint protocol is known yet.
`MSP_SET_MOTOR` exists only in the firmware bench module and still requires all
of the following:

- compile-time `ENABLE_BENCH_MOTOR_TESTS=true`;
- a physical jumper from the configured pin to ground;
- the exact phrase `PROPS_REMOVED`;
- a monotonically increasing command sequence;
- output in the restricted 1000–1100 range.

A 250 ms watchdog automatically commands every motor to minimum. An FC-disarmed
gate is documented as a TODO until Matrix's `MSP_STATUS` semantics are verified.
Do not fit props during bench motor tests.

The initial reference is the supplied **tag36h11 ID 0** printed at **200 mm**
(the tag detection square, not its paper margin). `laptop/apriltag_pose.py`
detects it with OpenCV, uses calibrated camera intrinsics to solve its pose in
meters, and emits the existing WAKE `pose` UDP message on port 5006. The hub
therefore receives the camera/drone initial height without any wall-distance
input.

`AUTONOMOUS` is not the default and is blocked while models, geofence, battery
limits, safety distances, or the Matrix interface are placeholders. Unknown
space is not free space. No accuracy or collision guarantee is claimed.

## Installation and tests

Python 3.10+ is required. ROS is not required.

```bash
python -m venv .venv
.venv/Scripts/activate          # Windows
python -m pip install -e ".[test]"
pytest
```

Install `.[vision]` for AprilTag work and `.[science]` for model training and
plane fitting experiments. Camera dependencies are isolated from replay and
mapping.

## Frames and time

All non-camera frames are right-handed. `WORLD` defaults to +X east, +Y north,
+Z up. `DRONE_BODY` and `FC_BODY` default to +X forward, +Y left, +Z up; verify
this against the actual FC and configure a rigid transform if it differs.
Detector-specific camera and tag conventions must pass through
`wake.pose.transforms`, not local sign flips.

`T_world_from_camera` and `T_body_from_tag` are configured in
`config/camera.yaml`. The latter moves the observed tag pose to the drone center
of mass. Identity `T_world_from_camera` is acceptable for the first experiment.

Firmware timestamps samples with the XIAO monotonic clock. A deployment must
estimate the XIAO-to-host clock relationship; the synchronizer accepts an
already converted host timestamp, brackets it with poses, linearly interpolates
translation, SLERPs rotation, and rejects excessive gaps. It never substitutes
the newest pose. The session records packet loss, latency, age, and sync error.

## Firmware

Review pins, 3.3 V UART compatibility, Wi-Fi values, and ports in
`firmware/xiao_wake_bridge/` before flashing. Configure an unused Matrix UART
for MSP at 115200 baud. The nonblocking parser maintains one outstanding MSP
request and prioritizes IMU/motor data, then attitude and battery. Reported
achieved rate is measured rather than assumed.

Telemetry protocol v2 requires drone ID, uint32 sequence, monotonic timestamp,
accelerometer, gyro, attitude, four motor outputs, battery, validity flags, and
health. The laptop rejects malformed, duplicate, and reordered packets and
counts loss across sequence rollover.

## AprilTag localization

The default family is configurable `tagStandard41h12`, tag ID 0. Measure the
black detection square accurately and set `tag.size_m`; it is intentionally
`null` in source control. Calibrate the exact camera at its actual resolution
and populate its matrix and distortion coefficients. Quality gates cover wrong
ID, reprojection error, decision margin, jumps, and staleness. The pose provider
boundary supports mock/replayed, UDP, and AprilTag sources without coupling the
rest of WAKE to a camera library.

## Commands

After `pip install -e .`, use:

```text
wake record
wake map
wake replay SESSION --speed 10
wake calibrate-free-air
wake calibrate-wall
wake train
wake evaluate-model
wake export-map MAP.json --ply map.ply
wake inspect-health
```

`0` replay speed means maximum; positive values are real-time multipliers. The
legacy `python laptop/wake_hub.py` command remains a compatibility launcher for
the validated recorder. Commands that need calibration data currently stop at
an explicit interface message instead of fabricating a trained artifact.

Every run creates `data/sessions/YYYYMMDD_HHMMSS/` with metadata, a configuration
snapshot, and separate JSONL raw telemetry, pose, synchronized, event, and
command streams. Disk writes use a bounded background queue so they cannot
block acquisition or safety logic.

## Required development order

1. Flash firmware and verify telemetry with props removed.
2. Verify IMU axes, gyro axes, attitude, motor order, and battery readings.
3. Set up the AprilTag pose system.
4. Verify the pose frame and measured tag-to-body transform.
5. Record free-air datasets across throttle, orientation, battery, hover,
   gentle translation, and gentle rotation.
6. Record controlled known-wall sessions using a manually measured WORLD plane;
   include safe variations and negative/free-space sessions.
7. Train and evaluate the free-air model using whole-session splits.
8. Train and evaluate the surface model, including false positives, distance
   MAE/P90, angular error, and confidence calibration.
9. Replay sessions and verify deterministic mapping and exports.
10. Fly a manual-control mapping test; WAKE only records/maps.
11. Experimentally validate uncertainty and stop-distance behavior.
12. Only then consider conservative autonomy, after implementing and verifying
    a documented Matrix high-level setpoint interface.

Never jump directly from software tests to autonomous room flight.

## Calibration and evaluation rules

Training examples are temporal windows. Split by entire session, never random
frames. A learned artifact must contain its version, training timestamp, feature
list, normalization, dataset IDs, validation metrics, and configuration hash.
Missing artifacts are reported as `UNCALIBRATED` and block autonomy.

Known-wall labels come from measured WORLD planes and AprilTag global pose, so
distance and body-frame normal are reproducible. Hardware experiments—not a
simple simulator—must establish whether the aerodynamic signal is usable.
Synthetic data is appropriate only for transforms, mapping, exports, planning,
and safety software.

Map reports should compare against measured reference geometry: surface-point
and plane-position error, normal and corner error, room-dimension error, false
occupied/free volume, coverage, and confidence calibration. A visually pleasing
map is not an accuracy metric, and unseen surfaces must never be filled in.

## Current v0.2 boundary

Core protocol validation, frame math, synchronization, gravity/specific-force
handling, filters, free-air/residual/surface abstractions, uncertainty-aware
sparse mapping, exports, recording/replay, high-level control abstraction,
independent safety policy, frontier scoring, and firmware safety/state-machine
boundaries are implemented and unit-tested. Hardware-specific camera detection,
scientifically trained models, calibrated thresholds, robust plane/mesh fitting,
and Matrix high-level commands require real hardware data or documentation and
remain explicit interfaces/placeholders rather than invented behavior.
