# WAKE v0.3.1 experimental platform

WAKE tests whether a single microdrone can infer nearby geometry from
disturbances in its own propeller wake. One ceiling AprilTag supplies global
6-DoF localization only. It is not a mapping sensor, and WAKE uses no LiDAR,
sonar, depth camera, radar, or other direct ranging sensor.

The measurable pipeline is:

```text
MEASURED TELEMETRY + GLOBAL POSE
        ↓
EXPECTED FREE-AIR DYNAMICS
        ↓
AERODYNAMIC RESIDUAL
        ↓
CALIBRATED SURFACE ESTIMATE + UNCERTAINTY
        ↓
PROBABILISTIC 3D MAP
```

Raw observations are immutable and separately recorded. Baseline heuristics do
not update the scientific map and never authorize autonomy. A poor calibration
or a negative result is useful research data; no mapping accuracy or collision
guarantee is claimed.

## Safety boundary

The Matrix flight controller retains stabilization, attitude control, and motor
mixing. WAKE never uses individual motors for autonomous flight. The Matrix
high-level adapter remains fail-closed (`FLIGHT_INTERFACE_DISABLED`) until a
documented, verified velocity/position/RC setpoint interface is available.

`MSP_SET_MOTOR` remains a props-off bench tool requiring compile-time opt-in, a
physical jumper, `PROPS_REMOVED`, increasing sequence numbers, PWM 1000–1100,
and a 250 ms minimum-output watchdog. Never fit props during bench tests.

Unknown space is not free. Manual-flight mapping sends no flight commands.
Autonomy is blocked unless camera/transforms/tag, clock, telemetry, both models,
held-out detection recall, operational envelope, geofence, battery thresholds,
experimentally validated safety distances/deceleration, drone radius, manual
mapping, and the Matrix interface all pass. The program prints every blocker.

## Exact first run

### 1 — Software

```bash
git clone https://github.com/anshM123/WAKE.git
cd WAKE
python -m venv .venv
.venv/Scripts/activate
python -m pip install -e ".[all]"
pytest
```

Linux/macOS activation is `source .venv/bin/activate`.

### 2 — XIAO and Matrix

Configure Wi-Fi, XIAO UART pins, laptop IP, and an unused Matrix UART running MSP
at 115200 baud. Verify 3.3 V compatibility. Flash
`firmware/xiao_wake_bridge/` with props removed. Firmware uses a nonblocking MSP
parser, per-field acquisition timestamps, measured sensor/packet rates, Wi-Fi
health, and clock-sync responses on UDP 5008.

### 3 — Telemetry

```bash
wake record --xiao-host XIAO_IP
```

Verify accelerometer/gyro axes, attitude signs, motor order, battery voltage,
packet loss, and actual IMU/motor/attitude/battery rates. `Ctrl+C` closes and
finalizes the session.

### 4 — Camera calibration

Print a measured checkerboard (default 9×6 inner corners, 24 mm squares), show
it at varied positions and tilts, then run:

```bash
wake calibrate-camera --square-size-m 0.024 --images 20
```

WAKE saves the resolution, matrix, distortion, RMS error, timestamp, and camera
identifier. Runtime rejects a different capture resolution and never accepts
the checked-in null placeholder as calibration.

The capture backend requests a one-frame buffer and timestamps immediately
after `grab()`, before decoding with `retrieve()`. Camera exposure/transport
latency is still hardware-specific: measure it experimentally and set
`camera.capture_latency_ms`. WAKE subtracts that calibrated latency from frame
timestamps and blocks autonomy while it remains null.

### 5 — Ceiling tag

Use a matte, rigid, completely flat `tag36h11`, ID 0, whose physical detection
square is exactly 200 mm × 200 mm (not the paper edge). Secure it face-down near
the center of the validated area. The upward camera must be rigid, low-vibration,
and keep the tag visible across the intended volume.

### 6 — WORLD and rigid transforms

WORLD is right-handed: +X chosen room-right, +Y orthogonal room-forward, +Z up.
Its origin is the floor point below the tag; the tag center is `[0, 0, H]`.
Measure ceiling height, tag yaw relative to +X, camera position from the drone
center, and camera roll/pitch/yaw in body coordinates:

```bash
wake calibrate-transforms \
  --ceiling-height-m H --tag-yaw-deg YAW \
  --camera-position-body-m X Y Z \
  --camera-rpy-body-deg R P Y
```

Physically move toward +X, +Y, and upward; reported X, Y, Z must respectively
increase. Verify yaw direction. Repeat with `--confirm-frame-check` only after
all tests pass. Sign changes belong in transforms, never scattered in code.

### 7 — Tag coverage

```bash
wake apriltag-test --duration 60
```

The preview overlays ID, XYZ, RPY, height below tag, pixel width, reprojection
error, and status. Move the unpowered/props-off drone through the intended
volume. If tracking is not effectively continuous, shrink the geofence. Never
claim full-room localization from inadequate coverage.

### 8 — Clock

```bash
wake clock-test --xiao-host XIAO_IP --duration 15
```

This displays RTT, host/XIAO offset, skew ppm, model age, fit residual, and
confidence. Multiple consecutive healthy samples are required for
`CLOCK SYNC GOOD`. All real-time host timestamps use `time.monotonic_ns()`;
Unix time appears only in metadata.
Runtime safety independently enforces `minimum_clock_confidence` and
`maximum_clock_residual_ms`; passing only the age check is insufficient.

### 9 — Free-air sessions

Collect at least three independent sessions, well away from boundaries where
practical, spanning stable hover, gentle X/Y/vertical translation, gentle yaw,
moderate roll/pitch, multiple throttle states, and early/middle/lower battery.
Avoid aggressive initial-calibration maneuvers.

```bash
wake calibrate-free-air --xiao-host XIAO_IP
```

### 10 — Train free-air model

```bash
wake train-free-air data/sessions/SESSION1 data/sessions/SESSION2 data/sessions/SESSION3 --output models/free_air.json
```

Temporal windows are split by entire session. The artifact reports held-out
per-axis MAE/RMSE/P90, normalization, dataset/split IDs, config hash, and its
operational envelope. Test sessions remain reserved.

### 11 — Define and collect a known wall

Define `n·x+d=0` with a measured unit normal/offset or three measured points:

```bash
wake define-plane --normal 1 0 0 --offset-m -2.35 --name east_wall
# or
wake define-plane --points X1 Y1 Z1 X2 Y2 Z2 X3 Y3 Z3
wake calibrate-wall --xiao-host XIAO_IP
```

Collect independent wall sessions at safe distances, headings, heights, lateral
motion, and motor loads, plus independent negative sessions near nothing.

### 12 — Train/evaluate the surface model

```bash
wake train-surface \
  --wall-sessions WALL1 WALL2 WALL3 \
  --negative-sessions NEG1 NEG2 NEG3 \
  --free-air-model models/free_air.json \
  --output models/surface.json
wake evaluate-model models/surface.json
```

The artifact contains detection precision/recall/FPR, distance MAE/P90, normal
and nearest-surface-direction angular errors, uncertainty, session splits, and
operational envelope. Direction-to-surface and geometric surface normal are
distinct outputs. Held-out recall is reported cumulatively by true distance;
the bin covering the configured caution zone must pass policy. File existence
or overall recall is insufficient.

### 13 — Replay

Set the two model paths in `config/wake.yaml`, then run:

```bash
wake replay data/sessions/SESSION --speed 10
```

Replay reconstructs the clock model, synchronization, pose interpolation,
features, free-air prediction, residual, surface estimate, and map. It reports a
deterministic SHA-256 map hash. `--speed 0` means maximum.

### 14 — Live manual mapping

Human pilot controls the Matrix. WAKE sends no commands:

```bash
wake live --xiao-host XIAO_IP
```

The PySide6/pyqtgraph popup updates independently at 15 Hz. Unknown space is
transparent; repeated confident evidence changes faint gray voxels toward dark,
solid-looking surfaces. It shows fitted patches, drone/heading, trajectory,
suspected ray, and health. Rotate/zoom, select views, pause, toggle layers, and
save screenshots. Use `wake map` for headless manual mapping and
`wake inspect-health` for live JSON health.

### 15 — Compare with measured geometry

```bash
wake export-map SESSION/final_map.json --output-prefix room
wake evaluate-map room_planes.json --reference-plane config/reference_plane.yaml
```

Exports are PLY points, finite supported OBJ patches, and plane JSON. Evaluation
reports measured plane position/normal errors, support, and confidence; missing
surfaces are never hallucinated to close a room.

### 16 — Validate the safety envelope

Experimentally establish detection recall, stop/caution/emergency ranges,
deceleration, latency, physical radius, battery reserve, and geofence. Only then
should autonomous development continue. Required stop distance includes reaction
time, command/network latency, braking distance, uncertainty, and drone radius;
configuration cannot override it downward.

Set `drone_radius_m` to the measured collision radius and
`safe_corridor_padding_m` to validated extra clearance. Known-free return space
is the full swept capsule between consecutive poses using their sum—not a fixed
voxel cube. WAKE leaves the corridor unavailable while either value is null.

## Modes and commands

Modes are `BENCH`, `RECORD_ONLY`, `CALIBRATION_FREE_AIR`, `CALIBRATION_WALL`,
`REPLAY`, `MAPPING_MANUAL_FLIGHT`, `SIMULATION`, and `AUTONOMOUS`. Default is
`RECORD_ONLY`.

```text
wake calibrate-camera       wake apriltag-test
wake calibrate-transforms   wake clock-test
wake record                 wake live
wake map                    wake inspect-health
wake calibrate-free-air     wake define-plane
wake calibrate-wall         wake train-free-air
wake train-surface          wake evaluate-model
wake replay                 wake evaluate-map
wake export-map             wake simulate
```

Each run records raw telemetry, raw/filtered pose, clock exchanges/model,
synchronized samples, preprocessing, prediction, residual, surface estimate,
map updates, health, commands, events, periodic map snapshots, final map, model
versions, Git revision, config/camera hashes, platform/Python, and Unix start/end
time in `data/sessions/YYYYMMDD_HHMMSS/`. Disk and UI queues are bounded; stale
visualization is dropped before scientific/safety data.

## Coordinate/time and map details

Telemetry boot-clock timestamps are converted with a rolling low-RTT affine
model `host_ns = a·xiao_us·1000+b`; `a` measures skew. Telemetry is bracketed by
poses, translation interpolates linearly, rotation uses SLERP, and stale/gapped
samples are rejected. Accelerometer readings are specific force and are rotated
with attitude before gravity is applied. Pose motion uses a local polynomial
derivative, not raw two-frame differences.

The sparse voxel map preserves UNKNOWN. Surface likelihood spreads through
range and angular uncertainty. Conflicts reduce confidence. Traversed volume is
strong known-free return corridor. Deterministic RANSAC extracts supported
wall/floor/ceiling candidates; finite patches use only observed support.

## Hardware-dependent TODOs and honest limits

- Verify and implement a documented Matrix high-level setpoint interface. Raw
  per-motor autonomy is prohibited.
- Measure all null safety values in `config/safety.yaml` on the real vehicle.
- Populate camera intrinsics and rigid transforms using the actual hardware.
- Collect independent free-air/wall/negative sessions and validate both models.
- Validate tag coverage, manual mapping, wake sensitivity, and stopping behavior.
- An FC-disarmed bench gate awaits verified Matrix `MSP_STATUS` semantics.

The synthetic box-room command tests only software mapping/planning behavior. It
does not validate wake physics.
