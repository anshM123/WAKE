# WAKE

WAKE investigates whether microdrones can infer nearby geometry from the way
their own propeller wake affects closed-loop flight dynamics.

The current hardware-in-the-loop starter stack is in this repository:

- XIAO ESP32-S3 Sense firmware that bridges Matrix MSP telemetry to Wi-Fi UDP.
- A laptop-side telemetry receiver and shared sparse voxel-map prototype.
- Guarded, props-off-only per-motor bench testing.

Run `python laptop/wake_hub.py` on the laptop. Configure and upload
[`firmware/xiao_wake_bridge/xiao_wake_bridge.ino`](firmware/xiao_wake_bridge/xiao_wake_bridge.ino)
to each XIAO after reviewing its pin, Wi-Fi, and motor-test settings.

## Safety

The Matrix is the flight controller and remains in charge of stabilization.
`MSP_SET_MOTOR` is guarded as a **props-off bench-test interface only**: it needs
a compile-time opt-in, a physical jumper, and the command phrase
`PROPS_REMOVED`. It is not used for autonomous flight.

Before wiring, configure an unused Matrix UART for MSP at 115200 baud, confirm
3.3 V UART compatibility, and verify motor order/direction with props removed.

## AprilTag initialization

The initial reference is the supplied **tag36h11 ID 0** printed at **91.3 mm**
(the tag detection square, not its paper margin). `laptop/apriltag_pose.py`
detects it with OpenCV, uses calibrated camera intrinsics to solve its pose in
meters, and emits the existing WAKE `pose` UDP message on port 5006. The hub
therefore receives the camera/drone initial height without any wall-distance
input.

1. Calibrate the exact camera at its actual capture resolution. Put the result
   in `config/camera_calibration.json` using the example schema, but do not use
   the example numbers.
2. Start the hub: `python laptop/wake_hub.py`.
3. Use a stationary webcam above the test area, or use the upward-facing drone
   camera as the script's video source. The tag must be fully visible and flat.
4. Run: `python laptop/apriltag_pose.py --calibration config/camera_calibration.json --drone-id wake-01 --show`.

It requires `opencv-contrib-python` (not plain `opencv-python`) and `numpy`:
`python -m pip install opencv-contrib-python numpy`. Camera-to-drone mounting
offsets can be supplied with `--camera-offset-m X Y Z`.
