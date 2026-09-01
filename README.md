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
