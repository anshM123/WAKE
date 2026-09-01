#!/usr/bin/env python3
"""Detect tag36h11 ID 0 and publish camera/drone pose to the WAKE hub.

The tag is the local world origin. With an upward-facing camera on the drone,
the camera's +Z position in this frame is the initial height below the ceiling
tag (after any camera-to-drone offset is applied).
"""
import argparse, json, math, socket, time
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit("Install dependencies: python -m pip install opencv-contrib-python numpy") from exc

def rpy_from_matrix(r):
    # Standard intrinsic XYZ roll-pitch-yaw extraction.
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    return [float(roll), float(pitch), float(yaw)]

def load_calibration(path):
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    k = np.asarray(raw["camera_matrix"], dtype=np.float64)
    d = np.asarray(raw["distortion_coefficients"], dtype=np.float64)
    if k.shape != (3, 3): raise ValueError("camera_matrix must be 3 by 3")
    return k, d

def main():
    p = argparse.ArgumentParser(description="AprilTag 36h11 ID 0 pose -> WAKE pose UDP")
    p.add_argument("--camera", type=int, default=0, help="OpenCV webcam device index")
    p.add_argument("--calibration", required=True, help="JSON camera intrinsics/distortion file")
    p.add_argument("--hub", default="127.0.0.1"); p.add_argument("--port", type=int, default=5006)
    p.add_argument("--drone-id", default="wake-01")
    p.add_argument("--tag-size-m", type=float, default=0.0913, help="black tag detection square, not paper edge")
    p.add_argument("--camera-offset-m", nargs=3, type=float, default=[0, 0, 0], metavar=("X","Y","Z"), help="camera position in drone body frame")
    p.add_argument("--show", action="store_true")
    a = p.parse_args()
    if a.tag_size_m <= 0: raise SystemExit("--tag-size-m must be positive")
    K, dist = load_calibration(a.calibration)
    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened(): raise SystemExit(f"Could not open camera {a.camera}")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    half = a.tag_size_m / 2
    # OpenCV tag object points, origin at center. The supplied 91.3 mm is this square.
    object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        ok, frame = cap.read()
        if not ok: continue
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            for c, tag_id in zip(corners, ids.flatten()):
                if int(tag_id) != 0: continue
                # PnP provides tag-in-camera. Invert it for camera/drone-in-tag/world.
                solved, rvec_ct, tvec_ct = cv2.solvePnP(object_points, c.reshape(4, 2), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not solved: continue
                R_ct, _ = cv2.Rodrigues(rvec_ct); R_wc = R_ct.T
                camera_world = -R_wc @ tvec_ct.reshape(3)
                drone_world = camera_world - R_wc @ np.asarray(a.camera_offset_m)
                payload = {"type":"pose", "source":"apriltag36h11", "tag_id":0, "id":a.drone_id,
                           "t_us":time.time_ns() // 1000, "position_m":drone_world.tolist(),
                           "rpy_rad":rpy_from_matrix(R_wc), "velocity_mps":[0.0,0.0,0.0],
                           "initial_height_m":float(abs(drone_world[2]))}
                udp.sendto(json.dumps(payload).encode("utf-8"), (a.hub, a.port))
                if a.show:
                    cv2.aruco.drawDetectedMarkers(frame, [c], np.array([[0]], dtype=np.int32))
                    cv2.putText(frame, f"ID 0 height {payload['initial_height_m']:.3f} m", (20,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,0), 2)
        if a.show:
            cv2.imshow("WAKE AprilTag pose", frame)
            if cv2.waitKey(1) == 27: break
    cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__": main()
