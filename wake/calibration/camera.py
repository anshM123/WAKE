"""Interactive checkerboard camera calibration."""
from __future__ import annotations
from datetime import datetime, timezone
from pathlib import Path
import time
import numpy as np
import yaml


def calibrate_camera(*, device: int, width: int, height: int, output: str | Path, columns: int = 9, rows: int = 6, square_size_m: float = 0.024, required_images: int = 20, camera_identifier: str = "camera-0") -> Path:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("install wake-mapper[vision]") from exc
    capture = cv2.VideoCapture(device)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise RuntimeError(f"NO_CAMERA: device {device}")
    object_template = np.zeros((rows * columns, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    object_template *= square_size_m
    object_points, image_points = [], []
    last_capture = 0.0
    try:
        while len(image_points) < required_images:
            ok, frame = capture.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, (columns, rows))
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
                if time.monotonic() - last_capture > .5:
                    object_points.append(object_template.copy())
                    image_points.append(corners)
                    last_capture = time.monotonic()
                cv2.drawChessboardCorners(frame, (columns, rows), corners, True)
            cv2.putText(frame, f"Images {len(image_points)}/{required_images}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)
            cv2.imshow("WAKE camera calibration - ESC to cancel", frame)
            if cv2.waitKey(1) == 27:
                raise RuntimeError("camera calibration cancelled")
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, (width, height), None, None)
        errors = []
        for objects, images, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)
            errors.append(float(cv2.norm(images, projected, cv2.NORM_L2) / len(projected)))
    finally:
        capture.release()
        cv2.destroyAllWindows()
    payload = {"image_width": width, "image_height": height, "camera_matrix": matrix.tolist(), "distortion_coefficients": distortion.reshape(-1).tolist(), "mean_reprojection_error_px": float(np.mean(errors)), "calibration_rms_px": float(rms), "calibration_timestamp": datetime.now(timezone.utc).isoformat(), "camera_identifier": camera_identifier}
    path = Path(output)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path
