"""OpenCV AprilTag localization with measured quality metrics and transforms."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time
from typing import Any
import numpy as np

from wake.config import load_yaml
from wake.pose.transforms import matrix_to_quaternion, quaternion_to_matrix, slerp
from wake.types import PoseSample


class CameraConfigurationError(RuntimeError):
    pass


@dataclass(frozen=True)
class AprilTagMetrics:
    tag_detected: bool = False
    reprojection_error_px: float = math.inf
    tag_pixel_width: float = 0.0
    pose_age_ms: float = math.inf
    pose_jump_m: float = 0.0
    angular_jump_deg: float = 0.0
    tracking_confidence: float = 0.0
    failure: str | None = "NO_TAG"


@dataclass(frozen=True)
class CameraCalibration:
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    mean_reprojection_error_px: float
    camera_identifier: str

    @classmethod
    def load(cls, path: str | Path) -> "CameraCalibration":
        raw = load_yaml(path)
        required = ("image_width", "image_height", "camera_matrix", "distortion_coefficients", "mean_reprojection_error_px", "camera_identifier")
        missing = [key for key in required if raw.get(key) is None]
        if missing:
            raise CameraConfigurationError(f"BAD_CAMERA_CALIBRATION: missing {', '.join(missing)}")
        matrix = np.asarray(raw["camera_matrix"], dtype=float)
        distortion = np.asarray(raw["distortion_coefficients"], dtype=float)
        if matrix.shape != (3, 3):
            raise CameraConfigurationError("camera_matrix must be 3x3")
        return cls(int(raw["image_width"]), int(raw["image_height"]), matrix, distortion, float(raw["mean_reprojection_error_px"]), str(raw["camera_identifier"]))


def _rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def _as_transform(value: Any, name: str) -> np.ndarray:
    if value is None:
        raise CameraConfigurationError(f"{name} is not calibrated")
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4):
        raise CameraConfigurationError(f"{name} must be 4x4")
    return transform


class AprilTagPoseProvider:
    """Own camera capture, detection, solve, transforms, filtering, and gates."""

    FAMILY_MAP = {"tag36h11": "DICT_APRILTAG_36h11"}

    def __init__(self, camera_config: dict[str, Any], *, open_camera: bool = True) -> None:
        self.config = camera_config
        camera, tag = camera_config["camera"], camera_config["tag"]
        self.tracking, transforms = camera_config["tracking"], camera_config["transforms"]
        if tag["family"] not in self.FAMILY_MAP:
            raise CameraConfigurationError(f"unsupported tag family {tag['family']}")
        self.drone_id, self.tag_id, self.tag_size_m = "wake-01", int(tag["id"]), float(tag["size_m"])
        self.T_world_from_tag = _as_transform(transforms.get("T_world_from_tag"), "T_world_from_tag")
        self.T_body_from_camera = _as_transform(transforms.get("T_body_from_camera"), "T_body_from_camera")
        self.calibration = CameraCalibration.load(camera["calibration_file"])
        self.width, self.height = int(camera["width"]), int(camera["height"])
        if (self.width, self.height) != (self.calibration.image_width, self.calibration.image_height):
            raise CameraConfigurationError("camera resolution differs from calibration")
        self.sequence = 0
        self.raw_pose: PoseSample | None = None
        self.filtered_pose: PoseSample | None = None
        self.metrics = AprilTagMetrics()
        self.capture = self.detector = None
        half = self.tag_size_m / 2.0
        self.object_points = np.asarray([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        if open_camera:
            self._open_camera(int(camera["device"]), tag["family"])

    def _open_camera(self, device: int, family: str) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise CameraConfigurationError("NO_CAMERA: install wake-mapper[vision]") from exc
        self.cv2 = cv2
        capture = cv2.VideoCapture(device)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not capture.isOpened():
            raise CameraConfigurationError(f"NO_CAMERA: device {device}")
        actual = (round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if actual != (self.width, self.height):
            capture.release()
            raise CameraConfigurationError(f"camera produced {actual}, calibrated for {(self.width, self.height)}")
        dictionary_id = getattr(cv2.aruco, self.FAMILY_MAP[family])
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dictionary_id), parameters)
        self.capture = capture

    def process_frame(self, frame: np.ndarray, timestamp_ns: int) -> PoseSample | None:
        corners, identifiers, _ = self.detector.detectMarkers(frame)
        if identifiers is None:
            self.metrics = AprilTagMetrics(failure="NO_TAG")
            return None
        matches = [index for index, value in enumerate(identifiers.flatten()) if int(value) == self.tag_id]
        if not matches:
            self.metrics = AprilTagMetrics(tag_detected=True, failure="WRONG_TAG_ID")
            return None
        image_corners = corners[matches[0]].reshape(4, 2).astype(np.float64)
        pixel_width = float(np.mean([np.linalg.norm(image_corners[(i + 1) % 4] - image_corners[i]) for i in range(4)]))
        if pixel_width < float(self.tracking["min_tag_pixel_width"]):
            self.metrics = AprilTagMetrics(True, tag_pixel_width=pixel_width, failure="TAG_TOO_SMALL")
            return None
        solved, rvec, tvec = self.cv2.solvePnP(self.object_points, image_corners, self.calibration.camera_matrix, self.calibration.distortion_coefficients, flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
        if not solved:
            self.metrics = AprilTagMetrics(True, tag_pixel_width=pixel_width, failure="POSE_SOLVE_FAILED")
            return None
        projected, _ = self.cv2.projectPoints(self.object_points, rvec, tvec, self.calibration.camera_matrix, self.calibration.distortion_coefficients)
        difference = projected.reshape(4, 2) - image_corners
        reprojection = float(np.sqrt(np.mean(np.sum(difference**2, axis=1))))
        if reprojection > float(self.tracking["max_reprojection_error_px"]):
            self.metrics = AprilTagMetrics(True, reprojection, pixel_width, failure="HIGH_REPROJECTION_ERROR")
            return None
        rotation_camera_from_tag, _ = self.cv2.Rodrigues(rvec)
        T_camera_from_tag = np.eye(4)
        T_camera_from_tag[:3, :3] = rotation_camera_from_tag
        T_camera_from_tag[:3, 3] = np.asarray(tvec).reshape(3)
        T_world_from_body = self.T_world_from_tag @ np.linalg.inv(T_camera_from_tag) @ np.linalg.inv(self.T_body_from_camera)
        position, rotation = T_world_from_body[:3, 3], T_world_from_body[:3, :3]
        jump_m, angular_jump = self._jump(position, rotation)
        if jump_m > float(self.tracking["max_pose_jump_m"]) or angular_jump > float(self.tracking["max_angular_jump_deg"]):
            self.metrics = AprilTagMetrics(True, reprojection, pixel_width, 0, jump_m, angular_jump, 0, "POSE_JUMP")
            return None
        confidence = self._confidence(reprojection, pixel_width)
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.raw_pose = PoseSample(self.drone_id, self.sequence, timestamp_ns, tuple(position.tolist()), matrix_to_quaternion(rotation), confidence, reprojection, self.tag_id)
        self.filtered_pose = self._filter(self.raw_pose)
        self.metrics = AprilTagMetrics(True, reprojection, pixel_width, 0, jump_m, angular_jump, confidence, None)
        return self.filtered_pose

    def _jump(self, position: np.ndarray, rotation: np.ndarray) -> tuple[float, float]:
        if self.raw_pose is None:
            return 0.0, 0.0
        return float(np.linalg.norm(position - np.asarray(self.raw_pose.position_world_m))), _rotation_angle_deg(quaternion_to_matrix(self.raw_pose.rotation_world_from_body), rotation)

    def _confidence(self, reprojection: float, pixel_width: float) -> float:
        reprojection_score = max(0.0, 1.0 - reprojection / float(self.tracking["max_reprojection_error_px"]))
        pixel_score = min(1.0, pixel_width / (2 * float(self.tracking["min_tag_pixel_width"])))
        return reprojection_score * pixel_score

    def _filter(self, raw: PoseSample) -> PoseSample:
        if self.filtered_pose is None:
            return raw
        alpha = float(self.tracking.get("smoothing_alpha", 0.25))
        position = (1 - alpha) * np.asarray(self.filtered_pose.position_world_m) + alpha * np.asarray(raw.position_world_m)
        rotation = slerp(self.filtered_pose.rotation_world_from_body, raw.rotation_world_from_body, alpha)
        return PoseSample(raw.drone_id, raw.sequence, raw.timestamp_ns, tuple(position.tolist()), rotation, raw.tracking_confidence, raw.reprojection_error, raw.tag_id)

    def capture_frame(self) -> tuple[np.ndarray, PoseSample | None]:
        if self.capture is None:
            raise CameraConfigurationError("NO_CAMERA")
        ok, frame = self.capture.read()
        timestamp_ns = time.monotonic_ns()
        if not ok:
            self.metrics = AprilTagMetrics(failure="NO_CAMERA_FRAME")
            raise CameraConfigurationError("NO_CAMERA_FRAME")
        return frame, self.process_frame(frame, timestamp_ns)

    def capture_once(self) -> PoseSample | None:
        _, pose = self.capture_frame()
        return pose

    def latest_pose(self) -> PoseSample | None:
        if self.filtered_pose is None:
            return None
        age_ms = (time.monotonic_ns() - self.filtered_pose.timestamp_ns) / 1e6
        if age_ms > float(self.tracking["stale_after_ms"]):
            values = {**self.metrics.__dict__, "pose_age_ms": age_ms, "failure": "TAG_STALE"}
            self.metrics = AprilTagMetrics(**values)
            return None
        return self.filtered_pose

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
