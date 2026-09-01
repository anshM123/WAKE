import time
import numpy as np
import pytest
import yaml

from wake.pose.apriltag_pose import AprilTagPoseProvider, CameraConfigurationError


def configuration(tmp_path):
    calibration = tmp_path / "intrinsics.yaml"
    calibration.write_text(yaml.safe_dump({
        "image_width": 640,
        "image_height": 480,
        "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
        "distortion_coefficients": [0, 0, 0, 0, 0],
        "mean_reprojection_error_px": .2,
        "camera_identifier": "test",
    }))
    return {
        "camera": {"device": 0, "width": 640, "height": 480, "calibration_file": str(calibration)},
        "tag": {"family": "tag36h11", "id": 0, "size_m": .2},
        "tracking": {"max_reprojection_error_px": 1.5, "min_tag_pixel_width": 35, "max_pose_jump_m": .2, "max_angular_jump_deg": 25, "stale_after_ms": 100, "smoothing_alpha": .25},
        "transforms": {"T_world_from_tag": np.eye(4).tolist(), "T_body_from_camera": np.eye(4).tolist()},
    }


def test_configuration_requires_transforms(tmp_path):
    config = configuration(tmp_path)
    config["transforms"]["T_world_from_tag"] = None
    with pytest.raises(CameraConfigurationError, match="not calibrated"):
        AprilTagPoseProvider(config, open_camera=False)


def test_camera_offset_and_ceiling_transform(tmp_path):
    config = configuration(tmp_path)
    world_from_tag = np.eye(4); world_from_tag[2, 3] = 2.5
    body_from_camera = np.eye(4); body_from_camera[0, 3] = .1
    config["transforms"] = {"T_world_from_tag": world_from_tag.tolist(), "T_body_from_camera": body_from_camera.tolist()}
    provider = AprilTagPoseProvider(config, open_camera=False)
    expected = world_from_tag @ np.eye(4) @ np.linalg.inv(body_from_camera)
    assert expected[:3, 3] == pytest.approx([-.1, 0, 2.5])


def test_stale_tag_returns_none(tmp_path):
    config = configuration(tmp_path)
    provider = AprilTagPoseProvider(config, open_camera=False)
    from wake.types import PoseSample
    provider.filtered_pose = PoseSample("wake-01", 1, time.monotonic_ns() - 200_000_000, (0, 0, 1), (1, 0, 0, 0), 1, 0, 0)
    assert provider.latest_pose() is None
    assert provider.metrics.failure == "TAG_STALE"
