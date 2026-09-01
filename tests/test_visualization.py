import numpy as np
from wake.visualization.live import LatestOnlyQueue, voxel_display_arrays


def test_latest_only_queue_drops_stale():
    queue = LatestOnlyQueue(); queue.put(1); queue.put(2)
    assert queue.latest() == 2


def test_voxels_solidify_with_evidence():
    weak = (((0, 0, 0), .6, .2, 1),)
    strong = (((0, 0, 0), .9, .9, 20),)
    _, weak_color = voxel_display_arrays(weak, .05)
    _, strong_color = voxel_display_arrays(strong, .05)
    assert strong_color[0, 3] > weak_color[0, 3]
    assert strong_color[0, 0] < weak_color[0, 0]


def test_unknown_voxels_are_invisible():
    positions, _ = voxel_display_arrays((((0, 0, 0), .5, 0, 0),), .05)
    assert positions.shape == (0, 3)
