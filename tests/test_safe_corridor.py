from wake.mapping.safe_corridor import mark_swept_corridor
from wake.mapping.voxel_map import SparseVoxelMap

def test_swept_corridor_has_no_gap_between_poses():
    mapping=SparseVoxelMap(.1);count=mark_swept_corridor(mapping,(0,0,0),(1,0,0),drone_radius_m=.12,safety_padding_m=.08,timestamp_ns=1);assert count>30
    for x in range(10):assert mapping.get((x,0,0)) is not None and mapping.get((x,0,0)).occupancy_log_odds<0

def test_corridor_radius_uses_drone_plus_padding():
    small=SparseVoxelMap(.1);large=SparseVoxelMap(.1);mark_swept_corridor(small,(0,0,0),(0,0,0),drone_radius_m=.05,safety_padding_m=0,timestamp_ns=1);mark_swept_corridor(large,(0,0,0),(0,0,0),drone_radius_m=.1,safety_padding_m=.15,timestamp_ns=1);assert len(large.voxels)>len(small.voxels)
