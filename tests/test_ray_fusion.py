from wake.mapping.evidence import MapEvidence
from wake.mapping.ray_fusion import fuse_surface_evidence
from wake.mapping.voxel_map import SparseVoxelMap
def test_angular_uncertainty_spreads_evidence():
    narrow=SparseVoxelMap(.05);wide=SparseVoxelMap(.05);base=dict(origin_world_m=(0,0,0),direction_world=(1,0,0),distance_m=.5,distance_sigma_m=.03,confidence=.9);fuse_surface_evidence(narrow,MapEvidence(**base,angular_sigma_rad=0));fuse_surface_evidence(wide,MapEvidence(**base,angular_sigma_rad=.3));assert len(wide.voxels)>len(narrow.voxels)
