from wake.mapping.voxel_map import SparseVoxelMap
def test_repeated_evidence_increases_occupancy():
    m=SparseVoxelMap();a=m.update((0,0,0),.5,1).occupancy_probability;b=m.update((0,0,0),.5,1).occupancy_probability;assert b>a
def test_conflict_lowers_confidence():
    m=SparseVoxelMap();m.update((0,0,0),1,1);before=m.get((0,0,0)).confidence;m.update((0,0,0),-1,1);assert m.get((0,0,0)).confidence<before
def test_unknown_remains_unknown():assert SparseVoxelMap().get((2,2,2)) is None
def test_negative_indexing():assert SparseVoxelMap(.1).index((-.01,-.1,-.11))==(-1,-1,-2)
