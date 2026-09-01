import numpy as np
from wake.mapping.mesh import reconstruct_planar_mesh
from wake.mapping.planes import extract_planes
def test_noisy_wall_with_outliers():
    rng=np.random.default_rng(1);wall=np.column_stack([np.full(200,2.)+rng.normal(0,.005,200),rng.uniform(-1,1,200),rng.uniform(0,2,200)]);outliers=rng.uniform(-2,2,(40,3));planes=extract_planes(np.vstack([wall,outliers]),minimum_support=80,distance_threshold_m=.025);assert planes and abs(abs(planes[0].normal_world[0])-1)<.02 and planes[0].classification=="WALL"
def test_floor_and_wall_are_separate():
    rng=np.random.default_rng(2);floor=np.column_stack([rng.uniform(-1,1,150),rng.uniform(-1,1,150),rng.normal(0,.003,150)]);wall=np.column_stack([rng.normal(0,.003,150),rng.uniform(-1,1,150),rng.uniform(0,2,150)]);planes=extract_planes(np.vstack([floor,wall]),minimum_support=80,distance_threshold_m=.02);assert len(planes)>=2;assert {plane.classification for plane in planes}>={"WALL","FLOOR_OR_CEILING"};assert len(reconstruct_planar_mesh(planes,minimum_confidence=.1).faces)>=4
