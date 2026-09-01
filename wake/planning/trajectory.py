"""Known-safe trajectory primitives used for conservative return paths."""
from wake.types import PoseSample
def safe_return_path(trajectory:list[PoseSample])->list[PoseSample]:return list(reversed(trajectory))

def known_safe_grid_path(start:tuple[int,int,int],goal:tuple[int,int,int],safe:set[tuple[int,int,int]])->list[tuple[int,int,int]]|None:
    """Breadth-first path restricted strictly to known-safe voxels."""
    if start not in safe or goal not in safe:return None
    queue=[start];parents={start:None}
    while queue:
        current=queue.pop(0)
        if current==goal:
            path=[]
            while current is not None:path.append(current);current=parents[current]
            return list(reversed(path))
        for axis in range(3):
            for delta in (-1,1):
                neighbor=list(current);neighbor[axis]+=delta;neighbor=tuple(neighbor)
                if neighbor in safe and neighbor not in parents:parents[neighbor]=current;queue.append(neighbor)
    return None
