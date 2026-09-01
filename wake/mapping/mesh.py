"""Evidence-supported planar reconstruction; no unseen closure."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from wake.mapping.planes import Plane
from wake.types import Vec3
@dataclass(frozen=True)
class SurfaceMesh:vertices:tuple[Vec3,...];faces:tuple[tuple[int,int,int],...]
def reconstruct_planar_mesh(planes:list[Plane],minimum_confidence:float=.6)->SurfaceMesh:
    vertices=[];faces=[]
    for plane in planes:
        if plane.confidence<minimum_confidence or plane.corners_world is None:continue
        start=len(vertices);vertices.extend(plane.corners_world);faces.extend(((start,start+1,start+2),(start,start+2,start+3)))
    return SurfaceMesh(tuple(vertices),tuple(faces))
def export_obj(mesh:SurfaceMesh,path:str|Path)->Path:
    target=Path(path);lines=[f"v {x} {y} {z}\n" for x,y,z in mesh.vertices];lines.extend(f"f {a+1} {b+1} {c+1}\n" for a,b,c in mesh.faces);target.write_text("".join(lines),encoding="utf-8");return target
