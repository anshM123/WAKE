"""The only location for coordinate transforms and quaternion math."""
from __future__ import annotations
import math
import numpy as np
from wake.types import Quaternion, Vec3

def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2); cp, sp = math.cos(pitch / 2), math.sin(pitch / 2); cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)

def quaternion_to_matrix(q: Quaternion) -> np.ndarray:
    qv = np.asarray(q, dtype=float); qv /= np.linalg.norm(qv); w, x, y, z = qv
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def matrix_to_quaternion(matrix: np.ndarray) -> Quaternion:
    m = np.asarray(matrix, dtype=float)[:3, :3]; trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2; q = (0.25*s, (m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s)
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s=math.sqrt(1+m[0,0]-m[1,1]-m[2,2])*2; q=((m[2,1]-m[1,2])/s,.25*s,(m[0,1]+m[1,0])/s,(m[0,2]+m[2,0])/s)
        elif i == 1:
            s=math.sqrt(1+m[1,1]-m[0,0]-m[2,2])*2; q=((m[0,2]-m[2,0])/s,(m[0,1]+m[1,0])/s,.25*s,(m[1,2]+m[2,1])/s)
        else:
            s=math.sqrt(1+m[2,2]-m[0,0]-m[1,1])*2; q=((m[1,0]-m[0,1])/s,(m[0,2]+m[2,0])/s,(m[1,2]+m[2,1])/s,.25*s)
    qv=np.asarray(q); qv/=np.linalg.norm(qv); return tuple(qv.tolist())

def slerp(q0: Quaternion, q1: Quaternion, fraction: float) -> Quaternion:
    a, b = np.asarray(q0, float), np.asarray(q1, float); dot = float(np.dot(a, b))
    if dot < 0: b, dot = -b, -dot
    if dot > .9995: out = a + fraction*(b-a); out /= np.linalg.norm(out); return tuple(out.tolist())
    theta = math.acos(max(-1., min(1., dot))); out = math.sin((1-fraction)*theta)/math.sin(theta)*a + math.sin(fraction*theta)/math.sin(theta)*b
    return tuple(out.tolist())

def transform_point(T_a_from_b: np.ndarray, point_b: Vec3) -> Vec3:
    value = np.asarray(T_a_from_b, float) @ np.array([*point_b, 1.0]); return tuple(value[:3].tolist())

def compose_body_pose(T_world_from_camera: np.ndarray, T_camera_from_tag: np.ndarray, T_tag_from_body: np.ndarray) -> tuple[Vec3, Quaternion]:
    T = np.asarray(T_world_from_camera) @ np.asarray(T_camera_from_tag) @ np.asarray(T_tag_from_body)
    return tuple(T[:3, 3].tolist()), matrix_to_quaternion(T[:3, :3])
