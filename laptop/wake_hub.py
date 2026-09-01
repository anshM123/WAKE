#!/usr/bin/env python3
"""WAKE laptop hub: telemetry + pose -> shared sparse occupancy hypotheses.

This is a research prototype. It does not receive wall distances or simulation
ground truth; `ResidualSurfaceModel` must be calibrated before flight use.
"""
import argparse, json, math, socket, threading, time
from collections import defaultdict
from pathlib import Path

def rotate(rpy, v):
    r,p,y=rpy; cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return (cy*cp*v[0]+(cy*sp*sr-sy*cr)*v[1]+(cy*sp*cr+sy*sr)*v[2], sy*cp*v[0]+(sy*sp*sr+cy*cr)*v[1]+(sy*sp*cr-cy*sr)*v[2], -sp*v[0]+cp*sr*v[1]+cp*cr*v[2])

class Hub:
    def __init__(self, path): self.pose={}; self.cells=defaultdict(float); self.path=Path(path); self.lock=threading.Lock()
    def ingest(self, m):
        ident=m.get('id')
        if not ident: return
        if m.get('type') == 'pose':
            self.pose[ident]=(tuple(m['position_m']),tuple(m['rpy_rad']))
            return
        if m.get('type') != 'telemetry' or ident not in self.pose: return
        imu,motors=m.get('imu',[]),m.get('motors',[])
        if len(imu)!=6 or len(motors)!=4: return
        # Raw acceleration is in g. This is a deliberately conservative,
        # non-calibrated residual proxy, not an aerodynamic law.
        x,y,z=map(float,imu[:3]); mean=sum(map(float,motors))/4
        z -= 1.0 + max(0,mean-1000)*.00002
        mag=math.sqrt(x*x+y*y+z*z)
        if mag < .18: return
        origin,rpy=self.pose[ident]; d=rotate(rpy,(x/mag,y/mag,z/mag)); distance=max(.10,min(.60,.60-.35*min(mag,1)))
        confidence=min(.35,.12+.18*min(mag,1)); voxel=.10
        with self.lock:
            for i in range(max(1,int(distance/voxel))):
                k=tuple(math.floor((origin[j]+d[j]*i*voxel)/voxel) for j in range(3)); self.cells[k]=max(-4,self.cells[k]-.12*confidence)
            k=tuple(math.floor((origin[j]+d[j]*distance)/voxel) for j in range(3)); self.cells[k]=min(4,self.cells[k]+.65*confidence)
    def save(self):
        while True:
            with self.lock: occupied=[{'ijk':list(k),'probability':round(1/(1+math.exp(-v)),4)} for k,v in self.cells.items() if v>.15]
            tmp=self.path.with_suffix('.tmp'); tmp.write_text(json.dumps({'voxel_m':.10,'occupied':occupied,'drones_with_pose':sorted(self.pose),'generated_unix_s':time.time()},indent=2)); tmp.replace(self.path); time.sleep(1)

def receive(sock,hub):
    while True:
        try: hub.ingest(json.loads(sock.recvfrom(4096)[0]))
        except (ValueError,UnicodeDecodeError,KeyError,TypeError): pass

if __name__ == '__main__':
    a=argparse.ArgumentParser(); a.add_argument('--bind',default='0.0.0.0'); a.add_argument('--telemetry-port',type=int,default=5005); a.add_argument('--pose-port',type=int,default=5006); a.add_argument('--map',default='wake_map.json'); a=a.parse_args(); h=Hub(a.map)
    threading.Thread(target=h.save,daemon=True).start()
    for port in (a.telemetry_port,a.pose_port):
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind((a.bind,port)); threading.Thread(target=receive,args=(s,h),daemon=True).start()
    print(f'WAKE hub on UDP {a.telemetry_port} (telemetry), {a.pose_port} (pose)')
    while True: time.sleep(3600)
