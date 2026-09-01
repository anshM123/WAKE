from __future__ import annotations
from dataclasses import asdict,is_dataclass
from datetime import datetime,timezone
from pathlib import Path
from queue import Queue,Full
from threading import Thread
from typing import Any
import json,shutil,subprocess

STREAMS=("telemetry","pose","raw_pose","filtered_pose","clock","clock_exchange","synchronized_samples","preprocessed_features","free_air_prediction","residual","surface_estimate","map_updates","health","events","commands")
def _jsonable(value:Any)->Any:
    if is_dataclass(value):value=asdict(value)
    if isinstance(value,dict):return {k:_jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [_jsonable(v) for v in value]
    if hasattr(value,"tolist"):return value.tolist()
    if hasattr(value,"value"):return value.value
    return value

class SessionRecorder:
    def __init__(self,root:str|Path="data/sessions",queue_size:int=10000)->None:
        stamp=datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S");self.path=Path(root)/stamp;self.path.mkdir(parents=True,exist_ok=False);self.queue:Queue[tuple[str,Any]|None]=Queue(queue_size);self.dropped=0;self._worker=Thread(target=self._write_loop,daemon=True);self._worker.start()
    def start(self,metadata:dict[str,Any],config_paths:list[str|Path]=[])->None:
        enriched={**metadata,"start_time":datetime.now(timezone.utc).isoformat(),"git_commit":self._git_commit()};(self.path/"metadata.json").write_text(json.dumps(_jsonable(enriched),indent=2),encoding="utf-8")
        snapshots=self.path/"config_snapshot";snapshots.mkdir()
        for source in config_paths:shutil.copy2(source,snapshots/Path(source).name)
    def record(self,stream:str,value:Any)->bool:
        if stream not in STREAMS:raise ValueError(f"unknown stream {stream}")
        try:self.queue.put_nowait((stream,value));return True
        except Full:self.dropped+=1;return False
    def close(self)->None:
        self.queue.put(None);self._worker.join();metadata_path=self.path/"metadata.json"
        if metadata_path.exists():
            metadata=json.loads(metadata_path.read_text(encoding="utf-8"));metadata["end_time"]=datetime.now(timezone.utc).isoformat();metadata["dropped_disk_records"]=self.dropped;metadata_path.write_text(json.dumps(metadata,indent=2),encoding="utf-8")
    def _write_loop(self)->None:
        handles={name:(self.path/f"{name}.jsonl").open("a",encoding="utf-8") for name in STREAMS}
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                stream,value=item;handles[stream].write(json.dumps(_jsonable(value),separators=(",",":"))+"\n")
        finally:
            for handle in handles.values():handle.close()
    @staticmethod
    def _git_commit()->str|None:
        try:return subprocess.check_output(["git","rev-parse","HEAD"],text=True,stderr=subprocess.DEVNULL).strip()
        except (OSError,subprocess.CalledProcessError):return None
