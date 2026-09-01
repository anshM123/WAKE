from __future__ import annotations
from pathlib import Path
from typing import Iterator
import json,time

def read_jsonl(path:str|Path)->Iterator[dict]:
    with Path(path).open(encoding="utf-8") as handle:
        for line_number,line in enumerate(handle,1):
            if line.strip():
                try:yield json.loads(line)
                except json.JSONDecodeError as exc:raise ValueError(f"invalid JSON at {path}:{line_number}") from exc

def replay_records(path:str|Path,speed:float=0)->Iterator[dict]:
    previous=None
    for record in read_jsonl(path):
        timestamp=record.get("timestamp_ns") or record.get("host_receive_timestamp_ns")
        if speed>0 and previous is not None and timestamp is not None:time.sleep(max(0,(timestamp-previous)/1e9/speed))
        if timestamp is not None:previous=timestamp
        yield record
