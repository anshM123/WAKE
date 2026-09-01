from __future__ import annotations
import argparse,json,logging
from pathlib import Path
from wake import __version__
from wake.config import load_yaml,validate_autonomy
from wake.mapping.export import export_json,export_ply
from wake.mapping.voxel_map import SparseVoxelMap
from wake.recording.replay import replay_records

def main(argv:list[str]|None=None)->int:
    parser=argparse.ArgumentParser(prog="wake",description="WAKE v0.2 research platform");parser.add_argument("--version",action="version",version=__version__);sub=parser.add_subparsers(dest="command",required=True)
    replay=sub.add_parser("replay");replay.add_argument("session");replay.add_argument("--speed",type=float,default=0,help="0=max, or 0.1/1/10")
    record=sub.add_parser("record");record.add_argument("--config",default="config/wake.yaml")
    sub.add_parser("map");sub.add_parser("calibrate-free-air");sub.add_parser("calibrate-wall");sub.add_parser("train");sub.add_parser("evaluate-model");sub.add_parser("inspect-health")
    export=sub.add_parser("export-map");export.add_argument("map_json");export.add_argument("--ply",required=True)
    args=parser.parse_args(argv);logging.basicConfig(level=logging.INFO)
    if args.command=="replay":
        session=Path(args.session);source=session/"telemetry.jsonl" if session.is_dir() else session;count=sum(1 for _ in replay_records(source,args.speed));print(json.dumps({"records_replayed":count,"source":str(source)}));return 0
    if args.command=="record":
        cfg=load_yaml(args.config)
        if cfg.get("mode")=="AUTONOMOUS":
            reasons=validate_autonomy(cfg,load_yaml("config/safety.yaml"))
            if reasons:raise SystemExit("AUTONOMOUS blocked: "+"; ".join(reasons))
        print("record command validates configuration; use scripts/run_hub.py for UDP acquisition");return 0
    if args.command=="export-map":
        raw=json.loads(Path(args.map_json).read_text());m=SparseVoxelMap(raw["voxel_size_m"])
        for item in raw["voxels"]:m.update(tuple(item["ijk"]),float(item.get("occupancy_log_odds",0)),float(item.get("confidence",0)))
        export_ply(m,args.ply);return 0
    print(f"{args.command}: interface ready; requires recorded calibration data or hardware configuration");return 0

if __name__=="__main__":raise SystemExit(main())
