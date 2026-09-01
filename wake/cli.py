from __future__ import annotations
import argparse,json,logging
from pathlib import Path
from wake import __version__
from wake.config import load_yaml,validate_autonomy
from wake.mapping.export import export_json,export_ply
from wake.mapping.voxel_map import SparseVoxelMap
from wake.recording.replay import replay_records
from wake.calibration.camera import calibrate_camera
from wake.pose.apriltag_pose import AprilTagPoseProvider
from wake.pose.transforms import quaternion_to_matrix
import math

def main(argv:list[str]|None=None)->int:
    parser=argparse.ArgumentParser(prog="wake",description="WAKE v0.2 research platform");parser.add_argument("--version",action="version",version=__version__);sub=parser.add_subparsers(dest="command",required=True)
    replay=sub.add_parser("replay");replay.add_argument("session");replay.add_argument("--speed",type=float,default=0,help="0=max, or 0.1/1/10")
    camera_cal=sub.add_parser("calibrate-camera");camera_cal.add_argument("--config",default="config/camera.yaml");camera_cal.add_argument("--columns",type=int,default=9);camera_cal.add_argument("--rows",type=int,default=6);camera_cal.add_argument("--square-size-m",type=float,default=.024);camera_cal.add_argument("--images",type=int,default=20)
    tag_test=sub.add_parser("apriltag-test");tag_test.add_argument("--config",default="config/camera.yaml")
    record=sub.add_parser("record");record.add_argument("--config",default="config/wake.yaml")
    sub.add_parser("map");sub.add_parser("calibrate-free-air");sub.add_parser("calibrate-wall");sub.add_parser("train");sub.add_parser("evaluate-model");sub.add_parser("inspect-health")
    export=sub.add_parser("export-map");export.add_argument("map_json");export.add_argument("--ply",required=True)
    args=parser.parse_args(argv);logging.basicConfig(level=logging.INFO)
    if args.command=="calibrate-camera":
        config=load_yaml(args.config);camera=config["camera"]
        output=calibrate_camera(device=int(camera["device"]),width=int(camera["width"]),height=int(camera["height"]),output=camera["calibration_file"],columns=args.columns,rows=args.rows,square_size_m=args.square_size_m,required_images=args.images,camera_identifier=f"device-{camera['device']}")
        print(output);return 0
    if args.command=="apriltag-test":
        return _run_apriltag_test(load_yaml(args.config))
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

def _run_apriltag_test(config:dict)->int:
    import cv2
    provider=AprilTagPoseProvider(config)
    try:
        while True:
            frame,pose=provider.capture_frame();metrics=provider.metrics
            lines=[f"Tag {config['tag']['id']}: {'GOOD' if pose else metrics.failure}",f"reprojection {metrics.reprojection_error_px:.2f} px  width {metrics.tag_pixel_width:.0f} px"]
            if pose:
                matrix=quaternion_to_matrix(pose.rotation_world_from_body);pitch=math.asin(max(-1.,min(1.,-matrix[2,0])));roll=math.atan2(matrix[2,1],matrix[2,2]);yaw=math.atan2(matrix[1,0],matrix[0,0]);x,y,z=pose.position_world_m;lines.extend([f"XYZ {x:.3f} {y:.3f} {z:.3f} m",f"RPY {math.degrees(roll):.1f} {math.degrees(pitch):.1f} {math.degrees(yaw):.1f} deg",f"height below ceiling tag {abs(config['transforms']['T_world_from_tag'][2][3]-z):.3f} m"])
            for index,line in enumerate(lines):cv2.putText(frame,line,(20,35+28*index),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0) if pose else (0,0,255),2)
            cv2.imshow("WAKE AprilTag test - ESC to exit",frame)
            if cv2.waitKey(1)==27:break
    finally:
        provider.close();cv2.destroyAllWindows()
    return 0


if __name__=="__main__":raise SystemExit(main())
