"""Operational command-line interface for WAKE acquisition and research."""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import argparse
import json
import logging
import math
import time
import numpy as np
import yaml

from wake import __version__
from wake.calibration.camera import calibrate_camera
from wake.calibration.labels import WorldPlane, plane_from_points, save_reference_plane, wall_geometry_label
from wake.calibration.train import save_artifact, train_linear_free_air
from wake.config import _distance_gated_recall,config_hash, load_yaml
from wake.estimation.features import instantaneous_features
from wake.estimation.motion import PoseMotionEstimator
from wake.estimation.free_air import LearnedFreeAirModel
from wake.estimation.residual import ResidualEstimator, residual_feature_vector
from wake.estimation.surface_model import CalibratedSurfaceModel, save_surface_artifact, train_surface_model
from wake.mapping.export import export_planes, export_ply
from wake.mapping.mesh import export_obj, reconstruct_planar_mesh
from wake.mapping.planes import extract_planes
from wake.mapping.voxel_map import SparseVoxelMap
from wake.pose.apriltag_pose import AprilTagPoseProvider
from wake.pose.transforms import quaternion_to_matrix, quaternion_from_rpy
from wake.recording.dataset import load_synchronized_rows, load_synchronized_session, split_sessions
from wake.recording.replay import ReplayPipeline
from wake.runtime import WakeRuntime
from wake.telemetry.clock_sync import ClockModel, ClockSynchronizer
from wake.visualization.live import LiveMapViewer

LOG = logging.getLogger(__name__)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="wake", description="WAKE v0.3 experimental mapping platform")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)
    camera = sub.add_parser("calibrate-camera"); camera.add_argument("--config", default="config/camera.yaml"); camera.add_argument("--columns", type=int, default=9); camera.add_argument("--rows", type=int, default=6); camera.add_argument("--square-size-m", type=float, default=.024); camera.add_argument("--images", type=int, default=20)
    tag = sub.add_parser("apriltag-test"); tag.add_argument("--config", default="config/camera.yaml"); tag.add_argument("--duration", type=float, default=0)
    transform = sub.add_parser("calibrate-transforms"); transform.add_argument("--config", default="config/camera.yaml"); transform.add_argument("--ceiling-height-m", type=float, required=True); transform.add_argument("--tag-yaw-deg", type=float, required=True); transform.add_argument("--camera-position-body-m", nargs=3, type=float, required=True); transform.add_argument("--camera-rpy-body-deg", nargs=3, type=float, required=True); transform.add_argument("--confirm-frame-check", action="store_true")
    clock = sub.add_parser("clock-test"); clock.add_argument("--xiao-host", required=True); clock.add_argument("--config", default="config/wake.yaml"); clock.add_argument("--duration", type=float, default=10)
    for name in ("record", "live", "map", "calibrate-free-air", "calibrate-wall", "inspect-health"):
        command = sub.add_parser(name); command.add_argument("--config", default="config/wake.yaml"); command.add_argument("--xiao-host", default="192.168.1.2"); command.add_argument("--duration", type=float, default=0)
    define = sub.add_parser("define-plane"); define.add_argument("--output", default="config/reference_plane.yaml"); define.add_argument("--name", default="reference_wall"); group = define.add_mutually_exclusive_group(required=True); group.add_argument("--points", nargs=9, type=float); group.add_argument("--normal", nargs=3, type=float); define.add_argument("--offset-m", type=float)
    train_free = sub.add_parser("train-free-air"); train_free.add_argument("sessions", nargs="+"); train_free.add_argument("--output", default="models/free_air.json"); train_free.add_argument("--config", default="config/wake.yaml"); train_free.add_argument("--window-size", type=int, default=10)
    train_surface = sub.add_parser("train-surface"); train_surface.add_argument("--wall-sessions", nargs="+", required=True); train_surface.add_argument("--negative-sessions", nargs="+", required=True); train_surface.add_argument("--free-air-model", required=True); train_surface.add_argument("--plane", default="config/reference_plane.yaml"); train_surface.add_argument("--output", default="models/surface.json"); train_surface.add_argument("--config", default="config/wake.yaml"); train_surface.add_argument("--maximum-wall-distance-m", type=float, default=1.0)
    evaluate = sub.add_parser("evaluate-model"); evaluate.add_argument("artifact")
    replay = sub.add_parser("replay"); replay.add_argument("session"); replay.add_argument("--config", default="config/wake.yaml"); replay.add_argument("--speed", type=float, default=0); replay.add_argument("--output")
    export = sub.add_parser("export-map"); export.add_argument("map_json"); export.add_argument("--output-prefix", default="wake_export"); export.add_argument("--occupied-threshold", type=float, default=.7)
    map_eval = sub.add_parser("evaluate-map"); map_eval.add_argument("planes_json"); map_eval.add_argument("--reference-plane", default="config/reference_plane.yaml")
    sub.add_parser("simulate")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    handlers = {
        "calibrate-camera": _camera_calibration, "apriltag-test": _apriltag_test,
        "calibrate-transforms": _calibrate_transforms, "clock-test": _clock_test,
        "record": lambda value: _run_live(value, camera=False, viewer=False),
        "live": lambda value: _run_live(value, camera=True, viewer=True),
        "map": lambda value: _run_live(value, camera=True, viewer=False),
        "calibrate-free-air": lambda value: _run_live(value, camera=True, viewer=False),
        "calibrate-wall": _calibrate_wall, "inspect-health": _inspect_health,
        "define-plane": _define_plane, "train-free-air": _train_free_air,
        "train-surface": _train_surface, "evaluate-model": _evaluate_model,
        "replay": _replay, "export-map": _export_map, "evaluate-map": _evaluate_map,
        "simulate": _simulate,
    }
    return handlers[args.command](args)


def _camera_calibration(args) -> int:
    config = load_yaml(args.config); camera = config["camera"]
    output = calibrate_camera(device=int(camera["device"]), width=int(camera["width"]), height=int(camera["height"]), output=camera["calibration_file"], columns=args.columns, rows=args.rows, square_size_m=args.square_size_m, required_images=args.images, camera_identifier=f"device-{camera['device']}")
    print(f"Saved camera calibration: {output}"); return 0


def _apriltag_test(args) -> int:
    import cv2
    config = load_yaml(args.config); provider = AprilTagPoseProvider(config); started = time.monotonic(); frames = detected = 0
    try:
        while args.duration <= 0 or time.monotonic() - started < args.duration:
            frame, pose = provider.capture_frame(); frames += 1; detected += int(pose is not None); metrics = provider.metrics
            lines = [f"Tag {config['tag']['id']}: {'GOOD' if pose else metrics.failure}", f"reprojection {metrics.reprojection_error_px:.2f} px  width {metrics.tag_pixel_width:.0f} px"]
            if pose:
                rotation = quaternion_to_matrix(pose.rotation_world_from_body); pitch = math.asin(max(-1., min(1., -rotation[2, 0]))); roll = math.atan2(rotation[2, 1], rotation[2, 2]); yaw = math.atan2(rotation[1, 0], rotation[0, 0]); x, y, z = pose.position_world_m; tag_height = config["transforms"]["T_world_from_tag"][2][3]
                lines.extend([f"XYZ {x:.3f} {y:.3f} {z:.3f} m", f"RPY {math.degrees(roll):.1f} {math.degrees(pitch):.1f} {math.degrees(yaw):.1f} deg", f"height below ceiling tag {abs(tag_height-z):.3f} m"])
            for index, line in enumerate(lines): cv2.putText(frame, line, (20, 35 + 28 * index), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0) if pose else (0, 0, 255), 2)
            cv2.imshow("WAKE AprilTag coverage test - ESC to exit", frame)
            if cv2.waitKey(1) == 27: break
    finally:
        provider.close(); cv2.destroyAllWindows()
    coverage = detected / max(1, frames); print(json.dumps({"frames": frames, "tracked_fraction": coverage, "geofence_validated": coverage >= .99}, indent=2)); return 0 if coverage >= .99 else 2


def _calibrate_transforms(args) -> int:
    config = load_yaml(args.config); yaw = math.radians(args.tag_yaw_deg)
    world_from_tag = np.eye(4); world_from_tag[:3, :3] = quaternion_to_matrix(quaternion_from_rpy(math.pi, 0, yaw)); world_from_tag[:3, 3] = [0, 0, args.ceiling_height_m]
    roll, pitch, camera_yaw = np.radians(args.camera_rpy_body_deg); body_from_camera = np.eye(4); body_from_camera[:3, :3] = quaternion_to_matrix(quaternion_from_rpy(roll, pitch, camera_yaw)); body_from_camera[:3, 3] = args.camera_position_body_m
    config["transforms"]["T_world_from_tag"] = world_from_tag.tolist(); config["transforms"]["T_body_from_camera"] = body_from_camera.tolist(); config["frame_check_confirmed"] = bool(args.confirm_frame_check)
    Path(args.config).write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    print("Transforms saved. Verify +X, +Y, +Z and yaw motion; mapping remains blocked until --confirm-frame-check is used."); return 0


def _clock_test(args) -> int:
    config = load_yaml(args.config); sync = config["synchronization"]; client = ClockSynchronizer(args.xiao_host, config["network"]["clock_port"], ClockModel(max_rtt_ms=sync["clock_max_rtt_ms"], stale_after_ms=sync["max_clock_model_age_ms"])); started = time.monotonic(); consecutive = 0
    try:
        while time.monotonic() - started < args.duration:
            client.exchange_once(); status = client.model.status(); good = client.model.healthy() and status.confidence >= .5; consecutive = consecutive + 1 if good else 0; print(f"RTT {status.rtt_ms:.2f} ms  offset {status.offset_ns:.0f} ns  skew {status.skew_ppm:.2f} ppm  age {status.model_age_ms:.0f} ms  residual {status.residual_ms:.3f} ms  confidence {status.confidence:.2f}"); time.sleep(.5)
    finally: client.close()
    declared = consecutive >= 5; print("CLOCK SYNC GOOD" if declared else "CLOCK SYNC BAD"); return 0 if declared else 2


def _run_live(args, *, camera: bool, viewer: bool) -> int:
    runtime = WakeRuntime(args.config, xiao_host=args.xiao_host, enable_camera=camera)
    mode_by_command={"record":"RECORD_ONLY","live":"MAPPING_MANUAL_FLIGHT","map":"MAPPING_MANUAL_FLIGHT","calibrate-free-air":"CALIBRATION_FREE_AIR","calibrate-wall":"CALIBRATION_WALL"}
    runtime.config["mode"]=mode_by_command.get(args.command,runtime.config["mode"])
    runtime.start(); print(f"Session: {runtime.recorder.path}")
    try:
        if viewer:
            ui = runtime.config["ui"]; LiveMapViewer(runtime.ui_queue, voxel_size_m=runtime.voxel_map.voxel_size_m, update_hz=ui["update_hz"], occupancy_threshold=ui["occupancy_threshold"], confidence_threshold=ui["confidence_threshold"]).run()
        else:
            started = time.monotonic()
            while args.duration <= 0 or time.monotonic() - started < args.duration: time.sleep(.25)
    except KeyboardInterrupt: pass
    finally: runtime.stop()
    return 0


def _inspect_health(args) -> int:
    runtime = WakeRuntime(args.config, xiao_host=args.xiao_host, enable_camera=True); runtime.start(); started = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - started < args.duration: print(json.dumps(asdict(runtime.health), indent=2)); time.sleep(1)
    except KeyboardInterrupt: pass
    finally: runtime.stop()
    return 0


def _define_plane(args) -> int:
    if args.points:
        values = [tuple(args.points[index:index+3]) for index in range(0, 9, 3)]; plane = plane_from_points(*values)
    else:
        if args.offset_m is None: raise SystemExit("--offset-m is required with --normal")
        normal = np.asarray(args.normal, float); normal /= np.linalg.norm(normal); plane = WorldPlane(tuple(normal.tolist()), args.offset_m / float(np.linalg.norm(np.asarray(args.normal))))
    print(save_reference_plane(plane, args.output, args.name)); return 0


def _calibrate_wall(args) -> int:
    raw = load_yaml("config/reference_plane.yaml")["reference_plane"]
    if raw["normal_world"] is None: raise SystemExit("define a measured plane with `wake define-plane` first")
    return _run_live(args, camera=True, viewer=False)


def _train_free_air(args) -> int:
    paths = [Path(path) for path in args.sessions]; split = split_sessions(paths); by_name = {path.name:path for path in paths}
    def combine(names):
        values=[load_synchronized_session(by_name[name],args.window_size) for name in names];return np.concatenate([v[0] for v in values]),np.concatenate([v[1] for v in values])
    train_x,train_y=combine(split.train_session_ids);validation_x,validation_y=combine(split.validation_session_ids);feature_names=[f"temporal_feature_{index}" for index in range(train_x.shape[1])];cfg=load_yaml(args.config);artifact=train_linear_free_air(train_x,train_y,feature_names=feature_names,dataset_ids=[path.name for path in paths],configuration_hash=config_hash(cfg),validation_features=validation_x,validation_targets=validation_y,split_session_ids=asdict(split));save_artifact(artifact,args.output);print(json.dumps(artifact.validation_metrics,indent=2));print(f"Reserved test sessions: {split.test_session_ids}");return 0


def _train_surface(args) -> int:
    cfg=load_yaml(args.config);free_air=LearnedFreeAirModel.load(args.free_air_model);plane_raw=load_yaml(args.plane)["reference_plane"];plane=WorldPlane(tuple(plane_raw["normal_world"]),float(plane_raw["offset_m"]));wall_paths=[Path(path) for path in args.wall_sessions];negative_paths=[Path(path) for path in args.negative_sessions];all_paths=wall_paths+negative_paths;wall_names={path.name for path in wall_paths};wall_split=split_sessions(wall_paths);negative_split=split_sessions(negative_paths);split=type(wall_split)(wall_split.train_session_ids+negative_split.train_session_ids,wall_split.validation_session_ids+negative_split.validation_session_ids,wall_split.test_session_ids+negative_split.test_session_ids);by_name={path.name:path for path in all_paths}
    def build(names):
        all_features=[];nearby=[];distances=[];directions=[];normals=[]
        for name in names:
            estimator=ResidualEstimator(cfg["filtering"]["persistence_samples"]);motion=PoseMotionEstimator();history=[]
            for row in load_synchronized_rows(by_name[name]):
                base=instantaneous_features(row,motion.update(row.pose));history=(history+[base])[-10:];window=np.asarray(history);features=np.concatenate([window[-1],window.mean(0),window.std(0),window[-1]-window[0]]);expected=free_air.predict(features);observed=np.asarray([*row.telemetry.accel_body_g,*row.telemetry.gyro_body]);residual=estimator.calculate(observed,expected,np.asarray(row.telemetry.motors));all_features.append(residual_feature_vector(residual))
                if name in wall_names:
                    distance,direction,normal=wall_geometry_label(row.pose,plane);is_near=distance<=args.maximum_wall_distance_m
                else:distance,direction,normal,is_near=args.maximum_wall_distance_m,(1.,0.,0.),(1.,0.,0.),False
                nearby.append(float(is_near));distances.append(distance);directions.append(direction);normals.append(normal)
        return np.asarray(all_features),np.asarray(nearby),np.asarray(distances),np.asarray(directions),np.asarray(normals)
    train=build(split.train_session_ids);validation=build(split.validation_session_ids);artifact=train_surface_model(*train,validation=validation,dataset_ids=[p.name for p in all_paths],split_session_ids=asdict(split),configuration_hash=config_hash(cfg,plane_raw));save_surface_artifact(artifact,args.output);print(json.dumps(artifact.validation_metrics,indent=2));return 0


def _evaluate_model(args) -> int:
    raw=json.loads(Path(args.artifact).read_text());metrics=raw.get("validation_metrics",{});print(json.dumps(metrics,indent=2));scope=metrics.get("validation_scope");safety=load_yaml("config/safety.yaml")
    if "recall_by_distance" in metrics:
        recall=_distance_gated_recall(metrics,safety.get("caution_distance_m"));acceptable=recall is not None and recall>=safety["detection_recall_min"];detail="caution-zone recall unavailable" if recall is None else f"caution-zone recall {recall:.3f}"
    else:acceptable=scope=="held-out-sessions";detail=str(scope)
    print(("AUTONOMY MODEL GATE: PASS — " if acceptable else "AUTONOMY MODEL GATE: BLOCKED — ")+detail);return 0 if acceptable else 2


def _replay(args) -> int:
    config=load_yaml(args.config);config["safety"]=load_yaml("config/safety.yaml");result=ReplayPipeline(args.session,config).run(args.output,args.speed);print(json.dumps({**asdict(result),"output_path":str(result.output_path)},indent=2));return 0


def _load_voxel_map(path:str|Path)->SparseVoxelMap:
    raw=json.loads(Path(path).read_text());mapping=SparseVoxelMap(raw["voxel_size_m"])
    for item in raw["voxels"]:
        probability=float(item["occupancy_probability"]);log_odds=math.log(max(1e-9,probability)/max(1e-9,1-probability));voxel=mapping.update(tuple(item["ijk"]),log_odds,float(item.get("confidence",0)),int(item.get("last_update_time_ns",1)));voxel.observation_count=int(item.get("observation_count",1))
    return mapping


def _export_map(args) -> int:
    mapping=_load_voxel_map(args.map_json);prefix=Path(args.output_prefix);export_ply(mapping,prefix.with_suffix(".ply"),args.occupied_threshold);points=[];confidence=[]
    for index,voxel in mapping.voxels.items():
        if voxel.occupancy_probability>=args.occupied_threshold:points.append(tuple((axis+.5)*mapping.voxel_size_m for axis in index));confidence.append(voxel.confidence)
    planes=extract_planes(np.asarray(points),confidences=np.asarray(confidence),minimum_support=30,distance_threshold_m=mapping.voxel_size_m*1.5) if len(points)>=30 else [];export_planes(planes,prefix.with_name(prefix.name+"_planes.json"));export_obj(reconstruct_planar_mesh(planes,.6),prefix.with_suffix(".obj"));print(f"Exported {prefix}.ply, {prefix}_planes.json, {prefix}.obj");return 0


def _evaluate_map(args) -> int:
    planes=json.loads(Path(args.planes_json).read_text());reference=load_yaml(args.reference_plane)["reference_plane"];normal=np.asarray(reference["normal_world"],float);offset=float(reference["offset_m"])
    if not planes:raise SystemExit("no fitted planes to evaluate")
    best=min(planes,key=lambda plane:min(np.linalg.norm(np.asarray(plane["normal_world"])-normal),np.linalg.norm(np.asarray(plane["normal_world"])+normal)));estimated_normal=np.asarray(best["normal_world"]);alignment=abs(float(estimated_normal@normal));report={"wall_plane_position_error_m":abs(abs(float(best["offset_m"]))-abs(offset)),"wall_normal_error_deg":math.degrees(math.acos(np.clip(alignment,-1,1))),"support_count":best["support_count"],"plane_confidence":best["confidence"]};print(json.dumps(report,indent=2));return 0


def _simulate(args) -> int:
    from wake.simulation import BoxRoom
    room=BoxRoom();trajectory=room.trajectory();estimates=[room.nearest_surface(p.position_world_m,.03) for p in trajectory];print(json.dumps({"poses":len(trajectory),"mean_range_m":float(np.mean([e.distance_m for e in estimates])),"warning":"software simulation only; does not validate wake physics"},indent=2));return 0


if __name__ == "__main__":
    raise SystemExit(main())
