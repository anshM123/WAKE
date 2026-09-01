#!/usr/bin/env python3
"""Validated telemetry recorder entry point; mapping is replayable offline."""
from __future__ import annotations
import argparse,logging
from wake.config import config_hash,load_yaml
from wake.recording.recorder import SessionRecorder
from wake.telemetry.receiver import TelemetryReceiver

def main()->None:
    parser=argparse.ArgumentParser();parser.add_argument("--config",default="config/wake.yaml");args=parser.parse_args();cfg=load_yaml(args.config);network=cfg["network"]
    safety=load_yaml("config/safety.yaml");calibration=load_yaml("config/calibration.yaml");camera=load_yaml("config/camera.yaml")
    metadata={"software_version":"0.2.0","firmware_version":"0.2.0","model_version":calibration.get("status","UNCALIBRATED"),"drone_id":cfg["drone_id"],"tag_id":camera["tag"]["id"],"tag_size_m":camera["tag"]["size_m"],"camera_calibration_hash":config_hash(camera),"config_hash":config_hash(cfg,safety,calibration,camera),"mode":cfg["mode"]}
    recorder=SessionRecorder(cfg["recording"]["root"],cfg["recording"]["queue_size"]);recorder.start(metadata,[args.config,"config/safety.yaml","config/calibration.yaml","config/camera.yaml"]);receiver=TelemetryReceiver(network["bind"],network["telemetry_port"])
    logging.info("Recording validated telemetry to %s",recorder.path)
    try:
        while True:
            sample=receiver.receive_once()
            if sample is not None:recorder.record("telemetry",sample)
    except KeyboardInterrupt:pass
    finally:recorder.close()
if __name__=="__main__":logging.basicConfig(level=logging.INFO);main()
