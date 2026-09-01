import pytest
from wake.protocol.messages import telemetry_from_mapping
from wake.protocol.sequence import SequenceTracker

def packet():return {"protocol_version":2,"type":"telemetry","drone_id":"wake-01","sequence":1,"timestamp_us":10,"accel_body_g":[0,0,1],"gyro_body":[0,0,0],"attitude_rpy_rad":[0,0,0],"motors":[1000]*4,"battery_v":4.0,"validity":31}
def test_valid_packet():assert telemetry_from_mapping(packet(),20).motors==(1000.,)*4
def test_missing_fields():
    value=packet();del value["motors"]
    with pytest.raises(ValueError,match="missing"):telemetry_from_mapping(value)
def test_invalid_lengths():
    value=packet();value["motors"]=[1000]*3
    with pytest.raises(ValueError,match="exactly 4"):telemetry_from_mapping(value)
def test_sequence_rollover_and_loss():
    tracker=SequenceTracker();tracker.update(0xFFFFFFFF);assert tracker.update(0).lost==0;assert tracker.update(3).lost==2
def test_reorder():
    tracker=SequenceTracker();tracker.update(10);assert tracker.update(9).reordered
