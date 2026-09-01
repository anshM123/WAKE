#pragma once
#include <Arduino.h>
#include "msp.h"
constexpr uint8_t VALID_ACCEL=1,VALID_GYRO=2,VALID_ATTITUDE=4,VALID_MOTORS=8,VALID_BATTERY=16;
struct TelemetryPacket{uint32_t sequence=0;uint64_t timestampUs=0,imuTimestampUs=0,motorsTimestampUs=0,attitudeTimestampUs=0,batteryTimestampUs=0;float accelG[3]={},gyro[3]={},attitudeRad[3]={};uint16_t motors[4]={};float batteryV=0;uint8_t validity=0;float packetHz=0,imuHz=0,motorHz=0,attitudeHz=0,batteryHz=0;uint32_t mspTimeouts=0;};
class TelemetryManager{public:explicit TelemetryManager(MspPort&msp):msp_(msp){}void begin();void update(uint32_t nowMs);bool packetReady()const{return ready_;}TelemetryPacket takePacket(){ready_=false;return packet_;}private:MspPort&msp_;TelemetryPacket packet_{};bool ready_=false;uint8_t phase_=0;uint32_t nextDueMs_=0,lastPublishMs_=0,rateWindowMs_=0;uint32_t packetCount_=0,imuCount_=0,motorCount_=0,attitudeCount_=0,batteryCount_=0;void consume(const MspResponse&r);void schedule(uint32_t nowMs);static int16_t i16(const uint8_t*p);static uint16_t u16(const uint8_t*p);};
