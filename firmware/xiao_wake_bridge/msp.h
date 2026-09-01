#pragma once
#include <Arduino.h>
namespace MspCommand{constexpr uint8_t STATUS=101,RAW_IMU=102,MOTOR=104,ATTITUDE=108,ANALOG=110,SET_MOTOR=214;}
struct MspResponse{uint8_t command=0,length=0,payload[64]={};bool valid=false;};
class MspPort{
 public:explicit MspPort(HardwareSerial&s):serial_(s){}void begin();bool request(uint8_t command,const uint8_t*payload=nullptr,uint8_t length=0);void update();bool takeResponse(MspResponse&response);bool busy()const{return awaiting_;}uint32_t timeoutCount()const{return timeoutCount_;}
 private:HardwareSerial&serial_;enum class ParseState:uint8_t{DOLLAR,M,DIRECTION,LENGTH,COMMAND,PAYLOAD,CHECKSUM}state_=ParseState::DOLLAR;MspResponse response_{};uint8_t index_=0,checksum_=0,wanted_=0;bool awaiting_=false;uint32_t requestStartedMs_=0,timeoutCount_=0;void parseByte(uint8_t value);void resetParser();
};
