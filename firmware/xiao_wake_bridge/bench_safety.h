#pragma once
#include <WiFiUdp.h>
#include "msp.h"
class BenchSafety{public:BenchSafety(MspPort&msp,int jumperPin,bool enabled):msp_(msp),jumperPin_(jumperPin),compiledEnabled_(enabled){}void begin();void update(WiFiUDP&udp,uint32_t nowMs);private:MspPort&msp_;int jumperPin_;bool compiledEnabled_,active_=false;uint32_t lastCommandMs_=0,lastSequence_=0;static constexpr uint32_t WATCHDOG_MS=250;void sendMinimum();void sendMotor(uint8_t motor,uint16_t pwm);};
