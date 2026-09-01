#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "telemetry.h"
class WakeNetworking{public:void begin();void update();void publishTelemetry(const TelemetryPacket&p);WiFiUDP&commandUdp(){return commandUdp_;}private:WiFiUDP telemetryUdp_,commandUdp_,clockUdp_;IPAddress laptop_;uint32_t lastReconnectMs_=0;void handleClockSync();};
