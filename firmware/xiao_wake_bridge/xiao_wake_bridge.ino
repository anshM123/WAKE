/* WAKE v0.2 XIAO -> Matrix MSP telemetry bridge.
 * Matrix retains stabilization. Direct motors remain props-off bench-only.
 */
#include <Arduino.h>
#include "msp.h"
#include "telemetry.h"
#include "networking.h"
#include "bench_safety.h"

constexpr uint32_t FC_BAUD = 115200;
constexpr int FC_RX_PIN = D7, FC_TX_PIN = D6, BENCH_ENABLE_PIN = D3;
constexpr bool ENABLE_BENCH_MOTOR_TESTS = false;
HardwareSerial fcSerial(1); MspPort msp(fcSerial); TelemetryManager telemetry(msp);
WakeNetworking network; BenchSafety bench(msp, BENCH_ENABLE_PIN, ENABLE_BENCH_MOTOR_TESTS);

void setup(){Serial.begin(115200);fcSerial.begin(FC_BAUD,SERIAL_8N1,FC_RX_PIN,FC_TX_PIN);msp.begin();telemetry.begin();network.begin();bench.begin();}
void loop(){const uint32_t nowMs=millis();msp.update();telemetry.update(nowMs);network.update();bench.update(network.commandUdp(),nowMs);if(telemetry.packetReady())network.publishTelemetry(telemetry.takePacket());}
