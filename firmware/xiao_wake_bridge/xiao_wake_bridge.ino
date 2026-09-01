/* Seeed XIAO ESP32-S3 Sense -> Matrix Brushed FC MSP telemetry bridge.
 * The Matrix controls flight; direct motor commands are props-off bench tests.
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <esp_timer.h>

constexpr char WIFI_SSID[]="CHANGE_ME", WIFI_PASSWORD[]="CHANGE_ME", LAPTOP_IP[]="192.168.1.10", DRONE_ID[]="wake-01";
constexpr uint16_t TELEMETRY_PORT=5005, COMMAND_PORT=5006; constexpr uint32_t FC_BAUD=115200;
// Verify pins and 3.3V UART compatibility against your Matrix revision.
constexpr int FC_RX_PIN=D7, FC_TX_PIN=D6, BENCH_ENABLE_PIN=D3;
constexpr bool ENABLE_BENCH_MOTOR_TESTS=false; // must be true AND jumper to GND
constexpr uint8_t RAW_IMU=102, MOTOR=104, ATTITUDE=108, ANALOG=110, SET_MOTOR=214;
HardwareSerial FC(1); WiFiUDP udp; IPAddress laptop;
struct Data { int16_t acc[3]={}, gyro[3]={}, att[3]={}; uint16_t motor[4]={}; uint8_t vbat=0; } data;
uint8_t cs(const uint8_t *p,size_t n){uint8_t v=0;while(n--)v^=*p++;return v;}
void sendMsp(uint8_t command,const uint8_t *p=nullptr,uint8_t n=0){uint8_t h[]={'$','M','<',n,command},b[66]={n,command};if(n)memcpy(b+2,p,n);FC.write(h,5);if(n)FC.write(p,n);FC.write(cs(b,n+2));}
bool readMsp(uint8_t wanted,uint8_t *p,uint8_t &n){uint32_t end=millis()+25;while(millis()<end){if(FC.available()<5){delay(1);continue;}if(FC.read()!='$'||FC.read()!='M'||FC.read()!='>')continue;n=FC.read();uint8_t cmd=FC.read();if(n>64)return false;while(FC.available()<n+1&&millis()<end)delay(1);if(FC.available()<n+1)return false;for(int i=0;i<n;i++)p[i]=FC.read();uint8_t c=FC.read(),b[66]={n,cmd};memcpy(b+2,p,n);return cmd==wanted&&c==cs(b,n+2);}return false;}
int16_t i16(const uint8_t*p){return int16_t(uint16_t(p[0])|uint16_t(p[1])<<8);} uint16_t u16(const uint8_t*p){return uint16_t(p[0])|uint16_t(p[1])<<8;}
void sample(){uint8_t p[64],n;sendMsp(RAW_IMU);if(readMsp(RAW_IMU,p,n)&&n>=12)for(int i=0;i<3;i++){data.acc[i]=i16(p+2*i);data.gyro[i]=i16(p+6+2*i);}sendMsp(ATTITUDE);if(readMsp(ATTITUDE,p,n)&&n>=6)for(int i=0;i<3;i++)data.att[i]=i16(p+2*i);sendMsp(MOTOR);if(readMsp(MOTOR,p,n)&&n>=8)for(int i=0;i<4;i++)data.motor[i]=u16(p+2*i);sendMsp(ANALOG);if(readMsp(ANALOG,p,n)&&n)data.vbat=p[0];}
void publish(){StaticJsonDocument<512>d;d["type"]="telemetry";d["id"]=DRONE_ID;d["t_us"]=(uint64_t)esp_timer_get_time();JsonArray a=d.createNestedArray("imu");for(int i=0;i<3;i++)a.add(data.acc[i]/512.f);for(int i=0;i<3;i++)a.add(data.gyro[i]);constexpr float RAD=.01745329252f;JsonArray q=d.createNestedArray("att");q.add(data.att[0]/10.f*RAD);q.add(data.att[1]/10.f*RAD);q.add(data.att[2]*RAD);JsonArray m=d.createNestedArray("motors");for(auto v:data.motor)m.add(v);d["vbat"]=data.vbat/10.f;char out[512];size_t n=serializeJson(d,out);udp.beginPacket(laptop,TELEMETRY_PORT);udp.write((uint8_t*)out,n);udp.endPacket();}
void bench(){int bytes=udp.parsePacket();if(!bytes)return;char input[256];int n=udp.read(input,255);input[n]=0;StaticJsonDocument<256>d;if(deserializeJson(d,input))return;if(String((const char*)d["type"])!="bench_motor"||String((const char*)d["phrase"])!="PROPS_REMOVED"||!ENABLE_BENCH_MOTOR_TESTS||digitalRead(BENCH_ENABLE_PIN)!=LOW)return;int motor=d["motor"]|-1,pwm=d["pwm"]|0;if(motor<0||motor>3||pwm<1000||pwm>1100)return;uint8_t p[16]={};for(int i=0;i<4;i++){uint16_t v=i==motor?pwm:1000;p[2*i]=v;p[2*i+1]=v>>8;}sendMsp(SET_MOTOR,p,16);}
void setup(){pinMode(BENCH_ENABLE_PIN,INPUT_PULLUP);FC.begin(FC_BAUD,SERIAL_8N1,FC_RX_PIN,FC_TX_PIN);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);while(WiFi.status()!=WL_CONNECTED)delay(250);laptop.fromString(LAPTOP_IP);udp.begin(COMMAND_PORT);}
void loop(){static uint32_t last=0;bench();if(millis()-last>=50){last=millis();sample();publish();}}
