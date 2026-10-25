// Robot motor controller: the team's 3-wheel omni motion code plus a command receiver.
//
// Commands arrive as text lines over WiFi UDP (port 4210) or USB serial (115200):
//   D <seq> <speed 0..100> <direction deg> <turn -100..100>   drive (direction 0 = the robot's own "dir 0")
//   S                                                         stop
// The motors stop by themselves if no command arrives for 300 ms.
// Every second the robot broadcasts "HELLO <id> <ip>" on UDP port 4211 so the laptop finds it.
//
// move(), MOTORA/B/C and dir_move() are the team's code, unchanged except `ma*2.55` (the `*` was lost
// in copying). drive() is dir_move() with any direction and a turn input instead of 30-degree steps.

#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"

const uint16_t CMD_PORT = 4210;
const uint16_t HELLO_PORT = 4211;
const uint32_t WATCHDOG_MS = 300;

WiFiUDP udp;
uint32_t lastCmdMs = 0;
uint32_t lastHelloMs = 0;
uint32_t wifiLostMs = 0;          // when the WiFi connection was last seen down (0 = connected)
bool stopped = true;
String serialLine;

//Speed in PWM 0~255
int ma, mb, mc, md;

void move(int ma, int mb, int mc, int md)
{
  //PWM scale convert 0~255 to 0~100(approx)
  int va = ma*2.55;
  int vb = mb*2.55;
  int vc = mc*2.55;
  int vd = md*2.55;

  MOTORA(va);
  MOTORB(vc);
  MOTORC(vb);
  //MOTORD(vd);
}

void MOTORA(int ma)
{
  int MA = abs(ma);
  if(ma < 0)
  {
    analogWrite(2,0);
    analogWrite(3,MA);
  }
  else if(ma > 0)
  {
    analogWrite(2,ma);
    analogWrite(3,0);
  }
  else
  {
    analogWrite(2,0);
    analogWrite(3,0);
  }
}

void MOTORB(int mb)
{
int MB = abs(mb);
  if(mb < 0)
  {
    analogWrite(0,0);
    analogWrite(1,MB);
  }
  else if(mb > 0)
  {
    analogWrite(0,mb);
    analogWrite(1,0);
  }
  else
  {
    analogWrite(0,0);
    analogWrite(1,0);
  }
}
void MOTORC(int mc)
{
int MC = abs(mc);
  if(mc < 0)
  {
    analogWrite(8,0);
    analogWrite(9,MC);
  }
  else if(mc > 0)
  {
    analogWrite(8,mc);
    analogWrite(9,0);
  }
  else
  {
    analogWrite(8,0);
    analogWrite(9,0);
  }
}

void dir_move(int dir, int vel)
{
  double Alpha = dir * 30 * PI / 180;
  double ax = vel * cos(Alpha);
  double ay = vel * sin(Alpha);
  double w = 1;//1.5 * (heading-180);

  int F1 = (int)(0.578 * ax - 0.333 * ay + 0.333 * w);
  int F2 = (int)(0 * ax + 0.567 * ay + 0.333 * w);
  int F3 = (int)(-0.578 * ax - 0.333 * ay + 0.333 * w);

  int maxF = max(abs(F1), max(abs(F2), abs(F3)));

  if (maxF > 0) {
    F1 = F1 * vel / maxF;
    F2 = F2 * vel / maxF;
    F3 = F3 * vel / maxF;
  }

  move(F1,F2,F3, 0);

  delay(20);
}

// Same wheel equations as dir_move(); the fastest wheel runs at max(speed, |turn|).
void drive(float speed, float dirDeg, float turn)
{
  speed = constrain(speed, 0, 100);
  turn = constrain(turn, -100, 100);
  double a = dirDeg * PI / 180;
  double ax = speed * cos(a);
  double ay = speed * sin(a);
  double F1 = 0.578 * ax - 0.333 * ay + 0.333 * turn;
  double F2 = 0 * ax + 0.567 * ay + 0.333 * turn;
  double F3 = -0.578 * ax - 0.333 * ay + 0.333 * turn;
  double maxF = max(fabs(F1), max(fabs(F2), fabs(F3)));
  double top = max((double)speed, fabs((double)turn));
  if (maxF > 0) {
    F1 = F1 * top / maxF;
    F2 = F2 * top / maxF;
    F3 = F3 * top / maxF;
  }
  move((int)F1, (int)F2, (int)F3, 0);
  stopped = (top == 0);
}

void stopMotors()
{
  move(0, 0, 0, 0);
  stopped = true;
}

void handleLine(const char *line)
{
  if (line[0] == 'S') {
    stopMotors();
    lastCmdMs = millis();
    return;
  }
  if (line[0] == 'D') {
    unsigned long seq;
    float speed, dirDeg, turn;
    if (sscanf(line + 1, "%lu %f %f %f", &seq, &speed, &dirDeg, &turn) == 4) {
      if (!isfinite(speed) || !isfinite(dirDeg) || !isfinite(turn)) {
        stopMotors();             // "nan" parses as a float; never turn it into wheel speeds
        return;
      }
      drive(speed, dirDeg, turn);
      lastCmdMs = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  stopMotors();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // low latency for drive commands
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  udp.begin(CMD_PORT);
}

void loop()
{
  char buf[96];
  int n = udp.parsePacket();
  if (n > 0) {
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = 0;
      handleLine(buf);
    }
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length()) handleLine(serialLine.c_str());
      serialLine = "";
    } else if (serialLine.length() < 90) {
      serialLine += c;
    }
  }
  uint32_t now = millis();
  if (!stopped && now - lastCmdMs > WATCHDOG_MS) {
    stopMotors();
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiLostMs = 0;
  } else if (wifiLostMs == 0) {
    wifiLostMs = now | 1;
  } else if (now - wifiLostMs > 10000) {
    WiFi.disconnect();            // auto-reconnect gives up after some errors: start over every 10 s
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiLostMs = now | 1;
  }
  if (now - lastHelloMs > 1000) {
    lastHelloMs = now;
    if (WiFi.status() == WL_CONNECTED) {
      String hello = String("HELLO ") + ROBOT_ID + " " + WiFi.localIP().toString();
      udp.beginPacket(IPAddress(255, 255, 255, 255), HELLO_PORT);
      udp.print(hello);
      udp.endPacket();
      Serial.println(hello);
    } else {
      Serial.println("wifi: connecting to " WIFI_SSID);
    }
  }
}
