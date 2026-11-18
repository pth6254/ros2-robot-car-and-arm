/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/

/*
 * 로봇팔 디지털 트윈 펌웨어
 *
 * [PC -> Arduino] 명령 : "a90b90c90d90e"   (a=base, b=shoulder, c=upperarm, d=lowerarm, 단위 deg)
 * [Arduino -> PC] 상태 : "s90.0,90.0,90.0,90.0\n"  (STATE_PERIOD_MS 마다, 현재 서보 위치)
 *                 시작 : "Robot Arm Ready ..."
 *
 * 서보는 목표 각도로 한 번에 점프하지 않고 MAX_SPEED_DEG_S 속도로 조금씩 이동한다.
 *  - USB 전원만 쓸 때 순간 전류(리셋/떨림)를 줄여 준다.
 *  - SG90은 위치 센서가 없으므로, 이렇게 직접 움직인 위치를 "실제 상태"로 보고한다.
 *
 * 마지막 자세를 EEPROM에 저장하고, 부팅 시 그 자세 그대로 서보를 켠다.
 *  - 부팅/리셋 때 팔이 90도로 튀지 않는다 (USB 전원 brownout 리셋 방지)
 *  - 팔이 멈춘 뒤 SAVE_DELAY_MS 가 지나면 저장 (바뀐 바이트만 기록 -> EEPROM 수명 문제 없음)
 */

#include <Servo.h>
#include <EEPROM.h>

const uint8_t NUM_JOINTS = 4;
const uint8_t SERVO_PINS[NUM_JOINTS] = {3, 4, 5, 6};   // base, shoulder, upperarm, lowerarm
const char    CMD_CHARS[NUM_JOINTS]  = {'a', 'b', 'c', 'd'};

const float         MAX_SPEED_DEG_S  = 120.0;  // 서보 이동 속도 (SG90 최대 약 600 deg/s)
const unsigned long UPDATE_PERIOD_MS = 10;     // 서보 위치 갱신 주기 (100 Hz)
const unsigned long STATE_PERIOD_MS  = 50;     // 상태 보고 주기   (20 Hz)
const unsigned long SAVE_DELAY_MS    = 1000;   // 멈춘 뒤 이 시간이 지나면 자세 저장

// EEPROM 레이아웃: [0] MAGIC, [1..8] 관절 각도 x4 (uint16, 0.1도 단위)
const uint8_t EEPROM_MAGIC = 0xA7;
const int     EEPROM_ADDR  = 0;

Servo servos[NUM_JOINTS];
float currentDeg[NUM_JOINTS];
float targetDeg[NUM_JOINTS];

char    rxBuf[48];
uint8_t rxLen = 0;

unsigned long lastUpdateMs = 0;
unsigned long lastStateMs  = 0;
unsigned long lastMoveMs   = 0;
bool          poseDirty    = false;   // 저장 안 된 움직임이 있음

void setup() {
  Serial.begin(115200);

  bool restored = loadPose();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    targetDeg[i] = currentDeg[i];
    // attach 전에 펄스 폭을 정해 두면 첫 펄스부터 저장된 자세 -> 서보가 튀지 않는다
    servos[i].writeMicroseconds(degToUs(currentDeg[i]));
    servos[i].attach(SERVO_PINS[i]);
  }

  Serial.println(F("Robot Arm Ready. cmd: a90b90c90d90e / state: s<a>,<b>,<c>,<d>"));
  Serial.println(restored ? F("Pose restored from EEPROM") : F("No saved pose -> 90,90,90,90"));
}

void loop() {
  readSerial();

  unsigned long now = millis();

  if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
    float dt = (now - lastUpdateMs) / 1000.0;
    lastUpdateMs = now;
    stepServos(dt);
  }

  if (now - lastStateMs >= STATE_PERIOD_MS) {
    lastStateMs = now;
    sendState();
  }

  if (poseDirty && now - lastMoveMs >= SAVE_DELAY_MS) {
    savePose();
    poseDirty = false;
  }
}

// 'e' 또는 줄바꿈이 올 때까지 모아서 한 명령으로 처리 (String 미사용 -> 메모리 단편화 없음)
void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'e' || c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        parseCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // 너무 긴 쓰레기 데이터는 버린다
    }
  }
}

// "a90b45c120d10" -> 각 문자 뒤의 숫자를 목표 각도로
void parseCommand(const char *cmd) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    const char *p = strchr(cmd, CMD_CHARS[i]);
    if (p == NULL) continue;
    p++;
    if (*p < '0' || *p > '9') continue;
    targetDeg[i] = constrain(atoi(p), 0, 180);
  }
}

void stepServos(float dt) {
  float maxStep = MAX_SPEED_DEG_S * dt;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    float diff = targetDeg[i] - currentDeg[i];
    if (diff == 0) continue;
    if (diff > maxStep)       diff = maxStep;
    else if (diff < -maxStep) diff = -maxStep;
    currentDeg[i] += diff;
    servos[i].writeMicroseconds(degToUs(currentDeg[i]));
    lastMoveMs = millis();
    poseDirty  = true;
  }
}

// 0~180도 -> 544~2400us (Servo 라이브러리 기본 범위), 1도보다 세밀하게 움직인다
int degToUs(float deg) {
  return 544 + (int)(deg * (2400 - 544) / 180.0 + 0.5);
}

bool loadPose() {
  if (EEPROM.read(EEPROM_ADDR) != EEPROM_MAGIC) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) currentDeg[i] = 90.0;
    return false;
  }
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t tenths;
    EEPROM.get(EEPROM_ADDR + 1 + i * 2, tenths);
    currentDeg[i] = constrain(tenths, 0, 1800) / 10.0;
  }
  return true;
}

void savePose() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t tenths = (uint16_t)(currentDeg[i] * 10.0 + 0.5);
    EEPROM.put(EEPROM_ADDR + 1 + i * 2, tenths);   // 값이 같으면 쓰지 않음 (update)
  }
  EEPROM.update(EEPROM_ADDR, EEPROM_MAGIC);
}

void sendState() {
  Serial.print('s');
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(currentDeg[i], 1);
  }
  Serial.println();
}
