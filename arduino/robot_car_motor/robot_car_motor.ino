/*
 * robot_car_motor.ino : ROS2 시리얼 노드의 명령을 받아 2바퀴 모터를 구동
 *
 * 프로토콜 (PC -> 아두이노, 115200bps, 한 줄에 명령 하나)
 *     "L,R\n"    L, R = 왼쪽/오른쪽 PWM  (-255 ~ 255, 음수 = 후진)
 *     예) "150,150\n" 전진   "-120,120\n" 제자리 좌회전   "0,0\n" 정지
 *
 * 안전장치: CMD_TIMEOUT_MS 동안 명령이 안 오면 자동 정지
 *           (USB가 빠지거나 ROS 노드가 죽어도 차가 혼자 달리지 않음)
 *
 * 시리얼 모니터(115200, 줄 끝: 새 줄)에서 "100,100" 을 직접 쳐서 ROS 없이 테스트 가능.
 *
 * 모터 드라이버: L298N 기준 (TB6612 도 IN1/IN2/PWM 구조가 같아서 핀만 바꾸면 됨,
 *               TB6612 는 STBY 핀을 HIGH 로 연결)
 */

// ---------------- 배선 (실제 연결에 맞게 수정) ----------------
// 왼쪽 모터 = L298N 의 A 채널
const int LEFT_EN  = 5;   // ENA (PWM 핀이어야 함: 우노 기준 3,5,6,9,10,11)
const int LEFT_IN1 = 7;   // IN1
const int LEFT_IN2 = 8;   // IN2
// 오른쪽 모터 = L298N 의 B 채널
const int RIGHT_EN  = 6;  // ENB (PWM 핀)
const int RIGHT_IN1 = 9;  // IN3
const int RIGHT_IN2 = 10; // IN4

// 전진 명령인데 바퀴가 뒤로 돌면 해당 쪽을 true 로 (배선 반대로 바꾸는 것과 같은 효과)
const bool LEFT_INVERT  = false;
const bool RIGHT_INVERT = false;

const unsigned long BAUD = 115200;
const unsigned long CMD_TIMEOUT_MS = 500;
// --------------------------------------------------------------

char buf[32];
byte bufLen = 0;
unsigned long lastCmdMs = 0;

void setMotor(int en, int in1, int in2, int pwm, bool invert) {
  if (invert) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);   // 정지(프리런)
  }
  analogWrite(en, abs(pwm));
}

void drive(int left, int right) {
  setMotor(LEFT_EN,  LEFT_IN1,  LEFT_IN2,  left,  LEFT_INVERT);
  setMotor(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, right, RIGHT_INVERT);
}

// "L,R" 한 줄 처리. 형식이 틀리면 무시하고 에러 응답
void handleLine(char *line) {
  char *comma = strchr(line, ',');
  if (comma == NULL) {
    Serial.println("ERR format, use L,R");
    return;
  }
  *comma = '\0';
  int left  = atoi(line);
  int right = atoi(comma + 1);
  drive(left, right);
  lastCmdMs = millis();
}

void setup() {
  int pins[] = {LEFT_EN, LEFT_IN1, LEFT_IN2, RIGHT_EN, RIGHT_IN1, RIGHT_IN2};
  for (int p : pins) pinMode(p, OUTPUT);
  drive(0, 0);
  Serial.begin(BAUD);
  Serial.println("READY");
}

void loop() {
  // 한 글자씩 모아서 '\n' 이 오면 한 줄 처리 (String 클래스 안 씀: 메모리 단편화 방지)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) {
        buf[bufLen] = '\0';
        handleLine(buf);
        bufLen = 0;
      }
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    } else {
      bufLen = 0;  // 너무 긴 줄은 버림
    }
  }

  // 워치독: 명령이 끊기면 정지
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    drive(0, 0);
  }
}
