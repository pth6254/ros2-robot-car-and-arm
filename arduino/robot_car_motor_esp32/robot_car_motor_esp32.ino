/*
 * robot_car_motor_esp32.ino : ESP32 버전 모터 제어 (Arduino Uno 버전과 같은 프로토콜)
 *
 * 프로토콜 (한 줄에 명령 하나)
 *     "L,R\n"    L, R = 왼쪽/오른쪽 PWM  (-255 ~ 255, 음수 = 후진)
 *
 * 연결 방식 (USB 유선은 항상 동작, WiFi 는 USE_WIFI 1 일 때 추가로 동작)
 *   - WiFi 무선: ros2 launch robot_car bringup.launch.py mode:=wifi
 *                -> PC 의 tcp_node 가 ESP32 의 TCP 서버(3333번 포트)에 접속
 *                -> 주소는 "robot-car.local" (mDNS) 로 찾음. 안 되면 host:=<ESP32 IP>
 *                   (ESP32 IP 는 부팅 시 USB 시리얼 모니터에 출력됨)
 *   - USB 유선 : ros2 launch robot_car bringup.launch.py mode:=real port:=/dev/ttyUSB0
 *
 * ★ 업로드 전에 wifi_config.example.h 를 wifi_config.h 로 복사하고 WiFi 이름/비밀번호를 입력하세요.
 *   PC 와 ESP32 는 같은 WiFi(공유기)에 연결되어 있어야 합니다. ESP32 는 2.4GHz 만 지원.
 *
 * 안전장치: CMD_TIMEOUT_MS 동안 명령이 안 오면 자동 정지
 *
 * 구조: Wheel 클래스 (바퀴 하나 = 핀, 방향 반전, 현재 출력, 가속 제한, 비상 정지)
 *       객체 2개 leftWheel / rightWheel 에 목표 속도를 나눠 주어 차량을 구동 (applyOutput)
 *
 * 앞쪽 IR 장애물 센서 2개 (D22 = 왼쪽, D23 = 오른쪽)
 *   - 어느 한쪽이라도 장애물을 감지하면 '전진'만 막음 (후진, 제자리 회전은 허용)
 *     -> ESP32 안에서 바로 판단하므로 WiFi 지연과 상관없이 즉시 멈춤
 *   - 센서 상태를 PC 로 보냄 (ESP32 -> PC):  "IR,왼쪽,오른쪽\n"  (1 = 장애물, 0 = 없음)
 *     상태가 바뀔 때 즉시 + 0.2초마다. PC 의 tcp_node / serial_node 가 /ir/left, /ir/right 로 발행
 *
 * MPU-6050 자이로 직선 주행 (GY-521: SDA = D21, SCL = TX2(GPIO17), INT = D4, VCC = 3V3)
 *   - DMP(센서 내부 자세 계산)로 방향(yaw)을 읽음. INT 핀 인터럽트로 새 데이터가 올 때마다 읽음
 *   - 직진 명령(L == R)이 들어오면 그 순간 방향을 목표로 잡고, 틀어진 만큼 좌우 PWM 을 보정
 *     회전 명령 중에는 보정하지 않고, 회전이 끝난 방향이 다음 직진의 목표가 됨
 *   - ★ 전원을 켠 직후 약 2초간 센서 보정 -> 이때 차를 움직이지 말 것
 *   - PC -> ESP32:  "H,kp,kd\n"   보정 세기 설정 (kp = 0 이면 보정 끔). tcp_node 가 접속 시 자동 전송
 *   - ESP32 -> PC:  "IMU,yaw,gz,qw,qx,qy,qz,hold\n"  0.05초마다 (yaw/gz: 도, 도/초, hold: 보정 중이면 1)
 *   - MPU-6050 이 없거나 응답이 없어도 모터 조종은 그대로 동작 (보정만 꺼짐)
 *
 * 안정성
 *   - 독립 워치독(esp_timer): loop() 가 멈춰도 명령이 0.5초 끊기면 모터 정지
 *   - 상태 보고는 '막히지 않는 전송' (WiFi 가 나쁘면 그 회차만 버림) -> loop 가 멈추지 않음
 *   - 가속 제한: 출력 변화를 부드럽게, 반대 방향 전환은 0 을 거쳐서 (순간 전류 -> 재부팅 방지)
 *   - ESP32 -> PC:  "BOOT,원인,켜진초"  (접속할 때), "STAT,워치독횟수,버린보고,켜진초" (5초마다)
 *
 * 보드: ESP32 DevKit (ESP32-WROOM-32), Arduino IDE 보드 매니저 "esp32" 2.0 이상
 *       (Tools > Board > ESP32 Dev Module)
 *
 * ※ ESP32 는 3.3V 핀입니다.
 *    - L298N 입력(IN1~4, ENA/ENB)은 3.3V 로 동작 OK
 *    - 3.3V 를 넘는 신호를 ESP32 핀에 직접 넣지 말 것 (IR 센서 VCC 는 3V3 에 연결)
 */

// ================= 연결 방식 =================
#ifndef USE_WIFI
#define USE_WIFI 1            // 1 = WiFi 무선 + USB 유선, 0 = USB 유선만
#endif
#if USE_WIFI
#include "wifi_config.h"      // ★ wifi_config.example.h 를 복사해서 만든 파일 (git 에는 올라가지 않음)
const char *WIFI_SSID = WIFI_SSID_VALUE;
const char *WIFI_PASS = WIFI_PASS_VALUE;

// ★ 고정 IP: 공유기가 ESP32 에 IP 를 자동으로 안 줘서(DHCP 실패) 직접 지정함
//   1 = 아래 IP 사용, 0 = 공유기 자동 할당(DHCP)
//   ※ 다른 기기와 IP 가 겹치지 않게, 공유기 관리화면에서 이 IP 를 ESP32 의 MAC 주소에 예약해 두면 가장 안전
#define USE_STATIC_IP 1
const IPAddress STATIC_IP(192, 168, 0, 200);
const IPAddress GATEWAY  (192, 168, 0, 1);
const IPAddress SUBNET   (255, 255, 255, 0);
#endif
const char *HOSTNAME  = "robot-car";         // PC 에서 robot-car.local 로 접속
const uint16_t TCP_PORT = 3333;

// ================= 배선 (ESP32 DevKit 기준) =================
// 사용하면 안 되는 핀: 6~11(내부 플래시), 34~39(입력 전용), 0/2/12/15(부팅 모드 핀)
// ※ 실제 배선 기준 (ENA/ENB 를 D18/D19 에 연결, IN4 를 D33 에 연결)
// ※ 실차 시험 결과 (w 후진, s 전진, a/d 정상) -> 좌우 모터가 서로 바뀌어 있고 방향도 반대였음
//   선은 그대로 두고 코드에서 맞춤: 왼쪽 <-> 오른쪽 채널 교환 + 각 채널의 방향 핀(IN) 순서 뒤집기
// 왼쪽 모터 = L298N B 채널 (OUT3/OUT4)
const int LEFT_EN  = 19;   // ENB  <- D19 (점퍼 캡 제거 후 연결, 속도 PWM)
const int LEFT_IN1 = 33;   // IN4  <- D33  (방향 뒤집기 위해 IN3/IN4 순서 바꿈)
const int LEFT_IN2 = 32;   // IN3  <- D32
// 오른쪽 모터 = L298N A 채널 (OUT1/OUT2)
const int RIGHT_EN  = 18;  // ENA  <- D18 (점퍼 캡 제거 후 연결, 속도 PWM)
const int RIGHT_IN1 = 27;  // IN2  <- D27  (방향 뒤집기 위해 IN1/IN2 순서 바꿈)
const int RIGHT_IN2 = 26;  // IN1  <- D26

// 전진 명령인데 바퀴가 뒤로 돌면 해당 쪽을 true 로
const bool LEFT_INVERT  = false;
const bool RIGHT_INVERT = false;

const unsigned long BAUD = 115200;
const unsigned long CMD_TIMEOUT_MS = 500;

// 가속 제한 (모터 출력이 한 번에 크게 바뀌지 않게) [PWM/초]
//   빠른 회전, 특히 a -> d 처럼 반대 방향으로 바꾸면 순간 전류가 크게 튀어
//   배터리 전압이 떨어지고 ESP32 가 재부팅될 수 있음 -> 출력 변화를 부드럽게
//   반대 방향으로 바꿀 때는 0 까지 줄였다가 다시 올림
const float RAMP_ACCEL = 500;    // 빨라질 때: 0 -> 157 약 0.3초, 0 -> 255 약 0.5초
const float RAMP_DECEL = 1500;   // 느려질 때: 157 -> 0 약 0.1초 (멈추는 건 빠르게)
// ※ 워치독 정지(명령 끊김)와 장애물 전진 차단은 가속 제한 없이 즉시 적용 (안전 우선)

// 독립 워치독 타이머: loop() 가 WiFi 전송 등으로 멈춰도 명령이 끊기면 모터를 세움
const uint64_t FAILSAFE_PERIOD_US = 50000;   // 0.05초마다 검사

// 앞쪽 IR 장애물 센서 (FC-51 류: VCC, GND, OUT 3핀). VCC 는 ESP32 3V3 에 연결
const int IR_LEFT  = 22;   // 왼쪽 센서 OUT  <- D22
const int IR_RIGHT = 23;   // 오른쪽 센서 OUT <- D23
const bool IR_ACTIVE_LOW = true;            // 장애물 감지 시 OUT = LOW (대부분의 모듈). 반대면 false
const bool OBSTACLE_STOP = true;            // false 로 하면 감지만 하고 주행은 막지 않음
const unsigned long IR_CLEAR_MS = 100;      // 장애물이 사라져도 이 시간 동안 계속 비어야 해제 (깜빡임 방지)
const unsigned long IR_REPORT_MS = 200;     // PC 로 상태 보내는 주기

// MPU-6050 (GY-521) 자이로 -> 직선 주행 보정
#ifndef USE_IMU
#define USE_IMU 1             // 0 으로 하면 MPU-6050 코드 전체 제외
#endif
const int IMU_SDA = 21;       // SDA <- D21
const int IMU_SCL = 17;       // SCL <- TX2 (GPIO17)
const int IMU_INT = 4;        // INT <- D4
const unsigned long IMU_REPORT_MS = 50;     // PC 로 방향 보내는 주기 (20Hz)
// 보정 PWM = headingKp * 방향오차[도] - headingKd * 회전속도[도/초]  (PC 에서 "H,kp,kd" 로 변경 가능)
float headingKp = 6.0;        // 오차 1도당 PWM 6
float headingKd = 0.4;        // 흔들림 억제
const int HEADING_MAX_CORR = 60;            // 보정 최대 PWM (너무 세게 꺾지 않도록)
// 방향(yaw) 계산 방식
//   0 = 자이로 Z축 적분 (기본). DMP INT 신호마다 회전 속도를 읽어 시간만큼 더함.
//       가속도(기울기) 해석과 무관해서 호환 칩(WHO_AM_I 0x98 등)에서도 정확함
//   1 = DMP 쿼터니언. 정품 MPU-6050 에서 사용 (호환 칩은 기울기를 잘못 계산해 방향이 틀어질 수 있음)
const int YAW_SOURCE = 0;
const float GYRO_LSB_PER_DPS = 16.4f;       // DMP 초기화가 자이로 범위를 ±2000도/초로 설정 -> 1도/초 = 16.4
const float GYRO_STILL_STD = 0.5f;          // 부팅 측정 중 자이로 흔들림이 이보다 크면 '움직였다'고 판단 [도/초]
const float GYRO_MAX_BIAS = 5.0f;           // 자이로 오차가 이보다 크면 정상 오차가 아니라 회전 중이었다고 판단 [도/초]
// ============================================================

// ================= Wheel 클래스: 바퀴(모터) 하나 =================
// 바퀴 하나에 필요한 것(핀 3개, 방향 반전, 현재 출력, 가속 제한, 비상 정지)을 한 객체에 묶음.
// 차량은 이 객체 2개(leftWheel, rightWheel)에 목표 속도를 나눠 주는 것으로 구동함.
//
//   setTarget(pwm)  목표 속도 설정 (-255 ~ 255, 음수 = 후진)
//   update(dt)      가속 제한을 거쳐 실제 출력을 목표 쪽으로 dt 만큼 움직이고 핀에 씀 (loop 마다)
//   setNow(pwm)     가속 제한 없이 즉시 적용 (워치독 정지, 장애물 전진 차단 등 안전 정지)
//   emergencyStop() 방향 핀만 LOW. 독립 워치독 타이머(다른 태스크)에서 불러도 안전한 동작만 함
class Wheel {
public:
  Wheel(const char *name, int enPin, int in1Pin, int in2Pin, bool invert)
      : name_(name), en_(enPin), in1_(in1Pin), in2_(in2Pin), invert_(invert) {}

  void begin() {                      // setup() 에서 한 번: 핀 설정 + 정지 상태로 시작
    pinMode(en_, OUTPUT);
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    setNow(0);
  }

  void setTarget(int pwm) { target_ = constrain(pwm, -255, 255); }

  void update(float dt) {
    out_ = rampToward(out_, target_, dt);
    writePins((int)out_);
  }

  void setNow(int pwm) {
    setTarget(pwm);
    out_ = target_;
    writePins(target_);
  }

  // ※ esp_timer 콜백(다른 태스크)에서 호출됨: GPIO 쓰기만 해야 loop() 와 동시에 실행돼도 안전
  void emergencyStop() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
  }
  bool isDriving() const { return digitalRead(in1_) || digitalRead(in2_); }

  int output() const { return (int)out_; }
  const char *name() const { return name_; }

private:
  const char *name_;
  int en_, in1_, in2_;
  bool invert_;
  int target_ = 0;      // 목표 출력
  float out_ = 0;       // 가속 제한을 거친 실제 출력

  // 방향 핀 + 속도(PWM) 출력
  void writePins(int pwm) {
    if (invert_) pwm = -pwm;
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
      digitalWrite(in1_, HIGH); digitalWrite(in2_, LOW);
    } else if (pwm < 0) {
      digitalWrite(in1_, LOW);  digitalWrite(in2_, HIGH);
    } else {
      digitalWrite(in1_, LOW);  digitalWrite(in2_, LOW);
    }
    analogWrite(en_, abs(pwm));   // ESP32 Arduino core 2.0+ 에서 지원
  }

  // 가속 제한: 현재 출력 cur 를 목표 target 쪽으로 dt 동안 허용된 만큼만 움직임
  //   0 에서 멀어지면(빨라지면) RAMP_ACCEL, 0 쪽으로 가면(느려지면) RAMP_DECEL
  //   반대 방향 목표면 일단 0 쪽(감속)으로만 움직임 -> 0 을 지난 뒤부터 가속
  static float rampToward(float cur, float target, float dt) {
    if (cur == target) return cur;
    bool slowingDown = (target == 0) || (cur > 0 && target < cur) || (cur < 0 && target > cur);
    float step = (slowingDown ? RAMP_DECEL : RAMP_ACCEL) * dt;
    if (cur > 0 && target < 0) target = 0;          // 반대 방향: 먼저 0 까지
    if (cur < 0 && target > 0) target = 0;
    if (target > cur) return fminf(cur + step, target);
    return fmaxf(cur - step, target);
  }
};

// 객체 2개 = 차량의 두 바퀴 (핀 번호는 위 배선 설정에서)
Wheel leftWheel ("left",  LEFT_EN,  LEFT_IN1,  LEFT_IN2,  LEFT_INVERT);
Wheel rightWheel("right", RIGHT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_INVERT);

#if USE_WIFI
#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>   // ::send(..., MSG_DONTWAIT) : 막히지 않는 전송
WiFiServer server(TCP_PORT);
WiFiClient client;
bool netStarted = false;   // TCP 서버 + mDNS 를 시작했는지
#endif

// 입력 통로마다 줄 버퍼를 따로 둠 (USB 와 WiFi 가 섞여 들어와도 안전)
struct LineBuffer {
  char buf[32];
  byte len = 0;
};
LineBuffer usbBuf;
#if USE_WIFI
LineBuffer wifiBuf;
#endif

volatile unsigned long lastCmdMs = 0;   // 워치독 타이머(다른 태스크)도 읽으므로 volatile
int cmdLeft = 0, cmdRight = 0;          // PC 에서 받은 마지막 명령 (그대로 보관, 실제 출력은 loop 에서 결정)
volatile unsigned long failsafeCount = 0;   // 독립 워치독이 모터를 세운 횟수 (진단용)

// IR 장애물 센서 상태
bool irLeft = false, irRight = false;   // 걸러진(안정된) 감지 상태
unsigned long irLeftSeenMs = 0, irRightSeenMs = 0;   // 마지막으로 감지된 시각
unsigned long irLastReportMs = 0;

// MPU-6050 상태 (USE_IMU 0 이어도 변수는 있음 -> imuOk = false 로 보정이 꺼진 채 동작)
bool imuOk = false;                     // DMP 초기화 성공
float yawDeg = 0, gzDps = 0;            // 방향 [도, 반시계 +], 회전 속도 [도/초]
float yawDriftDps = 0;                  // 부팅 때 측정한 '가만히 있어도 흐르는' 속도 -> 소프트웨어로 빼 줌
unsigned long yawDriftT0 = 0;
float gyroBiasZ = 0;                    // 부팅 때 측정한 자이로 Z 오차 [도/초] (YAW_SOURCE 0)
unsigned long gyroLastUs = 0;
float quatW = 1, quatX = 0, quatY = 0, quatZ = 0;
bool headingHolding = false;            // 지금 직진 보정 중인지
float targetYawDeg = 0;                 // 직진 시작 순간의 방향
unsigned long imuLastReadMs = 0, imuLastReportMs = 0, imuStartMs = 0;
volatile unsigned long imuIntCount = 0; // INT 인터럽트 횟수 (배선 확인용)
bool imuIntChecked = false;

#if USE_IMU
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
uint8_t imuFifo[64];
volatile bool imuDataReady = false;
#endif

#if USE_WIFI
// WiFi 가 연결됐으면 (한 번만) TCP 서버와 mDNS 를 시작. 늦게 연결돼도 loop() 에서 불러 줌
// (※ 함수는 LineBuffer 정의 아래에 둬야 함: Arduino 가 첫 함수 앞에 자동 선언을 끼워 넣기 때문)
bool startNetworkIfConnected() {
  if (netStarted) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFi.setSleep(false);   // 접속 후 다시 한 번 절전 끔 (접속 과정에서 기본값으로 돌아가는 경우 대비)
  server.begin();
  server.setNoDelay(true);
  if (MDNS.begin(HOSTNAME)) MDNS.addService("robot-car", "tcp", TCP_PORT);
  netStarted = true;
  Serial.printf("WiFi OK  IP=%s  ->  mode:=wifi  (또는 host:=%s)\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
  return true;
}

// 막히지 않는 전송: TCP 보낼 공간이 없으면 이번 보고는 버리고 바로 돌아옴
//   client.println() 은 WiFi 가 나쁠 때 전송이 끝날 때까지 수 초 멈출 수 있고,
//   그동안 loop() 의 모터 제어가 멈춤 -> 주기적인 상태 보고는 이 함수로 보냄 (다음 주기에 새 값이 감)
unsigned long wifiDropCount = 0;
void clientSend(const char *msg) {
  if (!client || !client.connected()) return;
  int fd = client.fd();
  if (fd < 0) return;
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "%s\r\n", msg);
  if (n <= 0 || n >= (int)sizeof(buf)) return;
  int sent = ::send(fd, buf, n, MSG_DONTWAIT);
  if (sent != n) wifiDropCount++;
}
#endif

// 재부팅 원인 (esp_reset_reason) -> 짧은 이름. BROWNOUT = 전압 부족
const char *bootReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";    // 전원 켜짐, USB 연결/EN 버튼, 시리얼 포트 열기
    case ESP_RST_BROWNOUT: return "BROWNOUT";   // ★ 전압 부족
    case ESP_RST_PANIC:    return "PANIC";      // 프로그램 오류
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WDT";        // 프로그램이 멈춰서 감시 타이머가 재부팅
    case ESP_RST_SW:       return "SW";         // 프로그램이 스스로 재시작 (업로드 직후 등)
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default:               return "UNKNOWN";
  }
}

// ---------------- 독립 워치독 (esp_timer, loop 와 별개로 0.05초마다 실행) ----------------
// loop() 가 WiFi 전송에 막혀 멈추면 loop 안의 워치독도 같이 멈춰서, 모터가 마지막 명령(예: 빠른 회전)을
// 계속 유지할 수 있음. 이 타이머는 loop 와 상관없이 돌면서, 명령이 끊겼으면 방향 핀을 모두 LOW 로 내려 정지.
// (방향 핀 LOW 만 쓰는 이유: GPIO 쓰기는 한 번에 끝나는 동작이라 loop 와 동시에 실행돼도 안전)
void failsafeTimer(void *) {
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    if (leftWheel.isDriving() || rightWheel.isDriving()) {
      leftWheel.emergencyStop();
      rightWheel.emergencyStop();
      failsafeCount++;
    }
  }
}

// 한 줄 처리. 0 = 형식 오류, 1 = 모터 명령, 2 = 설정 명령(응답 필요)
//   "L,R"     : 모터 명령. 바로 돌리지 않고 저장 -> loop() 의 applyOutput() 이 실제 출력 결정
//   "H,kp,kd" : 직선 주행 보정 세기 (kp = 0 이면 보정 끔)
int handleLine(char *line) {
  if (line[0] == 'H' || line[0] == 'h') {
    char *p1 = strchr(line, ',');
    if (p1 == NULL) return 0;
    char *p2 = strchr(p1 + 1, ',');
    headingKp = constrain((float)atof(p1 + 1), 0.0f, 50.0f);
    if (p2 != NULL) headingKd = constrain((float)atof(p2 + 1), 0.0f, 10.0f);
    headingHolding = false;                 // 새 설정으로 목표 방향 다시 잡기
    return 2;
  }
  char *comma = strchr(line, ',');
  if (comma == NULL) return false;
  *comma = '\0';
  cmdLeft  = constrain(atoi(line), -255, 255);
  cmdRight = constrain(atoi(comma + 1), -255, 255);
  lastCmdMs = millis();
  return 1;
}

// ---------------- MPU-6050 (DMP) ----------------
float wrap180(float a) {          // 각도를 -180 ~ 180 으로
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// 쿼터니언 -> yaw [도]. 칩 Z축이 위를 향하므로 반시계 회전이 + (ROS 와 같은 방향)
float yawFromQuat(float w, float x, float y, float z) {
  return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 57.29578f;
}

// I2C 버스 복구: ESP32 가 센서를 읽는 도중 재시작되면 센서가 SDA 를 LOW 로 붙잡고 멈출 수 있음
// (센서는 전원이 계속 켜져 있으므로). SCL 을 9번 흔들어 센서가 남은 비트를 내보내고 놓게 함.
// 반환: 복구 전 SDA, SCL 상태 (진단용 출력)
void i2cBusRecover(int sda, int scl, int &sdaBefore, int &sclBefore) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(10);
  sdaBefore = digitalRead(sda);
  sclBefore = digitalRead(scl);
  pinMode(scl, OUTPUT);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  // STOP 조건 (SCL HIGH 상태에서 SDA LOW -> HIGH)
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);  delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH); delayMicroseconds(5);
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
}

#if USE_IMU
void IRAM_ATTR imuIsr() {         // INT 핀: DMP 가 새 데이터를 FIFO 에 넣을 때마다 호출 (초당 약 100번)
  imuDataReady = true;
  imuIntCount++;
}
#endif

// 새 DMP 데이터가 있으면 방향(yaw)과 회전 속도 갱신
void updateIMU() {
#if USE_IMU
  if (!imuOk) return;
  unsigned long now = millis();
  // INT 가 오면 바로 읽고, INT 선에 문제가 있어도 20ms 마다는 직접 확인 (안전망)
  if (!imuDataReady && now - imuLastReadMs < 20) return;
  imuDataReady = false;
  if (!mpu.dmpGetCurrentFIFOPacket(imuFifo)) return;   // FIFO 를 비워 줘야 DMP/INT 가 계속 동작

  if (YAW_SOURCE == 0) {
    // ---- 자이로 Z 적분: 회전 속도(오차 제거) x 경과 시간을 계속 더함 ----
    unsigned long us = micros();
    float rate = mpu.getRotationZ() / GYRO_LSB_PER_DPS - gyroBiasZ;   // [도/초], 반시계 +
    if (fabsf(rate) < 0.3f) rate = 0;          // 아주 작은 떨림은 무시 (가만히 있을 때 흐름 방지)
    if (gyroLastUs != 0) {
      float dt = (us - gyroLastUs) / 1e6f;
      if (dt > 0 && dt < 0.1f) yawDeg = wrap180(yawDeg + rate * dt);
    }
    gyroLastUs = us;
    gzDps = 0.7f * gzDps + 0.3f * rate;
    // RViz 표시용 쿼터니언: 수평으로 서 있다고 보고 yaw 만으로 만듦
    float h = yawDeg * 0.0087266f;             // (도 -> 라디안) / 2
    quatW = cosf(h); quatX = 0; quatY = 0; quatZ = sinf(h);
  } else {
    // ---- DMP 쿼터니언 (정품 칩용) ----
    Quaternion q;
    mpu.dmpGetQuaternion(&q, imuFifo);
    quatW = q.w; quatX = q.x; quatY = q.y; quatZ = q.z;
    float yaw = wrap180(yawFromQuat(q.w, q.x, q.y, q.z) - yawDriftDps * (now - yawDriftT0) / 1000.0f);
    if (imuLastReadMs != 0) {
      float dt = (now - imuLastReadMs) / 1000.0f;
      if (dt > 0.001f) gzDps = 0.7f * gzDps + 0.3f * (wrap180(yaw - yawDeg) / dt);
    }
    yawDeg = yaw;
  }
  imuLastReadMs = now;
#endif
}

#if USE_IMU
// 자이로 Z 오차(평균)와 흔들림(표준편차)을 ms 동안 측정 [도/초]. 부팅 때 움직임이 감지되면 다시 재는 용도
// DMP FIFO 도 같이 비워 줌 (안 비우면 넘쳐서 DMP 가 멈춤)
void measureGyroBias(unsigned long ms, float &mean, float &sd) {
  double s = 0, s2 = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    mpu.dmpGetCurrentFIFOPacket(imuFifo);
    float r = mpu.getRotationZ() / GYRO_LSB_PER_DPS;
    s += r; s2 += (double)r * r; n++;
    delay(5);
  }
  mean = n ? s / n : 0;
  sd = n ? sqrtf(fmaxf(0.0f, (float)(s2 / n) - mean * mean)) : 99;
}
#endif

// 직선 주행 보정: 직진 명령(L == R, 0 아님)일 때만. 틀어진 방향만큼 좌우 PWM 차이를 줌
//   오차 > 0 (목표가 왼쪽) -> 오른쪽 바퀴 빠르게, 왼쪽 느리게 -> 반시계 회전으로 복귀
//   후진일 때도 같은 식이 맞음 (회전 방향은 '오른쪽 - 왼쪽' 속도 차이로 정해지므로)
void headingHold(int &l, int &r) {
  bool straight = imuOk && headingKp > 0 && l == r && l != 0;
  if (!straight) {
    headingHolding = false;
    return;
  }
  static bool holdForward = true;
  // 직진 시작 순간, 또는 전진 <-> 후진이 바뀐 순간의 방향을 새 목표로
  if (!headingHolding || (l > 0) != holdForward) {
    headingHolding = true;
    holdForward = (l > 0);
    targetYawDeg = yawDeg;
  }
  float err = wrap180(targetYawDeg - yawDeg);
  float corr = headingKp * err - headingKd * gzDps;
  corr = constrain(corr, (float)-HEADING_MAX_CORR, (float)HEADING_MAX_CORR);
  int base = l;
  l = constrain(base - (int)corr, -255, 255);
  r = constrain(base + (int)corr, -255, 255);
}

// ---------------- IR 장애물 센서 ----------------
bool irRaw(int pin) {
  bool level = digitalRead(pin);
  return IR_ACTIVE_LOW ? !level : level;
}

// 감지되면 즉시 '장애물', 사라지면 IR_CLEAR_MS 동안 계속 비어야 '없음' (경계 거리에서 깜빡임 방지)
// 상태가 바뀌었으면 true
bool updateIR() {
  unsigned long now = millis();
  if (irRaw(IR_LEFT))  irLeftSeenMs = now;
  if (irRaw(IR_RIGHT)) irRightSeenMs = now;
  bool l = (irLeftSeenMs  != 0) && (now - irLeftSeenMs  < IR_CLEAR_MS);
  bool r = (irRightSeenMs != 0) && (now - irRightSeenMs < IR_CLEAR_MS);
  bool changed = (l != irLeft) || (r != irRight);
  irLeft = l;
  irRight = r;
  return changed;
}

// 실제 모터 출력 = 저장된 명령에서, 장애물이 있으면 '전진 성분'만 제거
//   전진 성분 fwd = (L+R)/2, 회전 성분 rot = (R-L)/2  ->  L = fwd - rot, R = fwd + rot
//   예) 전진 157,157 -> 0,0 (정지) / 오른쪽 커브 200,100 -> 50,-50 (오른쪽 제자리 회전만)
//       후진, 제자리 회전 -> 그대로
void applyOutput() {
  static unsigned long lastUs = 0;
  unsigned long nowUs = micros();
  float dt = lastUs ? (nowUs - lastUs) / 1e6f : 0;
  lastUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;                                 // loop 가 오래 멈췄다 와도 한 번에 크게 바뀌지 않게

  int l = cmdLeft, r = cmdRight;
  bool immediate = false;                                   // true 면 가속 제한 없이 바로 적용 (안전 정지)
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {              // 워치독: 명령이 끊기면 즉시 정지
    l = r = 0;
    immediate = true;
  }
  if (OBSTACLE_STOP && (irLeft || irRight)) {
    int fwd = (l + r) / 2;
    int rot = (r - l) / 2;
    if (fwd > 0) {
      l = -rot;
      r = rot;
      immediate = true;                                     // 장애물 앞: 전진 성분을 즉시 제거
    }
  }
  // 장애물 처리 '다음'에 직선 보정: 장애물로 막혀 0,0 이 되면 보정도 하지 않음 (제자리에서 돌지 않게)
  headingHold(l, r);

  // 두 바퀴 객체에 목표를 나눠 줌
  if (immediate) {                    // 안전 정지: 가속 제한 없이 바로
    leftWheel.setNow(l);
    rightWheel.setNow(r);
  } else {                            // 평소: 목표 설정 후 가속 제한을 거쳐 갱신
    leftWheel.setTarget(l);
    rightWheel.setTarget(r);
    leftWheel.update(dt);
    rightWheel.update(dt);
  }
}

// 한 글자씩 모아서 '\n' 이 오면 처리. 형식 오류면 reply 로 알림
template <typename Stream_t>
void feed(LineBuffer &lb, char c, Stream_t &reply) {
  if (c == '\n' || c == '\r') {
    if (lb.len > 0) {
      lb.buf[lb.len] = '\0';
      int res = handleLine(lb.buf);
      if (res == 0) {
        reply.println("ERR format, use L,R");
      } else if (res == 2) {          // 설정 명령 응답
        char ack[48];
        snprintf(ack, sizeof(ack), "H,ok,kp=%.2f,kd=%.2f,imu=%d", headingKp, headingKd, imuOk ? 1 : 0);
        reply.println(ack);
      }
      lb.len = 0;
    }
  } else if (lb.len < sizeof(lb.buf) - 1) {
    lb.buf[lb.len++] = c;
  } else {
    lb.len = 0;
  }
}

void setup() {
  leftWheel.begin();                  // 바퀴 객체: 핀 설정 + 정지 상태로 시작
  rightWheel.begin();
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  Serial.begin(BAUD);
  Serial.printf("\nBOOT: 재부팅 원인 = %s%s\n", bootReasonName(),
                esp_reset_reason() == ESP_RST_BROWNOUT ? "  ★ 전압 부족: 배터리 충전, 급회전, VIN 콘덴서 확인" : "");

  // 독립 워치독 타이머 시작 (loop 와 별개로 0.05초마다 '명령 끊김' 검사)
  {
    esp_timer_create_args_t args = {};
    args.callback = &failsafeTimer;
    args.name = "failsafe";
    esp_timer_handle_t h;
    if (esp_timer_create(&args, &h) == ESP_OK && esp_timer_start_periodic(h, FAILSAFE_PERIOD_US) == ESP_OK)
      Serial.println("FAILSAFE: 독립 워치독 타이머 시작 (0.05초 주기)");
    else
      Serial.println("FAILSAFE: ★ 타이머 시작 실패 -> loop 안의 워치독만 동작");
  }

#if USE_IMU
  // ---- MPU-6050 DMP 초기화 (WiFi 보다 먼저: 보정하는 2초 동안 차가 가만히 있도록) ----
  int sdaBefore, sclBefore;
  i2cBusRecover(IMU_SDA, IMU_SCL, sdaBefore, sclBefore);
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  bool found = false;
  for (int attempt = 0; attempt < 3 && !found; attempt++) {   // 최대 3번 시도
    Wire.beginTransmission(0x68);
    found = (Wire.endTransmission() == 0);
    if (!found) delay(50);
  }
  if (!found) {
    Serial.printf("IMU: MPU-6050 응답 없음 (0x68) -> 직선 보정 없이 동작. 시작 시 선 상태 SDA=%s SCL=%s\n",
                  sdaBefore ? "HIGH" : "LOW", sclBefore ? "HIGH" : "LOW");
    Serial.println("IMU:   VCC=3V3, GND, SDA=D21, SCL=TX2 배선 확인 (선 상태가 LOW 면 단락이나 센서 전원 문제)");
  } else {
    mpu.initialize();
    uint8_t who = 0;
    I2Cdev::readByte(0x68, 0x75, &who);   // WHO_AM_I 전체 값 (정품 0x68, 호환 칩은 0x98 등)
    Serial.printf("IMU: 칩 발견 (WHO_AM_I=0x%02X%s)\n", who, who == 0x68 ? ", 정품 MPU-6050" : ", 호환 칩");
    uint8_t st = mpu.dmpInitialize();
    if (st != 0) {
      Serial.printf("IMU: DMP 초기화 실패 (code=%d) -> 직선 보정 없이 동작\n", st);
    } else {
      // ※ 라이브러리 CalibrateAccel/CalibrateGyro 는 호환 칩에서 끝나지 않고 멈출 수 있어 사용하지 않음.
      //   대신 2초 동안 가만히 두고 '저절로 흐르는 방향 속도'를 재서 소프트웨어로 빼 줌. 최대 2.5초면 반드시 끝남
      mpu.setDMPEnabled(true);
      Serial.println("IMU: 2초 동안 방향 흐름 측정 중... 차를 움직이지 마세요");
      unsigned long t0 = millis(), tFirst = 0, tLast = 0;
      float yFirst = 0, yLast = 0, yPrev = 0, unwrapped = 0;
      int packets = 0;
      bool started = false;
      double sumAx = 0, sumAy = 0, sumAz = 0, sumGz = 0, sumGz2 = 0;   // 원시 가속도/자이로 평균, 자이로 흔들림
      int nRaw = 0;
      Quaternion qLast;
      while (millis() - t0 < 2500) {
        if (mpu.dmpGetCurrentFIFOPacket(imuFifo)) {
          Quaternion q;
          mpu.dmpGetQuaternion(&q, imuFifo);
          qLast = q;
          float y = yawFromQuat(q.w, q.x, q.y, q.z);
          packets++;
          if (millis() - t0 > 500) {             // 처음 0.5초는 DMP 가 자리 잡는 시간이라 버림
            if (!started) { started = true; yFirst = y; yPrev = y; unwrapped = y; tFirst = millis(); }
            unwrapped += wrap180(y - yPrev);
            yPrev = y;
            yLast = unwrapped;
            tLast = millis();
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sumAx += ax; sumAy += ay; sumAz += az; sumGz += gz; sumGz2 += (double)gz * gz; nRaw++;
          }
        }
        delay(5);
      }
      if (nRaw > 0) {
        // 원시 가속도로 본 실제 기울기 (±2g 범위: 1g = 16384)
        float ax = sumAx / nRaw / 16384.0f, ay = sumAy / nRaw / 16384.0f, az = sumAz / nRaw / 16384.0f;
        float g = sqrtf(ax * ax + ay * ay + az * az);
        float tilt = (g > 0.01f) ? acosf(constrain(az / g, -1.0f, 1.0f)) * 57.29578f : -1;
        // DMP 가 생각하는 기울기 (쿼터니언의 중력 방향)
        float gzq = qLast.w * qLast.w - qLast.x * qLast.x - qLast.y * qLast.y + qLast.z * qLast.z;
        float dmpTilt = acosf(constrain(gzq, -1.0f, 1.0f)) * 57.29578f;
        Serial.printf("IMU: 가속도 평균 (%.2f, %.2f, %.2f) g, |g|=%.2f -> 실제 기울기 %.1f도 / DMP 추정 기울기 %.1f도\n",
                      ax, ay, az, g, tilt, dmpTilt);

        // ---- 자이로 Z 오차: 측정 중 차가 움직였으면 그 회전까지 '오차'로 들어가 직진이 한쪽으로 휨 ----
        //      -> 흔들림(표준편차)과 크기를 검사해서, 움직였으면 최대 3번 다시 측정
        float mean = sumGz / nRaw / GYRO_LSB_PER_DPS;
        float sd = sqrtf(fmaxf(0.0f, (float)(sumGz2 / nRaw) / (GYRO_LSB_PER_DPS * GYRO_LSB_PER_DPS) - mean * mean));
        float bestMean = mean, bestSd = sd;
        int tries = 0;
        while ((bestSd > GYRO_STILL_STD || fabsf(bestMean) > GYRO_MAX_BIAS) && tries < 3) {
          tries++;
          Serial.printf("IMU: ★ 측정 중 움직임 감지 (흔들림 %.2f 도/초, 오차 %.2f 도/초) -> 다시 측정 %d/3, 차를 움직이지 마세요\n",
                        bestSd, bestMean, tries);
          delay(500);
          measureGyroBias(1500, mean, sd);
          if (sd < bestSd) { bestMean = mean; bestSd = sd; }
        }
        gyroBiasZ = bestMean;
        bool still = bestSd <= GYRO_STILL_STD && fabsf(bestMean) <= GYRO_MAX_BIAS;
        Serial.printf("IMU: 자이로 Z 오차 %.3f 도/초 (흔들림 %.3f) -> 빼서 사용. 방향 계산: %s\n", gyroBiasZ, bestSd,
                      YAW_SOURCE == 0 ? "자이로 적분" : "DMP 쿼터니언");
        if (!still) Serial.println("IMU: ★ 끝까지 가만히 있지 않음 -> 직선 보정이 한쪽으로 휠 수 있음. 차를 멈춘 채 전원을 다시 켜세요");
        if (tilt > 15) Serial.println("IMU: ★ 센서가 기울어져 있음 -> 판에 평평하게 고정하세요");
      }
      if (packets < 50 || tLast <= tFirst) {
        Serial.printf("IMU: DMP 데이터가 거의 안 나옴 (%d개) -> 직선 보정 없이 동작\n", packets);
        mpu.setDMPEnabled(false);
      } else {
        yawDriftDps = (yLast - yFirst) / ((tLast - tFirst) / 1000.0f);
        if (fabsf(yawDriftDps) > 3.0f) {
          Serial.printf("IMU: 방향 흐름이 너무 큼 (%.2f 도/초) -> 측정 중 움직였을 수 있음. 보정 없이 사용\n",
                        yawDriftDps);
          yawDriftDps = 0;
        }
        yawDriftT0 = millis();
        pinMode(IMU_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(IMU_INT), imuIsr, RISING);
        imuOk = true;
        imuStartMs = millis();
        Serial.printf("IMU OK (DMP %d개/2.5초, DMP 방향 흐름 %.3f 도/초, INT=D4)\n",
                      packets, yawDriftDps);
      }
    }
  }
#endif

#if USE_WIFI
  // 접속이 끊기거나 거절될 때 이유 코드 출력 (2=인증만료 15=비밀번호/4-way 실패 201=AP못찾음 202=인증실패)
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    Serial.printf("WiFi 끊김/거절 reason=%d\n", info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // 절전 끔 -> 키 입력 반응 지연(수십~수백 ms) 방지
  WiFi.setAutoReconnect(true);   // 공유기 연결이 끊기면 자동 재접속
#if USE_STATIC_IP
  WiFi.config(STATIC_IP, GATEWAY, SUBNET, GATEWAY);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (!startNetworkIfConnected()) {
    // 실패로 끝내지 않음: 백그라운드에서 계속 재시도하고, 붙는 순간 loop() 에서 서버/mDNS 시작
    Serial.printf("WiFi 아직 미연결 (status=%d) -> 계속 재시도 중 (USB 유선으로는 동작)\n",
                  (int)WiFi.status());
  }
#endif
  Serial.println("READY");
}

void loop() {
  // ---- USB 유선 ----
  while (Serial.available() > 0) feed(usbBuf, (char)Serial.read(), Serial);

#if USE_WIFI
  // ---- WiFi 무선 (TCP, 클라이언트 1개) ----
  // 새 접속이 오면 '항상' 받아서 기존 연결과 교체함.
  //   WiFi 가 순간 끊기면 ESP32 는 이전 연결이 아직 살아 있다고 착각할 수 있음(상대가 사라진 반쯤 열린 연결).
  //   기존 연결이 끊겨야만 새 접속을 받으면, PC 가 다시 접속해도 한동안 명령이 무시됨 -> 최신 접속을 우선.
  //   (조종하는 PC 는 하나이므로 가장 최근에 접속한 쪽이 현재 PC)
  startNetworkIfConnected();
  if (netStarted) {
    WiFiClient incoming = server.available();
    if (incoming) {
      if (client && client.connected()) {
        client.stop();
        Serial.println("TCP: 새 접속이 와서 이전 연결을 닫고 교체");
      }
      client = incoming;
      client.setNoDelay(true);
      wifiBuf.len = 0;
      client.println("READY");
      char boot[64];
      snprintf(boot, sizeof(boot), "BOOT,%s,%lu", bootReasonName(), millis() / 1000);   // 원인, 켜진 지 몇 초
      client.println(boot);
    }
  }
  while (client && client.available() > 0) feed(wifiBuf, (char)client.read(), client);
#endif

  // ---- IR 장애물 센서: 읽고, 바뀌었거나 주기가 되면 PC 로 보고 ----
  bool changed = updateIR();
  if (changed || millis() - irLastReportMs >= IR_REPORT_MS) {
    irLastReportMs = millis();
    char msg[16];
    snprintf(msg, sizeof(msg), "IR,%d,%d", irLeft ? 1 : 0, irRight ? 1 : 0);
    Serial.println(msg);
#if USE_WIFI
    clientSend(msg);
#endif
  }

  // ---- MPU-6050: 새 데이터 읽기 + 0.05초마다 PC 로 보고 ----
  updateIMU();
  if (imuOk && !imuIntChecked && millis() - imuStartMs > 2000) {   // 시작 2초 뒤 INT 배선 확인 결과 한 번 출력
    imuIntChecked = true;
    unsigned long n = imuIntCount;
    char msg[96];
    if (n > 50) snprintf(msg, sizeof(msg), "IMU INT OK (2초 동안 %lu회)", n);
    else        snprintf(msg, sizeof(msg), "IMU INT 신호 없음 (%lu회) -> 20ms 마다 직접 읽기로 동작. D4 배선 확인", n);
    Serial.println(msg);
#if USE_WIFI
    clientSend(msg);
#endif
  }
  if (imuOk && millis() - imuLastReportMs >= IMU_REPORT_MS) {
    imuLastReportMs = millis();
    char msg[96];
    snprintf(msg, sizeof(msg), "IMU,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%d",
             yawDeg, gzDps, quatW, quatX, quatY, quatZ, headingHolding ? 1 : 0);
    Serial.println(msg);
#if USE_WIFI
    clientSend(msg);
#endif
  }

  // ---- 5초마다 상태 보고: 독립 워치독이 모터를 세운 횟수, WiFi 가 막혀 버린 보고 수, 켜진 시간 ----
  static unsigned long statMs = 0;
  if (millis() - statMs >= 5000) {
    statMs = millis();
    char msg[64];
#if USE_WIFI
    snprintf(msg, sizeof(msg), "STAT,%lu,%lu,%lu", failsafeCount, wifiDropCount, millis() / 1000);
#else
    snprintf(msg, sizeof(msg), "STAT,%lu,0,%lu", failsafeCount, millis() / 1000);
#endif
    Serial.println(msg);
#if USE_WIFI
    clientSend(msg);
#endif
  }

  // ---- 모터 출력: 저장된 명령 + 워치독(0.5초) + 장애물 시 전진 차단 + 직선 보정 + 가속 제한 ----
  applyOutput();
}
