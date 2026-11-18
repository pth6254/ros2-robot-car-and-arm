// ESP32 가 접속할 WiFi 정보 (robot_car_motor_esp32.ino 가 읽음)
//
// 사용법: 이 파일을 같은 폴더에 wifi_config.h 로 복사한 뒤 값을 채우세요.
//     cp wifi_config.example.h wifi_config.h
// wifi_config.h 는 .gitignore 에 들어 있어서 커밋되지 않습니다.
//
// ※ ESP32 는 2.4GHz WiFi 만 지원합니다. 5GHz 공유기 이름(SSID)을 쓰면 접속되지 않습니다.
//   PC 는 같은 공유기의 5GHz 쪽에 연결되어 있어도 통신됩니다.

#pragma once

#define WIFI_SSID_VALUE  "YOUR_2.4GHZ_SSID"   // 2.4GHz WiFi 이름
#define WIFI_PASS_VALUE  "YOUR_PASSWORD"      // WiFi 비밀번호
