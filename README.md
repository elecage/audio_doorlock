# ESP32-C3 Matter Servo Door Lock

ESP32-C3 Super Mini를 Wi-Fi Matter 도어락 디바이스로 만들고, RC 서보모터를 PWM으로 제어해 잠금/잠금해제를 수행하는 ESP-IDF 프로젝트입니다.

Google Home/Nest Mini에서 Matter Door Lock으로 등록할 수 있고, Google Assistant의 도어락 음성 해제 제한을 피하기 위한 별도 On/Off 스위치 endpoint도 함께 제공합니다. 이 스위치는 켜짐 명령을 받으면 서보를 열림 위치로 이동한 뒤 5초 후 자동으로 닫힘 위치로 돌아갑니다.

## 기능

- Matter device type: Door Lock
- Matter hub/controller: Google Nest Mini 2세대 + Google Home
- 음성 트리거용 추가 endpoint: On/Off Plug-in Unit
- 서보 제어: ESP32-C3 LEDC PWM, 50 Hz
- 서보 신호 핀: GPIO4
- 음성 트리거 자동 재잠금: 5초

## 배선

| RC Servo | ESP32-C3 Super Mini |
| --- | --- |
| Signal | GPIO4 |
| VCC | 외부 5 V 전원 |
| GND | 외부 전원 GND와 ESP32-C3 GND 공통 |

서보 전원은 ESP32-C3의 3.3 V 핀에서 직접 공급하지 않는 것을 권장합니다. SG90 같은 소형 서보도 순간 전류가 커서 보드가 리셋될 수 있습니다.

## 서보 각도 보정

[main/servo_lock.cpp](main/servo_lock.cpp)에서 실제 문고리/잠금장치에 맞게 아래 값을 조정합니다.

```cpp
constexpr int kLockedAngle = 15;
constexpr int kUnlockedAngle = 105;
constexpr int kMinPulseUs = 500;
constexpr int kMaxPulseUs = 2500;
```

현재 PWM 설정은 다음과 같습니다.

- GPIO: GPIO4
- Frequency: 50 Hz
- Duty resolution: 14-bit
- Locked pulse: 약 666 us
- Unlocked pulse: 약 1666 us

## 빌드

ESP-IDF v5.4.1과 ESP-Matter component를 사용합니다.

```powershell
idf.py set-target esp32c3
idf.py build
idf.py -p COM10 flash monitor
```

프로젝트는 `idf_component.yml`을 통해 `espressif/esp_matter` dependency를 받습니다.

## Google Home 등록

처음 부팅하거나 NVS를 지운 뒤에는 commissioning window가 열리고 BLE 이름 `MATTER-3840`으로 광고됩니다.

Google Home 앱에서 Matter 기기를 추가하고, 수동 코드를 입력합니다.

```text
34970112332
```

PIN을 직접 물으면 다음 값을 사용합니다.

```text
20202021
```

## Google Assistant 음성 제어

Google Assistant/Nest Mini는 보안 정책상 Door Lock의 음성 잠금해제를 차단합니다. 그래서 이 펌웨어는 도어락 endpoint와 별도로 음성 트리거용 On/Off 스위치 endpoint를 추가합니다.

Google Home 앱에서 새 스위치 이름을 예를 들어 `현관문 버튼`처럼 바꾼 뒤 다음처럼 사용할 수 있습니다.

```text
Hey Google, 현관문 버튼 켜줘
```

스위치가 켜지면 서보가 열림 위치로 이동하고, 5초 뒤 자동으로 닫힘 위치로 돌아갑니다.

## 재페어링

Google Home에 장치가 오프라인으로 남거나 endpoint 구성이 꼬이면 기존 장치를 Google Home에서 삭제하고 ESP32-C3의 NVS 영역을 지운 뒤 다시 페어링합니다.

```powershell
python $env:IDF_PATH\components\esptool_py\esptool\esptool.py --chip esp32c3 -p COM10 erase_region 0x9000 0x6000
```

그 후 장치를 재부팅하고 Google Home에서 새 Matter 기기로 등록합니다.

## 안전 참고

이 프로젝트는 개발/실험용 예제입니다. 실제 현관문에 적용하려면 수동 해제 구조, 전원 장애 대응, 물리적 안전성, 상태 센서, 인증/인터락, Matter 인증 요구사항을 별도로 검토해야 합니다.

## 참고 문서

- [Espressif ESP-Matter Programming Guide for ESP32-C3](https://documentation.espressif.com/esp-matter/en/latest/esp32c3/index.html)
- [Espressif ESP-Matter component registry](https://components.espressif.com/components/espressif/esp_matter)
- [ESP-IDF LEDC for ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/ledc.html)
- [Google Home Matter hub requirements](https://support.google.com/googlenest/answer/12391458)
