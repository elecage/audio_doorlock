# ESP32-C3 Matter Servo Door Lock

ESP-IDF project for turning an ESP32-C3 Super Mini into a Wi-Fi Matter door lock that drives an RC servo with PWM.

The firmware exposes two Matter endpoints:

- A standard Door Lock endpoint for Google Home app control.
- An auxiliary On/Off Plug-in Unit endpoint for voice-triggered opening through Google Assistant.

Google Assistant/Nest Mini blocks voice unlock commands for real Door Lock devices for security reasons. The auxiliary switch can be renamed in Google Home, for example to `Front door button`, and used as a momentary open trigger. When the switch turns on, the servo moves to the unlocked position and automatically returns to the locked position after 5 seconds.

## Hardware

| RC Servo | ESP32-C3 Super Mini |
| --- | --- |
| Signal | GPIO4 |
| VCC | External 5 V supply |
| GND | Common GND with ESP32-C3 |

Do not power the servo directly from the ESP32-C3 3.3 V pin. Even small servos such as SG90 can draw enough peak current to reset the board.

## Features

- Target: ESP32-C3
- Matter device type: Door Lock
- Auxiliary voice trigger: On/Off Plug-in Unit
- Matter hub tested with: Google Nest Mini 2nd generation + Google Home
- Servo PWM: LEDC low-speed mode, 50 Hz
- Servo signal pin: GPIO4
- Auto-relock delay for voice trigger: 5 seconds

## Servo Calibration

Adjust these values in [main/servo_lock.cpp](main/servo_lock.cpp) for your lock mechanism:

```cpp
constexpr int kLockedAngle = 15;
constexpr int kUnlockedAngle = 105;
constexpr int kMinPulseUs = 500;
constexpr int kMaxPulseUs = 2500;
```

Current pulse widths:

- Locked: about 666 us
- Unlocked: about 1666 us

## Build and Flash

This project uses ESP-IDF v5.4.1 and the `espressif/esp_matter` managed component.

```powershell
idf.py set-target esp32c3
idf.py build
idf.py -p COM10 flash monitor
```

## Google Home Pairing

After a fresh flash or NVS erase, the device opens a Matter commissioning window and advertises over BLE as:

```text
MATTER-3840
```

Use Google Home to add a new Matter device. Manual pairing code:

```text
34970112332
```

If Google Home asks for a setup PIN:

```text
20202021
```

## Voice Trigger

After pairing, rename the auxiliary switch endpoint in Google Home to a phrase that Google Assistant will treat as a switch, for example:

```text
Front door button
```

Then say:

```text
Hey Google, turn on Front door button
```

The servo will unlock, wait 5 seconds, and lock again.

## Re-pairing

If Google Home keeps the old device offline after firmware or endpoint changes, delete the old device in Google Home and erase the ESP32-C3 NVS partition:

```powershell
python $env:IDF_PATH\components\esptool_py\esptool\esptool.py --chip esp32c3 -p COM10 erase_region 0x9000 0x6000
```

Reboot the ESP32-C3 and pair it again as a new Matter device.

## Safety

This is a development prototype. Before using it on a real door, review manual override, power-loss behavior, mechanical safety, state sensing, access control, and any Matter certification requirements.

## References

- [Espressif ESP-Matter Programming Guide for ESP32-C3](https://documentation.espressif.com/esp-matter/en/latest/esp32c3/index.html)
- [Espressif ESP-Matter component registry](https://components.espressif.com/components/espressif/esp_matter)
- [ESP-IDF LEDC for ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/ledc.html)
- [Google Home Matter hub requirements](https://support.google.com/googlenest/answer/12391458)
