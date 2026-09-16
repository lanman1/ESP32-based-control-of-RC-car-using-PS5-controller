# GraveDig RC Tank Controller

GraveDig is an ESP32-based replacement control system for a tank-style RC vehicle with failed original electronics.

The project uses an original-generation ESP32 with Bluetooth Classic support, a Cytron MDD3A dual brushed-motor driver, a PS5 DualSense controller, WS2811/WS2812 addressable LEDs, and an onboard Wi-Fi configuration interface.

The current firmware revision is:

**Firmware Version: 0.4**

---

## Features

* PS5 DualSense control over Bluetooth
* Independent left and right track control
* Variable-speed brushed DC motor control
* Cytron MDD3A dual motor driver support
* Software-selectable low and normal speed profiles
* Adjustable acceleration and deceleration ramping
* Adjustable joystick response curve
* Adjustable joystick dead zone
* Independent left/right motor trim
* Software motor direction reversal
* Emergency motor shutdown
* Safe controller-disconnect behavior
* WS2811 / WS2812 status LED support
* Wi-Fi SoftAP configuration interface
* Persistent settings using ESP32 NVS
* Vehicle battery voltage monitoring
* Low and critical battery warnings
* DualSense lightbar battery warnings
* DualSense rumble battery warnings
* Optional critical-battery motor power limiting or shutdown
* Future GPIO allocation for servos and lighting expansion

---

# Hardware

## Main Controller

Recommended:

* ESP32-WROOM-32
* ESP32-WROOM-32E

The ESP32 must support **Bluetooth Classic / BR/EDR** for direct communication with a PS5 DualSense controller.

Boards based on ESP32-S3, ESP32-C3, ESP32-C6, and similar BLE-only variants are not suitable for the current DualSense implementation.

---

## Motor Driver

**Cytron MDD3A**

Specifications relevant to this project:

* 2 independent brushed DC motor channels
* approximately 3 A continuous per channel
* approximately 5 A peak per channel
* variable-speed PWM control
* 3.3 V-compatible logic
* suitable for differential/tank drive

---

## Controller

**Sony PS5 DualSense**

The controller connects directly to the ESP32 over Bluetooth Classic using Bluepad32.

---

## Addressable LEDs

The firmware supports WS2811 / WS2812-compatible addressable LEDs.

Pixel `0` is reserved for system status.

Pixels `1` and above are reserved for future vehicle lighting functions.

---

# GPIO Assignment

| Function                     | ESP32 GPIO |
| ---------------------------- | ---------: |
| MDD3A Motor 1 A              |    GPIO 17 |
| MDD3A Motor 1 B              |    GPIO 18 |
| MDD3A Motor 2 A              |    GPIO 22 |
| MDD3A Motor 2 B              |    GPIO 23 |
| WS2811 / WS2812 Data         |    GPIO 25 |
| Future Servo 1               |    GPIO 26 |
| Future Servo 2               |    GPIO 27 |
| Vehicle Battery ADC          |    GPIO 34 |
| Emergency Abort / PRG Button |     GPIO 0 |

For a final custom installation, GPIO 0 may be replaced with a normal GPIO such as GPIO 32 or GPIO 33 for the emergency input.

---

# Motor Driver Wiring

## ESP32 to MDD3A Control Header

| ESP32   | MDD3A |
| ------- | ----- |
| GPIO 17 | M1A   |
| GPIO 18 | M1B   |
| GPIO 22 | M2A   |
| GPIO 23 | M2B   |
| GND     | GND   |

The MDD3A `5Vo` output is not used to power the ESP32.

A dedicated regulated 5 V supply is recommended for the ESP32.

---

## MDD3A Motor and Power Terminals

```text
M1A terminal ─┐
              ├── Left track motor
M1B terminal ─┘

VB+ ───────────── Battery positive
VB- ───────────── Battery negative

M2A terminal ─┐
              ├── Right track motor
M2B terminal ─┘
```

All system grounds must share a common reference.

---

# Power Architecture

Recommended power layout:

```text
Vehicle Battery
      |
      +----> Cytron MDD3A ----> Left / Right Drive Motors
      |
      +----> 5 V Regulator ---> ESP32
      |
      +----> 5-6 V Regulator -> Future RC Servos
      |
      +----> 5 V Lighting ----> WS2811 / WS2812 LEDs
```

Do not power normal hobby servos directly from the ESP32 board.

Standard RC servos can receive their control signal directly from an ESP32 GPIO, but should use a separate properly sized 5-6 V power supply.

All grounds must remain common.

---

# PS5 DualSense Controls

## Driving

| Control            | Function                              |
| ------------------ | ------------------------------------- |
| Left Stick Y       | Left track forward / reverse          |
| Right Stick Y      | Right track forward / reverse         |
| OPTIONS            | Arm / disarm motors                   |
| L1                 | Low-speed profile                     |
| R1                 | Normal-speed profile                  |
| OPTIONS + Triangle | Enter / exit Wi-Fi configuration mode |

The vehicle cannot be armed unless both joysticks are near center.

---

# Motor Safety

The firmware includes several motor safety behaviors.

## Startup

Motors always start in the **SAFE / DISARMED** state.

## Controller Disconnect

If the DualSense disconnects:

* both motor commands immediately go to zero
* the vehicle becomes disarmed
* a controller reconnection does not automatically re-arm the vehicle

## Configuration Mode

Entering Wi-Fi configuration mode automatically:

* disarms the vehicle
* stops both motors
* prevents motor arming while configuration mode is active

## Emergency Abort

The development-board PRG button currently acts as an emergency abort.

Once activated:

* both motors immediately stop
* the vehicle remains aborted until the ESP32 is reset

---

# Speed Profiles

Default values:

| Profile      | Default |
| ------------ | ------: |
| Low Speed    |     30% |
| Normal Speed |     50% |

These values can be changed through the web configuration interface.

---

# Acceleration and Deceleration

The firmware ramps motor output instead of instantly applying joystick commands.

Default values:

| Setting      | Default |
| ------------ | ------: |
| Acceleration |  800 ms |
| Deceleration |  500 ms |

This reduces:

* drivetrain shock
* motor current spikes
* track slip
* abrupt starts and stops

Motor direction changes also ramp through zero before reverse power is applied.

---

# Joystick Response Curves

The web interface supports four response curves:

* Linear
* Soft
* Medium
* Aggressive

Default:

**Medium**

A softer response curve provides finer low-speed control near the center of the joystick while still allowing full output near maximum stick travel.

---

# Motor Trim

Each track can be independently trimmed.

Default:

```text
Left Motor Trim:   100%
Right Motor Trim:  100%
```

This allows compensation for differences between:

* motors
* gearboxes
* track tension
* drivetrain friction

The range is currently:

```text
50% - 120%
```

---

# Motor Reversal

The web interface includes:

* Reverse Left Motor
* Reverse Right Motor

This allows motor direction to be corrected in software without physically swapping motor wires.

---

# Status Pixel

Pixel `0` is reserved for system status.

Current status behavior:

| Color / Pattern | Meaning                                 |
| --------------- | --------------------------------------- |
| Blue pulse      | Waiting for controller                  |
| Solid blue      | Controller connected, motors safe       |
| Green           | Motors armed                            |
| Flashing yellow | Wi-Fi activation combination being held |
| Yellow          | Wi-Fi configuration active              |
| Orange          | Low vehicle battery                     |
| Red             | Critical vehicle battery                |
| Flashing red    | Emergency abort                         |

Battery warnings take priority over normal drive status.

---

# Wi-Fi Configuration Interface

The ESP32 hosts its own Wi-Fi network when configuration mode is enabled.

Default network:

```text
SSID: GraveDig
Password: GraveDig32
```

Configuration page:

```text
http://192.168.4.1
```

The default password should be changed before final deployment.

---

# Entering Configuration Mode

Configuration mode requires an intentional button combination.

Default behavior:

1. Vehicle must be disarmed.
2. Both joysticks must be centered.
3. Hold `OPTIONS + Triangle`.
4. Continue holding for 3 seconds.
5. The status pixel / controller light flashes yellow during the hold.
6. The `GraveDig` Wi-Fi network starts.
7. Motors remain disabled while Wi-Fi is active.

The same button combination can shut configuration mode down.

---

# Wi-Fi Startup Access Window

By default, Wi-Fi configuration mode can only be started during the first:

```text
60 seconds
```

after power-up.

This reduces the chance of accidentally enabling Wi-Fi while operating the vehicle.

The access window is configurable.

Setting the value to:

```text
0
```

allows Wi-Fi configuration mode to be entered at any time while the vehicle is SAFE.

---

# Wi-Fi Idle Timeout

Default:

```text
300 seconds
```

The configuration page polls the ESP32 while it remains open.

The idle timeout begins once browser activity stops.

Wi-Fi can also be shut down using:

* `OPTIONS + Triangle`
* the **Shut Down Wi-Fi** button in the web interface

---

# Web Configuration Settings

The current web interface provides configuration for:

## Drive

* Low-speed limit
* Normal-speed limit
* Stick dead zone
* Response curve
* Acceleration time
* Deceleration time
* Left motor trim
* Right motor trim
* Reverse left motor
* Reverse right motor

## Lighting

* Master pixel brightness

Pixel 0 remains reserved for system status.

## Battery

* Enable / disable vehicle battery monitoring
* Low voltage threshold
* Critical voltage threshold
* Recovery hysteresis
* ADC calibration multiplier
* Threshold confirmation time
* Critical battery action
* Critical motor power limit
* Status pixel warning enable
* DualSense lightbar warning enable
* DualSense rumble warning enable

## Wi-Fi

* Idle timeout
* Startup access window
* OPTIONS + Triangle hold duration

## System

* Current firmware version
* Controller status
* DualSense battery level
* Drive state
* Current speed profile
* Motor output
* Vehicle battery voltage
* Vehicle battery state
* Wi-Fi client count
* System uptime
* Restore factory defaults
* Shut down Wi-Fi

All saved settings are retained in ESP32 NVS.

---

# Vehicle Battery Monitoring

Vehicle battery monitoring uses GPIO 34.

Monitoring is disabled by default until the required voltage divider is installed.

Recommended divider:

```text
Vehicle Battery +
       |
      150K
       |
       +---------- GPIO34
       |
       33K
       |
      GND
```

Recommended additional capacitor:

```text
GPIO34 ---- 0.1 uF ---- GND
```

The divider ratio is approximately:

```text
5.545 : 1
```

This is suitable for common RC battery voltages including packs up to approximately 4S when used with the configured ESP32 ADC input.

Always verify the actual ADC voltage before connecting higher-voltage battery packs.

---

# Battery Monitoring Behavior

The firmware filters battery voltage readings to reduce false alarms caused by:

* motor startup current
* hard acceleration
* tank pivoting
* electrical noise

A battery threshold must remain exceeded for a configurable confirmation period before the state changes.

Default:

```text
3 seconds
```

---

# Battery States

The firmware uses three vehicle battery states:

```text
NORMAL
LOW
CRITICAL
```

Default example thresholds:

```text
Low:       7.20 V
Critical:  6.80 V
Hysteresis: 0.15 V
```

These defaults are intended only as example values and should be changed to match the actual battery chemistry and pack configuration.

---

# Battery Warning Feedback

## Low Battery

Default behavior:

* status pixel becomes orange
* DualSense lightbar becomes orange
* controller gives a double rumble notification
* rumble reminder approximately every 60 seconds

## Critical Battery

Default behavior:

* status pixel becomes red
* DualSense lightbar becomes red
* controller gives a stronger triple rumble notification
* rumble reminder approximately every 15 seconds

---

# Critical Battery Behavior

The web interface provides three options.

## Warn Only

The vehicle continues operating normally.

This is the recommended setting during initial testing.

## Limit Motor Power

Maximum drive output is reduced to a configurable percentage.

Default critical limit:

```text
30%
```

## Disable Drive

The vehicle immediately becomes disarmed and cannot be rearmed until battery voltage recovers above the hysteresis threshold.

---

# Battery Calibration

Use a multimeter to compare the actual pack voltage to the voltage displayed by GraveDig.

Example:

```text
Multimeter:       8.02 V
GraveDig reports: 7.88 V
```

Calculate:

```text
8.02 / 7.88 = 1.018
```

Enter:

```text
Battery Calibration Multiplier: 1.018
```

into the web configuration interface.

---

# DualSense Battery Monitoring

The web interface also displays the battery level reported by the PS5 DualSense controller when available.

Future firmware may add separate notifications for a low controller battery.

---

# Persistent Configuration

Settings are stored using the ESP32 `Preferences` library and NVS.

Settings survive:

* ESP32 restart
* vehicle power-off
* battery replacement

The web interface also includes a **Restore Factory Defaults** option.

---

# Required Arduino Libraries

The current firmware uses:

* Bluepad32
* WiFi
* WebServer
* Preferences
* Adafruit NeoPixel

The ESP32 board package must support the original ESP32 / ESP32-WROOM family.

For PS5 DualSense support, use the Bluepad32-compatible ESP32 Arduino environment.

---

# Bluetooth Pairing

For the first DualSense pairing:

1. Power on GraveDig.
2. Hold the DualSense **Create** button.
3. While holding Create, hold the **PS** button.
4. Wait for the controller LEDs to flash rapidly.
5. Bluepad32 should discover and connect to the controller.

Pairing information is retained.

Do not call:

```cpp
BP32.forgetBluetoothKeys();
```

during normal startup unless intentionally clearing controller pairing data.

---

# WS2811 / WS2812 Electrical Notes

The ESP32 outputs 3.3 V logic.

Many 5 V WS281x LEDs will work with a 3.3 V data signal over short wiring, but a level shifter is recommended for reliable operation.

Recommended devices include:

* 74AHCT125
* 74HCT14
* similar 3.3 V-to-5 V logic translators

The LED supply should be sized independently for the number of pixels installed.

All LED and ESP32 grounds must be connected.

---

# Future Servo Support

The current GPIO plan reserves:

```text
GPIO 26 - Servo 1
GPIO 27 - Servo 2
```

Standard hobby RC servos do not require an external motor driver.

Each servo requires:

```text
ESP32 GPIO ------ Servo signal
5-6 V supply ---- Servo power
Common ground --- Servo ground
```

Do not power full-size servos from the ESP32 development board.

For larger numbers of servos, an I2C PWM controller such as a PCA9685 may be added later.

---

# Planned Lighting Features

Pixels `1` and above are currently reserved for future lighting functions.

Possible future features include:

* headlights
* taillights
* brake lights
* reverse lights
* turn signals
* running lights
* startup animations
* scanner effects
* configurable effect patterns
* brightness profiles
* lighting linked to throttle or direction

---

# Planned Future Improvements

Potential future firmware additions include:

* configurable pixel assignments
* dedicated lighting engine
* servo configuration interface
* servo endpoint limits
* servo centering and reversing
* battery percentage estimation by battery chemistry
* DualSense low-battery warning
* configuration export/import
* JSON backup of settings
* firmware update support
* physical configuration button
* dedicated emergency-stop GPIO
* custom GraveDig PCB
* current sensing
* motor stall detection
* temperature monitoring
* optional telemetry logging

---

# Safety

This project controls electric motors capable of unexpectedly moving the vehicle.

During development:

* raise tracks or wheels off the ground
* use conservative speed limits
* verify joystick direction before ground testing
* provide appropriate motor and battery fusing
* size wiring for expected motor current
* use a proper regulator for the ESP32
* use a separate suitable power supply for servos and high-current lighting
* ensure all grounds are common
* test controller-disconnect behavior before driving
* test battery warnings before relying on them
* keep an accessible physical power disconnect

The software emergency stop is not a substitute for a physical battery disconnect or correctly sized fuse.

---

# Current Default Configuration

```text
Firmware:                  0.4

Low Speed:                 30%
Normal Speed:              50%

Stick Dead Zone:           55
Response Curve:            Medium

Acceleration:              800 ms
Deceleration:              500 ms

Left Motor Trim:           100%
Right Motor Trim:          100%

Reverse Left Motor:        Off
Reverse Right Motor:       Off

Pixel Brightness:          80 / 255

Wi-Fi SSID:                GraveDig
Wi-Fi Password:            GraveDig32
Wi-Fi Idle Timeout:        300 sec
Wi-Fi Startup Window:      60 sec
Wi-Fi Hold Time:           3 sec

Battery Monitoring:        Disabled by default
Low Battery:               7.20 V
Critical Battery:          6.80 V
Battery Hysteresis:        0.15 V
Battery Confirmation:      3 sec
Battery Calibration:       1.000

Critical Battery Action:   Warn Only
Critical Power Limit:      30%
```

---

# Project Status

The following functions have been successfully bench-tested in earlier development revisions:

* ESP32 to MDD3A communication
* left motor forward / reverse
* right motor forward / reverse
* variable PWM motor speed control
* independent track control
* original onboard OLED test interface

The current firmware replaces the OLED interface with:

* WS281x system-status indication
* DualSense lightbar feedback
* DualSense rumble feedback
* browser-based configuration

Further vehicle-level testing is ongoing.

---

# License

Add your preferred project license here.

For an open-source hardware/software project, common choices include:

* MIT License
* Apache License 2.0
* GNU GPLv3

---

# Disclaimer

This project is provided for experimental and hobby use.

Battery systems, motors, motor drivers, wiring, and mechanical systems can produce significant current, heat, and movement. Verify component ratings and incorporate appropriate electrical and mechanical safety protections for your specific vehicle.




