# ESP RC Tank Controller

ESP RC is an ESP32-based replacement control system for a tank-style RC vehicle with failed original electronics.

The project uses an original-generation ESP32 with Bluetooth Classic support, a Cytron MDD3A dual brushed-motor driver, a PS5 DualSense controller, WS2811/WS2812 addressable LEDs, and an onboard Wi-Fi configuration interface.

The current firmware revision is:

**Firmware Version: 0.7.0**

**Hardware tested and mostly functional.** Known Wi-Fi / Bluetooth coexistence, pairing / reconnection and DualSense LED issues remain. See [Project Status](#project-status) and the [issue tracker](docs/ISSUES.md).

See [CHANGELOG.md](CHANGELOG.md) for revision history.

---

## Features

* PS5 DualSense control over Bluetooth
* Selectable tank and arcade drive with persistent settings
* Proportional steering or pivot steering with configurable sensitivity
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
| MDD3A Motor 1 A              |    GPIO 25 |
| MDD3A Motor 1 B              |    GPIO 26 |
| MDD3A Motor 2 A              |    GPIO 27 |
| MDD3A Motor 2 B              |    GPIO 14 |
| WS2811 / WS2812 Data         |     GPIO 4 |
| Future Servo 1               |    GPIO 32 |
| Future Auxiliary Output      |    GPIO 33 |
| Vehicle Battery ADC          |    GPIO 34 |
| Emergency Abort / PRG Button |     GPIO 0 |

Firmware 0.6.0 and later use this WROOM mapping. The web interface shows the same table (GPIO Map card), generated from the firmware's pin definitions. Firmware 0.5 and earlier used GPIO 17/18/22/23 for the motors and GPIO 25 for the LEDs; rewire before flashing 0.6.0 onto an older build.

For a final custom installation, GPIO 0 may be replaced with a spare general-purpose GPIO for the emergency input.

---

# Motor Driver Wiring

## ESP32 to MDD3A Control Header

| ESP32   | MDD3A |
| ------- | ----- |
| GPIO 25 | M1A   |
| GPIO 26 | M1B   |
| GPIO 27 | M2A   |
| GPIO 14 | M2B   |
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
| Left Stick Y       | Tank: left track; arcade: both tracks throttle |
| Right Stick Y      | Tank: right track forward / reverse |
| Right Stick X      | Arcade: left / right steering |
| OPTIONS            | Arm (on release) / disarm (on press)  |
| PS                 | E-stop: stop motors and disarm (never arms) |
| L1                 | Low-speed profile                     |
| R1                 | Normal-speed profile                  |
| OPTIONS + Triangle | Enter / exit Wi-Fi configuration mode |
| Create + PS        | Pairing mode                          |

Arming happens when OPTIONS is **released**, and only if Triangle was not pressed at any point during that press, so the OPTIONS + Triangle combination can never arm the vehicle, whichever button goes down first. Disarming happens as soon as OPTIONS is pressed. The vehicle cannot be armed unless all active drive axes are inside the configured deadband. After reconnecting or leaving configuration mode, release OPTIONS and Triangle before pressing OPTIONS to arm.

A refused arm is reported by rumble and on Serial: **2 pulses** means the sticks or triggers are not neutral, **3 pulses** means battery protection, Wi-Fi configuration mode or an emergency abort is blocking drive.

### Drive modes

Tank mode is the default: left Y controls the left track, right Y controls the right track. Arcade mode uses left Y for throttle and right X for steering. Positive/right steering produces right yaw, including while reversing.

Proportional steering scales steering by throttle magnitude, so a centered throttle cannot start a pivot. Pivot steering permits opposite track directions at zero throttle. Steering sensitivity is adjustable from 0–200%; the steering input is clamped before mixing. Inline SVG diagrams in the web UI explain both layouts without internet access.

The mixer normalizes both outputs after trim, preserving their ratio and staying within the lower of the selected speed profile and maximum output setting. A critical-battery limit can reduce that cap further. When a limit is reduced (L1 or the critical limit), the output ramps down to the new cap at the deceleration rate instead of dropping instantly; disarm, disconnect and e-stop still stop immediately. Existing acceleration/deceleration and motor reversal remain available.

---

# Motor Safety

The firmware includes several motor safety behaviors.

## Startup

Motors always start in the **SAFE / DISARMED** state.

## Controller Initialization

A Bluetooth connection does not by itself allow driving. On every connect, reconnect, resumed report stream, or exit from configuration mode, the controller enters an **INITIALIZING** state:

* a 1 second Bluetooth settle period
* all sticks inside the deadband and both triggers released
* neutral held steady for 250 ms
* stale button-edge state cleared

The controller then becomes **READY**, and the vehicle remains disarmed until an explicit OPTIONS press. The status pixel and lightbar pulse cyan while initializing.

Only one controller is accepted at a time. A second controller that connects is disconnected, and new Bluetooth connections are disabled while a controller is active; they are re-enabled when it disconnects.

## Controller Disconnect

If the DualSense disconnects:

* both motor commands immediately go to zero
* the vehicle becomes disarmed
* a controller reconnection does not automatically re-arm the vehicle
* if controller reports stop for **300 ms**, both motors stop at once but the vehicle stays armed; when reports resume, drive stays at zero until the sticks and triggers return to neutral
* a two-second controller-data timeout also stops and disarms the vehicle if the radio stops delivering reports before the Bluetooth stack reports a disconnect; when reports resume, the controller must pass initialization again

## Configuration Mode

Entering Wi-Fi configuration mode automatically:

* disarms the vehicle
* stops both motors
* prevents motor arming while configuration mode is active

## Controller E-Stop

Pressing **PS** while armed stops both motors immediately, disarms the vehicle and gives one long, strong rumble. It is stop-only: PS never arms. Re-arm with OPTIONS as usual. (While no controller is connected, PS wakes a paired DualSense as before.)

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
| Fast flashing red | Emergency abort                       |
| Flashing purple | Wi-Fi activation combination being held |
| Purple          | Wi-Fi configuration active              |
| Slow flashing red | Drive locked out by battery protection (critical latched, no reading, or not yet qualified) |
| Red             | Critical vehicle battery                |
| Orange          | Low vehicle battery                     |
| Blue pulse      | Waiting for controller (status LED only) |
| Cyan pulse      | Controller initializing / waiting for neutral |
| Green           | Motors armed                            |
| Solid blue      | Controller connected, motors safe       |

The table is in priority order: when more than one applies, the higher row wins. Wi-Fi states are shown above battery warnings so configuration mode is always visible. The DualSense lightbar uses the same colors; the pixel and lightbar battery warnings are enabled separately, the lightbar is off until a controller connects, and the brightness setting applies to the pixel only. The web interface includes the same legend.

---

# Wi-Fi Configuration Interface

The ESP32 hosts its own Wi-Fi network when configuration mode is enabled.

Default network:

```text
SSID: ESPRC
Password: ESPRC123
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
5. The status pixel / controller light flashes purple during the hold, then stays purple while configuration is active.
6. The `ESPRC` Wi-Fi network starts.
7. Motors remain disabled while Wi-Fi is active.

The same button combination can shut configuration mode down.

---

# Wi-Fi Access

Configuration mode can be entered at any time after boot while disarmed, with the active drive axes centered. Hold OPTIONS + Triangle for the configured duration (default 3 seconds). The former 60-second startup window has been removed, including its web field and NVS setting; an old saved window is ignored.

Wi-Fi remains off at startup. Entry is rejected while armed or emergency-aborted. Serial output reports AP startup/failure, SSID, IP address, station count changes, inactivity shutdown, and manual shutdown. The firmware uses SSID `ESPRC` and password `ESPRC123` in the actual AP configuration.

Leaving configuration mode (controller combination, web button or idle timeout) is staged: the request is recorded, then the web server stops, the access point disconnects, and the radio turns off, with a short pause between each step. Each step is logged on Serial, along with free heap at the request. This replaces the 0.6.0 back-to-back shutdown that could crash with `Stack canary watchpoint triggered (sys_evt)`.

---

## Current Wi-Fi / Bluetooth workaround (0.7.0)

Hardware testing of 0.7.0 found that SoftAP association is unreliable while the DualSense remains connected. Reliability improves when Wi-Fi configuration mode is enabled first and the DualSense is then powered off or disconnected before joining the AP. This manual workaround is cumbersome; 0.7.0 does not automatically suspend Bluetooth before starting SoftAP.

The existing 0.7.0 entry method still requires a connected controller for **OPTIONS + Triangle**. When using that method, enter configuration mode first, then power off the DualSense before attempting to join **ESPRC** from the phone or computer. Powering off the controller does not itself enter configuration mode. With the controller off, use the web **Shut Down Wi-Fi** button or the existing idle timeout to exit. Reconnect the controller afterward, allow the normal initialization / neutral validation to complete, and explicitly arm with OPTIONS; reconnection never auto-arms.

Automatic Bluetooth suspension and restoration is planned under [F8 in the issue tracker](docs/ISSUES.md#f8---wi-fi--bluetooth-coexistence-during-configuration).

# Wi-Fi Idle Timeout

Default:

```text
300 seconds
```

The idle timeout counts from the last user action on the configuration page: loading the page, saving, restoring defaults or clearing pairing. The page's live status polling does not count, so Wi-Fi times out even if the page is left open.

Wi-Fi can also be shut down using:

* `OPTIONS + Triangle`
* the **Shut Down Wi-Fi** button in the web interface

---

# Web Configuration Settings

The current web interface provides configuration for:

## Drive

* Tank / arcade mode
* Proportional / pivot steering
* Steering sensitivity
* Maximum motor output
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
* OPTIONS + Triangle hold duration

## Reference cards

* Battery divider wiring diagram
* Controller controls and Create + PS pairing diagram
* OPTIONS + Triangle combination diagram
* Status indicator color legend
* GPIO map

## System

* Current firmware version
* Controller status
* DualSense battery level
* Drive state
* Current speed profile
* Motor output
* Vehicle battery voltage
* Vehicle battery state
* Active protection (battery lockout, power limit, stale-data stop)
* Wi-Fi client count
* System uptime
* Last reset reason
* Restore factory defaults
* Shut down Wi-Fi

All saved settings are retained in ESP32 NVS. Form values are rendered from current settings, including selected options and checkboxes. Invalid, missing, nonnumeric, nonfinite, or out-of-range numerical values reject the entire save, and the error names the failing field or rule; critical voltage must be lower than warning voltage. Corrupt saved settings restore safe defaults with battery monitoring and critical shutdown enabled. Settings are stored in the `esprc` NVS namespace. Firmware 0.6.0 and earlier used a different namespace that is not migrated, so the first boot after updating from those versions starts from factory defaults; re-enter any customized settings. Bluetooth pairing is stored separately and is not affected. New settings default to tank mode, proportional steering, 100% sensitivity, and 100% maximum output.

---

# Vehicle Battery Monitoring

Vehicle battery monitoring uses GPIO 34.

Monitoring is disabled by default until the required voltage divider is installed.

Recommended divider:

```text
Vehicle Battery +
       |
      100K
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
4.0303 : 1
```

This firmware configuration is for **2S LiPo only**. At full charge (8.4 V), the ADC receives approximately `8.4 × 33 / 133 = 2.084 V`. GPIO34 is an ADC1 input, so battery measurement remains available during Wi-Fi configuration.

Version 0.5 changed the firmware divider constant from 150 kΩ / 33 kΩ to **100 kΩ / 33 kΩ**. Verify the physical divider matches before enabling monitoring; a board still fitted with 150 kΩ must be changed or use matching firmware constants. The battery ADC remains on GPIO 34 in 0.6.0.

---

# Battery Monitoring Behavior

The firmware averages 12 calibrated ADC millivolt readings every 100 ms, applies the divider/calibration factor, then uses exponential filtering, hysteresis, and a configurable qualification time. Initial low/critical readings must qualify too. When monitoring is disabled, ADC-based warnings and protective actions are disabled and queued battery rumble is cancelled when the setting is saved. With critical shutdown selected, an unknown/unconnected battery reading also prevents arming. If monitoring is enabled but GPIO 34 reads under 0.5 V, Serial reports that no voltage is present, the web status shows "No reading on GPIO 34", and the status light flashes red slowly. Fit the divider or disable monitoring.

These measures reduce false alarms caused by:

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

Maximum drive output is reduced to a configurable percentage. Once critical is confirmed, the limit stays in effect until the ESP32 is power-cycled.

Default critical limit:

```text
30%
```

## Disable Drive

The vehicle immediately becomes disarmed and **cannot be rearmed until the ESP32 is power-cycled**, normally after fitting a charged battery. A LiPo's resting voltage rebounds once the load is removed, so allowing recovery by voltage would let the pack be driven, sag and re-armed repeatedly toward over-discharge. The status light flashes red slowly while locked out. The displayed battery state may still move back to LOW or NORMAL; the lockout remains. Saving settings, changing the critical action, disabling monitoring or restoring defaults does not clear it either; only a power cycle does.

---

# Battery Calibration

Use a multimeter to compare the actual pack voltage to the voltage displayed by ESP RC.

Example:

```text
Multimeter:       8.02 V
ESP RC reports: 7.88 V
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

# Bluetooth Pairing and Reconnection

Bluepad32 stores Bluetooth link keys in ESP32 NVS. The v0.4 sketch already avoided clearing keys at startup; v0.5 and later preserve that behavior, explicitly enable connections/scanning at boot and after disconnect, and disable virtual mouse devices so the DualSense touchpad cannot occupy the gamepad slot.

For initial pairing, hold **Create + PS** until the DualSense flashes rapidly. On later power cycles, power the ESP32 and press **PS** to wake the previously paired controller. Bluepad32 is expected to reconnect using saved keys; the controller cannot be woken by the ESP32 while powered off. Reconnection always leaves drive disarmed and requires released buttons, neutral sticks, and a new OPTIONS press.

**Known 0.7.0 hardware-test issues:** an existing pairing may not survive ESP32 / controller power cycles reliably, so **Create + PS** re-pairing can be needed after reboot. Initial pairing is also inconsistent and may require more than one attempt. These remain open issues; saved keys do not guarantee successful reconnection in every power-cycle scenario.

The DualSense LEDs have also been observed to remain blue even though pairing succeeds and the vehicle status pixel behaves normally. In this scenario, controller LED color alone is not a reliable indication of connection / application state. See H6-H8 in the [issue tracker](docs/ISSUES.md#web-ui-and-bluetooth).

To replace a controller:

1. Disarm and open the Wi-Fi configuration page.
2. Choose **Clear controller pairing** and confirm. This disconnects the current controller and clears Bluepad32 keys, without deleting vehicle settings.
3. Turn off the old controller, then hold **Create + PS** on the replacement.

If the old controller is unavailable, send the exact command `PAIR RESET` followed by a newline in Serial Monitor at 115200 baud while disarmed. Pairing reset is blocked while armed.

Serial startup diagnostics print the Bluepad32 version and local Bluetooth address. Persistent pairing still requires that NVS is not erased during upload and that the controller's pairing has not been replaced by pairing to another host. If reconnecting still fails, capture those diagnostics and test a single explicit pairing reset. This change does not claim to repair every controller or board-specific Bluetooth failure.

See the [Bluepad32 API](https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32_arduino/ArduinoBluepad32.h) and [official Arduino setup documentation](https://bluepad32.readthedocs.io/en/latest/plat_arduino/).

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
GPIO 32 - Servo 1
GPIO 33 - Auxiliary output
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
* improved Wi-Fi / Bluetooth coexistence: automatically suspend Bluetooth / disconnect the controller before starting SoftAP, then restore Bluetooth after the existing staged Wi-Fi shutdown; require controller reconnect and neutral validation, and never auto-arm ([F8](docs/ISSUES.md#f8---wi-fi--bluetooth-coexistence-during-configuration))
* physical configuration button
* dedicated emergency-stop GPIO
* custom ESPRC PCB
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
Firmware:                  0.7.0
Drive Mode:                Tank
Steering Mode:             Proportional
Steering Sensitivity:      100%
Maximum Motor Output:      100% (profile cap still applies)

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

Wi-Fi SSID:                ESPRC
Wi-Fi Password:            ESPRC123
Wi-Fi Idle Timeout:        300 sec
Wi-Fi Access:              Anytime while disarmed
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

**Firmware 0.7.0 has been hardware tested and is mostly functional**, based on owner-reported testing. This does not establish that every individual regression case has passed; the [hardware regression checklist](docs/ISSUES.md#070-hardware-test-status-and-regression-checklist) remains available for follow-up validation.

Known issues remain:

* SoftAP association is unreliable with the DualSense connected. Enable Wi-Fi configuration mode, then power off / disconnect the controller before joining the AP. This improves reliability but is cumbersome; automatic Bluetooth suspension / disconnect and restoration is planned (F8).
* Existing pairing may not survive ESP32 / controller power cycles, requiring **Create + PS** re-pairing (H6).
* Initial pairing is inconsistent (H7).
* DualSense LEDs can remain blue despite successful pairing and normal vehicle status-pixel behavior (H8).

See [Bluetooth Pairing and Reconnection](#bluetooth-pairing-and-reconnection) and the [current Wi-Fi workaround](#current-wi-fi--bluetooth-workaround-070) for details.

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

License

ESP RC is licensed under the GNU General Public License version 3 or later (GPL-3.0-or-later).

You are free to use, study, modify, and redistribute this software under the terms of the GNU GPL. If you distribute modified versions or derivative works based on this software, the corresponding source code must also be made available under the GPL.

This software is provided without warranty of any kind. See the LICENSE file for the complete license terms.

Copyright © 2026 RJ Riemensnider

---

# Disclaimer

This project is provided for experimental and hobby use.

Battery systems, motors, motor drivers, wiring, and mechanical systems can produce significant current, heat, and movement. Verify component ratings and incorporate appropriate electrical and mechanical safety protections for your specific vehicle.





## Serial diagnostics

USB Serial (115200 baud) is event-driven: it prints boot information, including the **reset reason** (power-on, crash, watchdog, brownout and so on), and meaningful state changes (controller connect / initializing / ready / disconnect, second controller rejected, stale-data stop, data timeout and resume, arm / disarm, blocked arming with the reason, PS e-stop, speed profile, battery state and lockout, Wi-Fi on, each Wi-Fi shutdown stage, pairing reset, settings saved, emergency abort). The reset reason also appears in the web Live status. There is no once-per-second status line. For live stick and button diagnostics, set `VERBOSE_CONTROLLER_DEBUG` to `true` near the controller globals in the sketch.

## Building and validation

Use the **ESP32 + Bluepad32** board package for the original ESP32, not a BLE-only ESP32 variant. Version 0.5 was compiled and linked against board package `esp32-bluepad32:esp32@4.1.0` (bundled Arduino ESP32 core 2.0.17), using Adafruit NeoPixel 1.15.5; 0.7.0 was compiled against the same toolchain (about 93% of the default app partition). Motor PWM is 20 kHz, 8-bit. The existing Arduino core 3.x PWM compatibility branch is retained but has not been built.

The sketch is kept at its existing repository path. For Arduino IDE/CLI, copy `Code/ESP32-RC-Tank-WebUI.ino` into a folder named `ESP32-RC-Tank-WebUI`, then select **ESP32 Dev Module** under the Bluepad32 board package. CLI example, with that package and NeoPixel installed:

```sh
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32 ESP32-RC-Tank-WebUI
```

There are no automated tests in this repository yet. Validation is done on hardware. With tracks raised, verify both drive modes, steering direction, power-cycle reconnection, disconnect stopping, Wi-Fi entry/exit, and battery warnings before ground operation.
