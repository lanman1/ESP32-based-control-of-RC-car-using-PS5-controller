# Changelog

All notable changes to the ESP RC firmware are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Issue IDs (for example `R1`, `H3`) refer to [docs/ISSUES.md](docs/ISSUES.md) until they are filed as GitHub issues.

## [Unreleased] - target 0.7.0

### Changed
- **Settings reset on update:** the NVS settings namespace is now `esprc`. Settings saved by 0.6.0 or earlier are not migrated, so the first boot starts from factory defaults. Bluetooth pairing is unaffected.

### Planned
- Staged / deferred Wi-Fi shutdown for controller, web, and idle-timeout exits (H3).
- Priority safety fixes: OPTIONS-before-Triangle arming (R1), second controller stranding the vehicle (R2), critical-battery re-arm cycling (R6).
- Battery divider wiring diagram (H1), DualSense Create + PS pairing diagram (H2), and DualSense OPTIONS + Triangle Wi-Fi combo diagram (U1) in the web UI.
- Remaining review findings R3-R5, R7-R10 and minor items M1-M6.

### Documentation
- README no longer describes the host regression tests; `tests/test_firmware.py` never existed in this repository (D2, T1).

## [0.6.0] - 2026-09

Baseline imported from the generated `ESP32-RC-Tank-WebUI-v6.ino`. Not compiled or hardware-tested as part of this import.

### Added
- Controller INITIALIZING state: a new connection, resumed reports, or exit from configuration mode require a 1 s settle plus 250 ms of neutral sticks and triggers before READY. The vehicle never auto-arms.
- Cyan pulse on the status pixel and lightbar while the controller initializes.
- Optional `VERBOSE_CONTROLLER_DEBUG` live input stream (off by default).
- Serial logging of Wi-Fi client count changes.

### Changed
- **GPIO mapping (rewire required):** motors M1A/M1B/M2A/M2B on GPIO 25/26/27/14 (was 17/18/22/23); WS281x data on GPIO 4 (was 25); reserved Servo 1 on GPIO 32 and Aux on GPIO 33 (were Servo 1/2 on GPIO 26/27).
- USB Serial is event-driven; the 1 Hz status line is removed.
- Controller-data timeout is 2 s and re-enters INITIALIZING when reports resume.
- The web **Shut Down Wi-Fi** button defers shutdown by 500 ms so the response page can be delivered.

### Documentation
- README updated for the 0.6.0 pin map, controller initialization, data timeout, and status colors.
- README no longer claims the settings NVS namespace is migrated from a legacy one.
- README notes that `tests/test_firmware.py` is not yet in the repository.

### Known issues
- Disabling configuration Wi-Fi with OPTIONS + Triangle has crashed with `Stack canary watchpoint triggered (sys_evt)` (H3).
- Review findings R1-R10 and M1-M6 from the v0.5 review are still present.
- GPIO 14 (M2B) may output a brief signal during ESP32 boot, before firmware configures PWM. Verify on hardware that motor 2 does not twitch at power-up (V2).

## [0.5]

Last release before this changelog. Tank / arcade drive modes, proportional / pivot steering, inline SVG drive diagrams, 100 kΩ / 33 kΩ battery divider constants, anytime Wi-Fi entry while disarmed, pairing reset via web and Serial, and strict save validation. See the git history for details.
