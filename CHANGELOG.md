# Changelog

All notable changes to the ESP RC firmware are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Issue IDs (for example `R1`, `H3`) refer to [docs/ISSUES.md](docs/ISSUES.md) until they are filed as GitHub issues.

## [0.7.0] - 2026-09 (release branch)

Compiled against `esp32-bluepad32:esp32@4.1.0` and Adafruit NeoPixel 1.15.5 (about 93% of the default app partition). **Not yet hardware-tested.** See "Test before merging" in [docs/ISSUES.md](docs/ISSUES.md).

### Upgrade notes
- **Settings reset on update:** the NVS settings namespace is now `esprc`. Settings saved by 0.6.0 or earlier are not migrated, so the first boot starts from factory defaults. Bluetooth pairing is unaffected.
- **Critical battery now latches:** with Limit motor power or Disable drive, a confirmed critical battery stays in effect until the ESP32 is power-cycled (R6).
- **Wi-Fi configuration is now purple** instead of yellow (R5).

### Added
- PS button is a stop-only controller e-stop: stops the motors and disarms at once (R9).
- Motors stop 300 ms after controller reports stop, without disarming; drive resumes only after the controls return to neutral (S1).
- Reset reason on Serial at boot and in the web Live status (S2).
- Web UI: battery divider wiring diagram (H1), Controller card with controls table and Create + PS pairing diagram (H2), OPTIONS + Triangle diagram in the Wi-Fi card (U1), Status Indicator Colors legend (U2), GPIO Map (U3).
- Web Live status shows the active protection (battery lockout, power limit, stale-data stop).
- Web UI controller diagrams redrawn in a filled, shaded style: button map (Controller card), tank and arcade stick diagrams (Drive card), pairing and OPTIONS + Triangle diagrams.

### Fixed
- OPTIONS now arms on release, only if Triangle was not pressed during that press, so the Wi-Fi combo can no longer arm the vehicle. Disarm still happens on press (R1).
- Denied arming rumbles and logs the reason: 3 pulses for battery lockout, Wi-Fi mode or abort, 2 for non-neutral controls (R3).
- A second controller is disconnected, and new connections are disabled while one is active, so an extra controller cannot strand the vehicle (R2).
- Wi-Fi shutdown is staged from `loop()` (stop server, disconnect AP, radio off, about 150 ms apart) for the controller combo, web button and idle timeout, targeting the `sys_evt` stack-canary crash (H3).
- The data-timeout path clears the Wi-Fi combo state (R4).
- Wi-Fi hold / active colors take priority over battery warnings (R5).
- Critical battery can no longer be cleared by resting-voltage rebound, settings changes or restoring defaults; only a power cycle clears it (R6).
- Monitoring enabled with no divider fitted is reported on Serial, in the web status, and with a slow red flash (R7).
- Lowering the speed cap ramps down at the deceleration rate instead of cutting instantly (R8).
- The Wi-Fi idle timeout no longer resets on `/status` polling (R10).
- Bluepad32 init order: virtual devices disabled before connections are enabled (H4).
- Abort-button debounce is non-blocking (M3); rumble timing is safe across `millis()` wrap (M4); save errors name the failing field or rule (M5).

### Changed
- Motor PWM raised from 10 kHz to 20 kHz (M2).

### Documentation
- README updated for all of the above.
- README no longer describes the host regression tests; `tests/test_firmware.py` never existed in this repository (D2, T1).
- Critical recovery wording is explicit: power cycle clears the lockout (D3).

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
