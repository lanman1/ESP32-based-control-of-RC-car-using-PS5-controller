# ESP RC — Handoff for Claude Code

Context carried over from a claude.ai chat session (September 2026). Read this first, then read `README.md` and `Code/ESP32-RC-Tank-WebUI.ino`.

## Project

ESP32 (WROOM-32/32E) firmware for a tracked RC vehicle: Cytron MDD3A motor driver, PS5 DualSense via Bluepad32, WS281x status pixel, SoftAP web configuration, NVS settings, 2S LiPo monitoring on GPIO 34. Current firmware: 0.5. Built with board package `esp32-bluepad32:esp32@4.1.0` (Arduino core 2.0.17) and Adafruit NeoPixel 1.15.5. The README is the spec.

## Goals for this session

1. Set up change tracking: `CHANGELOG.md`, one GitHub issue per review finding, a branch per fix, and a version bump to 0.6 once the first batch lands.
2. Fix the priority items (1, 2, 6 below) first, then work through the rest.
3. Keep the README accurate as behavior changes.
4. The README references `tests/test_firmware.py`, but it isn't in the repo. Ask the owner whether it exists locally and should be committed.

Work on branches and let the owner review before merging. Don't force-push or rewrite history.

## Code review findings (v0.5)

Line numbers are approximate.

### Bugs and edge cases

1. **OPTIONS pressed before Triangle arms instead of entering config** (`processButtons`, ~3680). Arming fires on the OPTIONS rising edge, and Triangle only suppresses it if already held. Fix: arm on OPTIONS release, and only if Triangle wasn't pressed during that press. **Priority.**
2. **A second controller can occupy a slot and strand the vehicle** (`onConnectedController`, ~3521). Extras are ignored but not disconnected, and new connections stay enabled. If the first drops, the already-connected extra never triggers the connect callback. Fix: disconnect extras and/or disable new connections while one is active. **Priority.**
3. **Denied arming gives no feedback.** Battery lockout falls into the disarm `else` branch, so nothing rumbles. This is most noticeable in the ~3 s battery qualification after boot. Add a rejection rumble.
4. **The data-timeout path leaves stale combo state** (~4018). It doesn't clear `wifiComboActive`, so the status can stay flashing yellow.
5. **Status precedence contradicts the README** (`getStatusColor`, ~2044). Battery warnings hide the Wi-Fi hold and Wi-Fi active indication, which contradicts README config step 5. Put the Wi-Fi states above battery. Wi-Fi yellow and low-battery orange also look nearly identical.

### Design issues

6. **"Disable Drive" can cycle a LiPo toward over-discharge.** Recovery happens at critical + hysteresis (6.95 V). Resting rebound easily exceeds that, so the user can re-arm, sag, and repeat. Latch until power cycle, or require low + hysteresis. **Priority.**
7. **Corrupt-settings recovery can lock out a vehicle with no divider installed** (forced battery on + Disable Drive + reading under 0.5 V gives UNKNOWN, which blocks arming). Add a Serial message and a distinct lockout indication.
8. **Speed-limit reductions bypass the deceleration ramp** (~1445). L1 or the critical limit clamps instantly. Ramp down to the new cap instead, but keep disarm and disconnect instant.
9. **No controller e-stop.** The PRG button is on the vehicle. Add a dedicated stop-only button (e.g., PS or Cross).
10. **The Wi-Fi idle timeout never fires while the page is open**, because `/status` polling counts as activity. Count only user actions.

### README vs code mismatches

- The README claims the legacy settings namespace is migrated, but it's just the only namespace used.
- `tests/` is described but missing.
- Critical recovery wording is ambiguous (crit + hyst vs low + hyst). Make it explicit after fixing item 6.

### Minor

- Public default password `ESPRC123`; consider deriving a per-device password from the MAC.
- PWM at 10 kHz is audible; the MDD3A supports 20 kHz.
- Blocking `delay(20)` in `checkAbortButton`.
- The rumble scheduler's `now < nextPulse` comparison breaks at millis() wrap (~49 days).
- The generic save-validation error message always blames voltage ordering.
- Confirm that the GPIO 0 auto-program circuitry can't trigger a spurious abort when Serial Monitor opens.

### Verify on hardware

Confirm that the 1 s data timeout doesn't falsely disarm with the controller held still. Arm with tracks raised, leave the controller untouched for 10 s, and watch Serial telemetry.
