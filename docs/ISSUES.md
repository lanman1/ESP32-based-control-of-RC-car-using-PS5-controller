# Issue tracker (pending GitHub issues)

Each entry becomes one GitHub issue once the GitHub CLI is available. Keep the ID in the issue title so CHANGELOG and branch names stay traceable. Line numbers refer to `Code/ESP32-RC-Tank-WebUI.ino` at 0.6.0.

Sources: the v0.5 code review (re-checked against 0.6.0) and follow-up web UI / Bluetooth notes. The original handoff documents were removed on 2026-09-23 and remain in git history.

Status: `open`, `in progress (branch)`, `code done in <version>, untested`, `done (version)`.

## Test before merging `release/0.7.0`

With the tracks raised, after flashing 0.7.0 (settings reset to defaults on first boot):

1. **R1 / R3:** OPTIONS then Triangle, and Triangle then OPTIONS, must never arm. Normal arm on release and disarm on press work. Arming with a stick pushed gives 2 pulses.
2. **R9:** PS while armed stops and disarms at once (one long rumble). PS while disarmed does nothing.
3. **S1 / V1:** armed with the controller held still for 10 s: no stale stop or timeout on Serial. Then walk out of range or cover the controller: motors stop within about 0.3 s.
4. **R2:** connect a second paired controller: Serial shows it rejected and the first keeps driving.
5. **H3:** enter and leave Wi-Fi config with the combo, with the web button, and by idle timeout (set 30 s). Each shows the shutdown stages and no crash. Last reset reason stays "power-on".
6. **R5 / U2:** the Wi-Fi hold flashes purple and active is solid purple on both the pixel and the lightbar.
7. **R6 / R7:** with monitoring on, Disable drive, and no divider (or a bench supply below critical): arming gives 3 pulses and a slow red flash. Raising the voltage, saving settings or restoring defaults does not clear a latched critical; only a power cycle does.
8. **R8:** full throttle, then press L1: speed ramps down rather than cutting.
9. **M2:** motor whine gone at 20 kHz, and both motors still reach full speed.
10. **Web UI:** all new cards render on a phone; the idle timeout fires with the page left open (R10).

## Priority

| ID | Title | Status |
|----|-------|--------|
| R1 | OPTIONS pressed before Triangle arms instead of entering config | code done in 0.7.0, untested |
| R2 | Second controller can occupy the slot and strand the vehicle | code done in 0.7.0, untested |
| R6 | Disable Drive lets a LiPo cycle toward over-discharge | code done in 0.7.0, untested |
| H3 | Staged / deferred Wi-Fi shutdown (sys_evt stack-canary crash) | code done in 0.7.0, untested |
| S1 | Stop motors quickly when controller data stops (before the 2 s disarm) | code done in 0.7.0, untested |
| S2 | Log and show the ESP32 reset reason | code done in 0.7.0, untested |

## Bugs and design

| ID | Title | Status |
|----|-------|--------|
| R3 | Denied arming gives no feedback | code done in 0.7.0, untested |
| R4 | Data-timeout path leaves stale Wi-Fi combo state | code done in 0.7.0, untested |
| R5 | Battery colors hide Wi-Fi hold / active indication | code done in 0.7.0, untested |
| R7 | Corrupt-settings recovery can lock out a vehicle with no divider | code done in 0.7.0, untested |
| R8 | Speed-limit reductions bypass the deceleration ramp | code done in 0.7.0, untested |
| R9 | No controller e-stop | code done in 0.7.0, untested |
| R10 | Wi-Fi idle timeout never fires while the page is open | code done in 0.7.0, untested |

## Web UI and Bluetooth

| ID | Title | Status |
|----|-------|--------|
| H1 | Battery divider wiring SVG in web UI | code done in 0.7.0, untested |
| H2 | DualSense Create + PS pairing SVG in web UI | code done in 0.7.0, untested |
| H4 | Bluepad32 init order; virtual device failure non-fatal | code done in 0.7.0, untested |
| H5 | Pairing documentation cleanup (Create + PS, not Options) | done (0.7.0 audit; all text says Create + PS) |
| U1 | DualSense OPTIONS + Triangle Wi-Fi combo diagram in web UI | code done in 0.7.0, untested |
| U2 | Status indicator color legend in web UI | code done in 0.7.0, untested |
| U3 | GPIO map in web UI | code done in 0.7.0, untested |

## Documentation

| ID | Title | Status |
|----|-------|--------|
| D1 | README claims the settings namespace is migrated | done (namespace renamed to `esprc`) |
| D2 | README references missing `tests/` | done (tests section removed) |
| D3 | Critical recovery wording ambiguous | done (0.7.0 README: power cycle clears) |
| T1 | Commit `tests/test_firmware.py` if it exists locally | done (file does not exist) |

## Minor

| ID | Title | Status |
|----|-------|--------|
| M1 | Public default Wi-Fi password `ESPRC123` | deferred (owner keeps it for now) |
| M2 | 10 kHz PWM is audible; MDD3A supports 20 kHz | code done in 0.7.0, untested |
| M3 | Blocking `delay(20)` in `checkAbortButton` | code done in 0.7.0, untested |
| M4 | Rumble scheduler `now < nextPulse` breaks at millis() wrap | code done in 0.7.0, untested |
| M5 | Save-validation error always blames voltage ordering | code done in 0.7.0, untested |
| M6 | Confirm GPIO 0 auto-program circuit cannot trigger a spurious abort | open (hardware) |

## Hardware verification

| ID | Title | Status |
|----|-------|--------|
| V1 | Data timeout (2 s) does not falsely disarm with controller held still | open |
| V2 | GPIO 14 (M2B) boot-time output does not twitch motor 2 | open |

## Future (after 0.7.0)

| ID | Title | Status |
|----|-------|--------|
| F1 | Settings backup / restore (JSON download and upload) | future |
| F2 | Firmware update over Wi-Fi (OTA) | future |
| F3 | D-pad live max-speed adjustment | future |
| F4 | Trigger throttle drive mode (R2 forward, L2 reverse) | future |
| F5 | Lighting engine for pixels 1+ | future |
| F6 | Servo configuration for GPIO 32 | future |
| F7 | Dedicated emergency-stop input | future |

---

## Details

### R1 - OPTIONS pressed before Triangle arms instead of entering config
`processButtons` (~4061) arms on the OPTIONS rising edge; Triangle only suppresses it if already held. Pressing OPTIONS first, then Triangle, arms the vehicle. Fix: act on OPTIONS release, and only if Triangle was not pressed at any point during that OPTIONS press.

### R2 - Second controller can occupy the slot and strand the vehicle
`onConnectedController` (~3885) ignores extra gamepads without disconnecting them, and new connections stay enabled. If the active controller drops, the already-connected extra never fires the connect callback. Fix: disconnect extras and disable new connections while one controller is active; re-enable on disconnect.

### R6 - Disable Drive lets a LiPo cycle toward over-discharge
`determineBatteryState` (~1699) leaves CRITICAL for LOW at critical + hysteresis (6.95 V default). Resting rebound exceeds that, so the user can re-arm, sag, and repeat. Fix: latch critical Disable Drive until power cycle, or require low + hysteresis to recover.

### H3 - Staged / deferred Wi-Fi shutdown
Observed `Guru Meditation Error: Core 0 panic'ed` / `Stack canary watchpoint triggered (sys_evt)` when leaving configuration mode with OPTIONS + Triangle. `stopConfigWiFi` (~3395) runs `server.stop()`, `WiFi.softAPdisconnect(true)` and `WiFi.mode(WIFI_OFF)` back to back; the combo (~3867) and idle timeout (~3521) call it immediately. Fix: one state machine serviced from `loop()` with ~150 ms between stages; all three exits only request shutdown. Log each stage and free heap at request. The fix is a hypothesis until confirmed on hardware.

### S1 - Stop motors quickly when controller data stops
Today nothing happens for `CONTROLLER_TIMEOUT_MS` (2 s) after reports stop, so a vehicle at speed keeps driving for up to 2 s if the Bluetooth link stalls. Add a short motor-stop timeout (about 300 ms, as a constant) that sets motor output to zero without disarming. The existing 2 s timeout still disarms and re-enters INITIALIZING. When reports resume inside 2 s, motors stay at zero until the sticks return to neutral, so the vehicle doesn't lurch. Log the stop once. Check it with V1: a still DualSense keeps sending reports, so holding it still must not trigger S1.

### S2 - Log and show the ESP32 reset reason
At boot, read `esp_reset_reason()` and print it to Serial as text (power-on, software, panic/crash, interrupt or task watchdog, brownout, deep sleep). Show it in the web UI Live status. This helps diagnose H3 crashes and brownouts under motor load.

### R3 - Denied arming gives no feedback
When battery lockout blocks arming, the code falls into the disarm `else` branch and nothing rumbles. Most noticeable during the ~3 s battery qualification after boot. Add a rejection rumble and Serial message.

### R4 - Data-timeout path leaves stale Wi-Fi combo state
The timeout block in `loop()` (~4480) does not clear `wifiComboActive`, so the status can stay flashing yellow until READY.

### R5 - Battery colors hide Wi-Fi indication
`getStatusColor` (~2071) checks battery before Wi-Fi hold / active, contradicting README configuration step 5. Move Wi-Fi states above battery. Wi-Fi yellow (255,150,0) and low-battery orange (255,75,0) are also hard to tell apart.

### R7 - Corrupt-settings recovery can lock out a vehicle with no divider
Corrupt settings force battery monitoring on with Disable Drive. With no divider fitted, the reading is under 0.5 V, the state is UNKNOWN, and arming is blocked. Add a Serial message and a distinct lockout indication.

### R8 - Speed-limit reductions bypass the deceleration ramp
`serviceMotorRamp` (~1492) clamps applied output to the new cap instantly when L1 or the critical limit reduces it. Ramp down to the cap instead; keep disarm and disconnect instant.

### R9 - No controller e-stop
The only e-stop is the PRG button on the vehicle. Add a dedicated stop-only controller button.

### R10 - Wi-Fi idle timeout never fires while the page is open
`handleStatus` calls `noteWebActivity()`, and the page polls `/status` every 2 s. Count only user actions (page load, save, defaults, pairing reset).

### H1 - Battery divider wiring SVG
Inline SVG in the Battery Configuration card: 2S LiPo, battery + to R1 100 kOhm, ADC sense node to GPIO 34, R2 33 kOhm to ground, optional 100 nF to ground, battery - to ESP32 ground (common ground). Notes: ratio ~4.0303:1; 8.4 V gives ~2.08 V at the ADC; never connect the battery directly to GPIO 34. No external resources.

### H2 - DualSense pairing SVG
Inline controller illustration in the controller section showing Create (small button left of the touchpad) and PS (center). Text: "Hold Create + PS until the controller light flashes rapidly." Make clear Options (right of the touchpad) is not the pairing button.

### U1 - Wi-Fi combo diagram
Owner request for 0.7.0. Inline SVG in the Wi-Fi Configuration card showing where OPTIONS (small button right of the touchpad) and Triangle (top face button) are, with the text: hold OPTIONS + Triangle for the configured time (default 3 s) while disarmed with sticks centered; the light flashes yellow during the hold; the same combo turns Wi-Fi off. Share one DualSense outline with H2, highlighting different buttons, so the page stays small and Create vs OPTIONS is visibly distinct. Describe the post-R1 press behavior. Offline only, no external resources.

### H4 - Bluepad32 init order
Order: `BP32.setup(...)`, `BP32.enableVirtualDevice(false)`, `BP32.enableNewBluetoothConnections(true)`. Currently the last two are swapped (~4361). `DS5: Failed to create virtual device` must not block a gamepad that connects.

### H5 - Pairing documentation cleanup
Audit comments, web help, README, and Serial text for pairing instructions; all must say Create + PS. 0.6.0 text is already correct in the places checked; H2 adds the diagram.

### D3 - Critical recovery wording
After R6, state explicitly in README and web help which voltage clears a critical Disable Drive lockout.

### U2 - Status indicator color legend
Reference card listing every status pixel / lightbar color in `getStatusColor()` priority order, with CSS swatches (flash and pulse animated). Must be updated together with R5.

### U3 - GPIO map
Reference card built from the pin `#define`s so it always matches the firmware, plus the PWM frequency and resolution.

### T1 - Host tests
Resolved 2026-09-23: the owner confirmed `tests/test_firmware.py` does not exist. The README tests section was replaced with a note that there are no automated tests yet.

### F1-F7 - Future features
- F1: download current settings as JSON and upload them to restore. Protects settings across future NVS changes. It can't help with the 0.7.0 reset, since 0.6.0 lacks it. Already listed in README "Planned Future Improvements".
- F2: upload firmware from the config web page (Arduino `Update` library). Check first that the partition scheme leaves room for two app images alongside Bluepad32. Also in the README planned list.
- F3: D-pad up/down changes the max output in steps while driving, with a rumble to confirm. Limits still apply. Decide whether the value is saved.
- F4: optional car-style mode: R2 = forward, L2 = reverse, left stick X = steering, alongside the tank and arcade modes. Needs neutral-trigger checks for arming, which INITIALIZING already covers.
- F5: headlights, taillights, brake and reverse lights on pixels 1+, linked to throttle and direction. See README "Planned Lighting Features".
- F6: servo on GPIO 32 with endpoints, centering and reversing in the web UI.
- F7: a wired e-stop input separate from the BOOT/PRG button on GPIO 0.

### M1-M6
- M1: consider a per-device password derived from the MAC. Deferred at owner request.
- M2: raise PWM to 20 kHz (check MDD3A datasheet limit and 8-bit resolution at 20 kHz).
- M3: replace `delay(20)` debounce with a non-blocking timer.
- M4: use `(long)(now - nextPulse) < 0` style comparison.
- M5: report the actual failing rule in `handleSave`.
- M6: confirm the USB auto-program DTR/RTS circuit cannot pull GPIO 0 low when Serial Monitor opens.

### V1 - Data timeout with a still controller
Arm with tracks raised, leave the controller untouched for 10 s, watch Serial for a false data timeout.

### V2 - GPIO 14 at boot
GPIO 14 can emit a signal during ESP32 boot before `setupPWM()` runs. With tracks raised, power-cycle several times and watch motor 2.
