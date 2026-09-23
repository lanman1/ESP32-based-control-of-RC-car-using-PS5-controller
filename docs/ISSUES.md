# Issue tracker (pending GitHub issues)

Each entry becomes one GitHub issue once the GitHub CLI is available. Keep the ID in the issue title so CHANGELOG and branch names stay traceable. Line numbers refer to `Code/ESP32-RC-Tank-WebUI.ino` at 0.6.0.

Sources: [HANDOFF.md](HANDOFF.md) (v0.5 review, re-checked against 0.6.0) and [HANDOFF2.md](HANDOFF2.md).

Status: `open`, `in progress (branch)`, `done (version)`.

## Priority

| ID | Title | Status |
|----|-------|--------|
| R1 | OPTIONS pressed before Triangle arms instead of entering config | open |
| R2 | Second controller can occupy the slot and strand the vehicle | open |
| R6 | Disable Drive lets a LiPo cycle toward over-discharge | open |
| H3 | Staged / deferred Wi-Fi shutdown (sys_evt stack-canary crash) | open |

## Bugs and design

| ID | Title | Status |
|----|-------|--------|
| R3 | Denied arming gives no feedback | open |
| R4 | Data-timeout path leaves stale Wi-Fi combo state | open |
| R5 | Battery colors hide Wi-Fi hold / active indication | open |
| R7 | Corrupt-settings recovery can lock out a vehicle with no divider | open |
| R8 | Speed-limit reductions bypass the deceleration ramp | open |
| R9 | No controller e-stop | open |
| R10 | Wi-Fi idle timeout never fires while the page is open | open |

## Web UI and Bluetooth (HANDOFF2)

| ID | Title | Status |
|----|-------|--------|
| H1 | Battery divider wiring SVG in web UI | open |
| H2 | DualSense Create + PS pairing SVG in web UI | open |
| H4 | Bluepad32 init order; virtual device failure non-fatal | open |
| H5 | Pairing documentation cleanup (Create + PS, not Options) | open |
| U1 | DualSense OPTIONS + Triangle Wi-Fi combo diagram in web UI | open |

## Documentation

| ID | Title | Status |
|----|-------|--------|
| D1 | README claims the settings namespace is migrated | done (namespace renamed to `esprc`) |
| D2 | README references missing `tests/` | partly done (noted as missing); see T1 |
| D3 | Critical recovery wording ambiguous | open (after R6) |
| T1 | Commit `tests/test_firmware.py` if it exists locally | waiting on owner |

## Minor

| ID | Title | Status |
|----|-------|--------|
| M1 | Public default Wi-Fi password `ESPRC123` | deferred (owner keeps it for now) |
| M2 | 10 kHz PWM is audible; MDD3A supports 20 kHz | open |
| M3 | Blocking `delay(20)` in `checkAbortButton` | open |
| M4 | Rumble scheduler `now < nextPulse` breaks at millis() wrap | open |
| M5 | Save-validation error always blames voltage ordering | open |
| M6 | Confirm GPIO 0 auto-program circuit cannot trigger a spurious abort | open (hardware) |

## Hardware verification

| ID | Title | Status |
|----|-------|--------|
| V1 | Data timeout (2 s) does not falsely disarm with controller held still | open |
| V2 | GPIO 14 (M2B) boot-time output does not twitch motor 2 | open |

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

### T1 - Host tests
README describes `tests/test_firmware.py`, but it is not in the repo. Owner to confirm whether it exists locally and should be committed.

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
