# ESPRC Firmware Handoff

This document summarizes the changes discussed after the current v6 Arduino sketch was generated, but not yet applied to firmware.

## Baseline

Current known generated sketch:

- `ESP32-RC-Tank-WebUI-v6.ino`
- Firmware version string: `0.6.0`

The current v6 already includes:
- WROOM pin mapping
- controller initialization / neutral-validation state
- event-driven serial logging by default
- selectable drive modes
- web UI
- battery monitoring
- Wi-Fi configuration mode
- Bluepad32 controller support
- persistent settings and existing v5 features

The items below are still pending.

---

## 1. Web UI: Battery Wiring Graphic

Add an inline SVG diagram to the battery configuration section showing the approved 2S LiPo measurement circuit.

### Required content

- 2S LiPo battery
- Battery positive to R1
- R1 = 100 kΩ
- ADC sense node
- GPIO34 connected to the ADC sense node
- R2 = 33 kΩ from sense node to ground
- Optional 100 nF capacitor from sense node to ground
- Battery negative tied to ESP32 ground
- Clearly show common ground requirement

### Notes to display

- Divider ratio is approximately 4.0303:1
- 8.4 V battery input produces approximately 2.08 V at the ADC
- Do not connect the battery directly to GPIO34

The diagram must be self-contained and use inline SVG/CSS only; no external image or web dependency.

---

## 2. Web UI: DualSense Pairing Graphic

Add an inline SVG or lightweight controller illustration to the Bluetooth/controller section showing the physical location of:

- Create button
- PS button

### Pairing instruction

Display:

`Hold Create + PS until the controller light flashes rapidly.`

The graphic should make clear that:

- Create is the small button on the left side of the touchpad
- PS is the center PlayStation button

This is important because the Options button was previously mistaken for the Create button.

The diagram must work completely offline from the ESP32 web UI.

---

## 3. Deferred / Staged Wi-Fi Shutdown

A stack-canary crash was observed when disabling Wi-Fi configuration mode with the controller button combination.

Observed panic:

- `Guru Meditation Error: Core 0 panic'ed`
- `Stack canary watchpoint triggered (sys_evt)`

The current Wi-Fi shutdown path appears to perform too much system-event work synchronously.

### Current risky sequence

The current shutdown path effectively performs:

```cpp
server.stop();
WiFi.softAPdisconnect(true);
WiFi.mode(WIFI_OFF);
```

in one immediate call path.

This should be replaced.

### Required new behavior

Controller, web UI, and inactivity timeout should all request Wi-Fi shutdown through the same deferred shutdown state machine.

Do not call the complete shutdown sequence directly from controller button processing.

### Suggested state machine

```cpp
enum WiFiShutdownState {
    WIFI_SHUTDOWN_NONE,
    WIFI_SHUTDOWN_STOP_SERVER,
    WIFI_SHUTDOWN_DISCONNECT_AP,
    WIFI_SHUTDOWN_DISABLE_RADIO,
    WIFI_SHUTDOWN_COMPLETE
};
```

Suggested globals:

```cpp
bool wifiShutdownRequested = false;
WiFiShutdownState wifiShutdownState = WIFI_SHUTDOWN_NONE;
unsigned long wifiShutdownStageTime = 0;
```

### Shutdown flow

1. Request shutdown
2. Stop web server
3. Wait roughly 100–200 ms
4. Disconnect SoftAP
5. Wait roughly 100–200 ms
6. Disable Wi-Fi radio
7. Mark configuration mode inactive
8. Keep motors disarmed
9. Force controller through `CONTROLLER_INITIALIZING`
10. Require settle + neutral validation before READY
11. Never auto-arm after Wi-Fi exits

### Controller button behavior

Instead of:

```cpp
stopConfigWiFi();
```

request shutdown:

```cpp
wifiShutdownRequested = true;
```

The main loop should service the shutdown state machine.

### Inactivity timeout

The inactivity timeout must also use the same deferred shutdown path.

### Web UI shutdown

Any web UI control used to exit configuration mode must also use the same deferred shutdown path.

### Diagnostics

Log only state transitions, for example:

- `Wi-Fi shutdown requested`
- `Wi-Fi shutdown: stopping server`
- `Wi-Fi shutdown: disconnecting AP`
- `Wi-Fi shutdown: disabling radio`
- `Wi-Fi configuration OFF`
- `Controller revalidation required`

Also log free heap at shutdown request, for example:

```cpp
Serial.printf(
    "Wi-Fi shutdown requested. Free heap: %u\n",
    ESP.getFreeHeap()
);
```

Do not restore periodic one-line-per-second serial spam.

---

## 4. Controller Pairing / Virtual Device Cleanup

During DualSense pairing, the following message was observed:

`DS5: Failed to create virtual device`

This appears related to Bluepad32 virtual-device / touchpad handling and is not required for RC operation.

### Required behavior

Explicitly keep virtual devices disabled:

```cpp
BP32.enableVirtualDevice(false);
```

The RC firmware does not need DualSense touchpad-as-mouse behavior.

Ensure initialization order is clean:

```cpp
BP32.setup(...);
BP32.enableVirtualDevice(false);
BP32.enableNewBluetoothConnections(true);
```

Do not treat the virtual-device failure as fatal if the main gamepad connects successfully.

If the message persists even with virtual devices disabled, preserve diagnostic logging but do not block the controller solely because of that message.

---

## 5. Bluetooth Pairing Documentation / UI Help

The correct DualSense pairing combination is:

`Create + PS`

The Create button is on the left side of the touchpad.

The Options button is on the right side and should not be documented as the pairing button.

Update any in-code comments, web UI help text, README text, or pairing instructions that are ambiguous.

---

## 6. Preserve v6 Controller Startup Safety

Do not remove the controller startup/reconnect state machine already added in v6.

Required behavior must remain:

- boot disarmed
- motor outputs zero
- connection does not imply READY
- enter `CONTROLLER_INITIALIZING`
- wait approximately 1 second
- require sticks and triggers neutral
- require neutral stable for a short period
- clear stale button-edge state
- transition to `CONTROLLER_READY`
- remain disarmed until explicit arm action
- disconnect immediately stops motors and disarms
- reconnect requires initialization and manual re-arm
- Wi-Fi exit requires initialization and neutral validation before READY

---

## 7. Preserve Event-Driven Serial Logging

The default USB serial behavior should remain state-change driven.

Do not reintroduce one-line-per-second telemetry.

Default logs should include only meaningful changes such as:

- boot
- controller connected
- controller initializing
- controller ready
- controller disconnected
- data timeout
- reports resumed
- armed
- disarmed
- arming blocked due to non-neutral controls
- Wi-Fi configuration ON/OFF
- Wi-Fi shutdown stages
- drive mode changes
- speed profile changes
- battery state transitions
- Bluetooth pairing reset
- settings saved
- safety stop events

Optional live input debugging may remain behind:

```cpp
constexpr bool VERBOSE_CONTROLLER_DEBUG = false;
```

---

## 8. No Further eFuse / Hardware Work Required in Firmware

Diagnostic testing established:

- Wi-Fi works correctly on generic ESP32 boards
- BLE scanning works correctly
- Bluetooth Classic GAP inquiry works correctly
- generic ESP32 successfully discovers a Bluetooth speaker
- earlier pairing failures were largely caused by using `Options + PS` instead of `Create + PS`

No firmware workaround for eFuses or radio hardware is currently required.

Do not add eFuse manipulation or radio-calibration changes.

---

## 9. Current Hardware / Pin Baseline

Continue to use the WROOM baseline mapping:

```text
M1A / left motor input A     GPIO25
M1B / left motor input B     GPIO26
M2A / right motor input A    GPIO27
M2B / right motor input B    GPIO14
WS281x status LED            GPIO4
Battery ADC                  GPIO34
Servo reserve                GPIO32
Aux reserve                  GPIO33
BOOT / emergency input       GPIO0
```

Battery divider:

```text
R1 = 100 kΩ
R2 = 33 kΩ
Optional capacitor = 100 nF
ADC = GPIO34
Battery = 2S LiPo
```

Wi-Fi configuration:

```text
SSID: ESPRC
Password: ESPRC123
```

---

## Recommended Next Firmware Version

Suggested version:

`0.7.0`

Primary goals:

1. staged/deferred Wi-Fi shutdown to eliminate `sys_evt` stack-canary crashes
2. battery wiring SVG in web UI
3. DualSense Create + PS pairing SVG in web UI
4. pairing documentation cleanup
5. preserve all v6 controller-safety and event-driven logging behavior
