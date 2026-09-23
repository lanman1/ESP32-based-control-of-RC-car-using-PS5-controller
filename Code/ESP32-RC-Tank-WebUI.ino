/*
 ======================================================================
 ESP RC TANK CONTROLLER
 Firmware 0.7.0 - Safety, Wi-Fi shutdown and web UI update

 Revision 0.7.0:
   - OPTIONS arms on release; Triangle during the press cancels (R1).
   - Denied arming rumbles and logs the reason (R3).
   - PS button is a stop-only controller e-stop (R9).
   - Motors stop 300 ms after controller data stops (S1).
   - Only one controller at a time; extras are disconnected (R2).
   - Critical battery lockout latches until power cycle (R6).
   - Staged, deferred Wi-Fi shutdown for every exit path (H3).
   - Wi-Fi status is purple and shown above battery warnings (R5).
   - Reset reason shown on Serial and in the web UI (S2).
   - Speed-limit drops follow the deceleration ramp (R8).
   - 20 kHz motor PWM (M2).
   - Web UI: wiring, pairing and combo diagrams; color legend; GPIO map.

 ESP32-WROOM-32 / ESP32-WROOM-32E
 Cytron MDD3A
 PS5 DualSense via Bluepad32
 WS2811 / WS2812 status & lighting
 Wi-Fi SoftAP configuration
 Vehicle battery monitoring

 ----------------------------------------------------------------------
 PIN ASSIGNMENTS
 ----------------------------------------------------------------------

 GPIO 25  -> MDD3A M1A
 GPIO 26  -> MDD3A M1B

 GPIO 27  -> MDD3A M2A
 GPIO 14  -> MDD3A M2B

 GPIO 4   -> WS2811 / WS2812 data
             Pixel 0 reserved for SYSTEM STATUS

 GPIO 32  -> Reserved future Servo 1
 GPIO 33  -> Reserved future Auxiliary output

 GPIO 34  -> Vehicle battery ADC

 GPIO 0   -> Emergency abort button on development board

 ----------------------------------------------------------------------
 PS5 CONTROLS
 ----------------------------------------------------------------------

 LEFT STICK Y    Tank: left track / Arcade: both tracks throttle
 RIGHT STICK Y   Tank: right track
 RIGHT STICK X   Arcade: steering

 OPTIONS         Arm (on release) / Disarm (on press)
 PS              E-stop: disarm immediately (stop only)

 L1              Low-speed profile
 R1              Normal-speed profile

 OPTIONS +
 TRIANGLE
 held 3 seconds  Toggle configuration Wi-Fi

 CREATE + PS     Pairing mode (hold until the light flashes rapidly)

 ----------------------------------------------------------------------
 WI-FI
 ----------------------------------------------------------------------

 SSID:       ESPRC
 Password:   ESPRC123
 Address:    http://192.168.4.1

 ----------------------------------------------------------------------
 STATUS COLORS (highest priority first)
 ----------------------------------------------------------------------

 Flash red        Emergency abort (power-cycle to clear)
 Flash purple     Wi-Fi activation hold in progress
 Purple           Configuration Wi-Fi active
 Slow flash red   Drive locked out by battery protection
 Red              CRITICAL vehicle battery
 Orange           LOW vehicle battery
 Blue pulse       Waiting for controller (status LED only)
 Cyan pulse       Controller initializing / waiting for neutral
 Green            Motors ARMED
 Blue             Connected / SAFE

 ======================================================================
*/

#include <Arduino.h>
#include <Bluepad32.h>

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include <Adafruit_NeoPixel.h>

#include <esp_arduino_version.h>
#include <esp_system.h>

#include <math.h>


// ======================================================================
// VERSION
// ======================================================================

#define FIRMWARE_VERSION "0.7.0"


// ======================================================================
// HARDWARE
// ======================================================================

// MDD3A
#define M1A_PIN 25
#define M1B_PIN 26
#define M2A_PIN 27
#define M2B_PIN 14

// WS281x
#define PIXEL_PIN 4
#define PIXEL_COUNT 8

Adafruit_NeoPixel pixels(
    PIXEL_COUNT,
    PIXEL_PIN,
    NEO_GRB + NEO_KHZ800
);

// Vehicle battery
#define BATTERY_ADC_PIN 34

// Reserved expansion pins
#define SERVO1_PIN 32
#define AUX1_PIN 33

// Divider:
//
// Battery + -> 100K -> ADC -> 33K -> Ground
//
const float BAT_R_TOP = 100000.0;
const float BAT_R_BOTTOM = 33000.0;

const float BAT_DIVIDER_RATIO =
    (BAT_R_TOP + BAT_R_BOTTOM) /
    BAT_R_BOTTOM;

// Emergency button
#define ABORT_BUTTON 0


// ======================================================================
// MOTOR PWM
// ======================================================================

// 20 kHz is above hearing and within the MDD3A limit (M2).
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define PWM_FULL 255

// ESP32 Arduino 2.x channels
#define PWM_CH_M1A 0
#define PWM_CH_M1B 1
#define PWM_CH_M2A 2
#define PWM_CH_M2B 3


// ======================================================================
// WI-FI
// ======================================================================

const char* AP_SSID =
    "ESPRC";

const char* AP_PASSWORD =
    "ESPRC123";

WebServer server(80);

bool wifiConfigActive = false;

unsigned long wifiLastActivity = 0;

// Staged Wi-Fi shutdown (H3). Every exit path only requests shutdown;
// loop() then performs one step at a time with a pause between steps.
enum WiFiShutdownState {
    WIFI_SHUTDOWN_NONE,
    WIFI_SHUTDOWN_PENDING,
    WIFI_SHUTDOWN_DISCONNECT_AP,
    WIFI_SHUTDOWN_DISABLE_RADIO
};

WiFiShutdownState wifiShutdownState = WIFI_SHUTDOWN_NONE;
unsigned long wifiShutdownStageStart = 0;
unsigned long wifiShutdownStageDelay = 0;
const unsigned long WIFI_SHUTDOWN_STAGE_MS = 150;


// ======================================================================
// SETTINGS
// ======================================================================

Preferences prefs;

const char* SETTINGS_NAMESPACE =
    "esprc";


struct TankSettings {

    // Drive: 0 = tank, 1 = arcade; steering: 0 = proportional, 1 = pivot.
    int driveMode;
    int steeringMode;
    int steeringSensitivity;
    int maxOutputPercent;
    int lowSpeedPercent;
    int normalSpeedPercent;

    int stickDeadzone;

    int accelerationMs;
    int decelerationMs;

    int leftTrimPercent;
    int rightTrimPercent;

    bool invertLeft;
    bool invertRight;

    // 0 Linear
    // 1 Soft
    // 2 Medium
    // 3 Aggressive
    int responseCurve;


    // Lighting
    int pixelBrightness;


    // Wi-Fi
    int wifiTimeoutSeconds;


    int wifiHoldSeconds;


    // Battery
    bool batteryEnabled;

    float lowBatteryVoltage;
    float criticalBatteryVoltage;
    float batteryHysteresis;

    float batteryCalibration;

    int batteryConfirmSeconds;

    bool batteryPixelWarning;
    bool batteryControllerLight;
    bool batteryRumble;


    // Critical behavior:
    //
    // 0 = warning only
    // 1 = reduce motor power
    // 2 = disable drive
    //
    int criticalBehavior;

    int criticalPowerLimitPercent;
};


TankSettings settings;


// ======================================================================
// CONTROLLER
// ======================================================================

ControllerPtr controller = nullptr;

enum ControllerState {
    CONTROLLER_DISCONNECTED,
    CONTROLLER_INITIALIZING,
    CONTROLLER_READY
};

ControllerState controllerState =
    CONTROLLER_DISCONNECTED;

unsigned long controllerConnectedAt = 0;
unsigned long controllerNeutralSince = 0;

bool controllerWaitingForNeutralLogged = false;
bool controllerTimeoutActive = false;

// Allow Bluetooth to settle before accepting any control input.
// After the settle period, controls must remain neutral briefly.
const unsigned long CONTROLLER_SETTLE_MS = 1000;
const unsigned long CONTROLLER_NEUTRAL_STABLE_MS = 250;

// Trigger values are normally 0 when released.
const int CONTROLLER_TRIGGER_NEUTRAL_MAX = 32;

// Default Serial behavior is event-driven.  Enable this only when
// live stick / button diagnostics are required.
constexpr bool VERBOSE_CONTROLLER_DEBUG = false;
const unsigned long VERBOSE_CONTROLLER_DEBUG_MS = 250;

bool motorsArmed = false;
bool emergencyAbort = false;


// ======================================================================
// BUTTON EDGE / HOLD STATE
// ======================================================================

bool previousOptions = true;
bool armButtonReleased = false;
bool optionsPressValid = false;      // R1: OPTIONS press eligible to arm on release
bool triangleDuringOptions = false;  // R1: Triangle seen during the current OPTIONS press
unsigned long lastControllerReport = 0;
const unsigned long CONTROLLER_TIMEOUT_MS = 2000;

// S1: stop the motors (without disarming) when reports go quiet for this
// long. After reports resume, drive stays at zero until the sticks and
// triggers return to neutral.
const unsigned long CONTROLLER_STALE_STOP_MS = 300;
bool controllerStaleStop = false;
bool driveNeedsNeutral = false;

bool previousPS = true;   // R9: PS button edge
unsigned long lastVerboseControllerDebug = 0;
bool previousL1 = false;
bool previousR1 = false;

bool wifiComboActive = false;
bool wifiComboTriggered = false;

unsigned long wifiComboStart = 0;


// ======================================================================
// SPEED PROFILE
// ======================================================================

bool lowSpeedProfile = false;


// ======================================================================
// MOTOR STATE
// ======================================================================

int requestedLeftMotor = 0;
int requestedRightMotor = 0;

float appliedLeftMotor = 0;
float appliedRightMotor = 0;

int leftMotorCommand = 0;
int rightMotorCommand = 0;

unsigned long lastRampUpdate = 0;


// ======================================================================
// BATTERY STATE
// ======================================================================

enum BatteryState {
    BATTERY_UNKNOWN,
    BATTERY_NORMAL,
    BATTERY_LOW,
    BATTERY_CRITICAL
};


BatteryState batteryState =
    BATTERY_UNKNOWN;

BatteryState batteryCandidate =
    BATTERY_UNKNOWN;

unsigned long batteryCandidateSince = 0;

float batteryVoltage = 0.0;
float batteryFilteredVoltage = 0.0;

bool batteryFilterInitialized = false;

unsigned long lastBatterySample = 0;

unsigned long lastBatteryReminder = 0;

// R6: once CRITICAL is confirmed, the critical action in force at that
// moment (1 = limit power, 2 = disable drive) stays until power cycle.
// It is kept separately so saving settings or restoring defaults from
// the web page cannot clear it. 0 = not latched.
int batteryLatchedBehavior = 0;

// R7: monitoring is enabled but GPIO 34 reads almost nothing.
bool batteryNoReading = false;


// ======================================================================
// RUMBLE SCHEDULER
// ======================================================================

struct RumblePattern {
    bool active;
    int pulsesRemaining;

    uint16_t durationMs;
    uint16_t gapMs;

    uint8_t weak;
    uint8_t strong;

    unsigned long nextPulse;
};


RumblePattern rumblePattern;


// ======================================================================
// STATUS COLORS
// ======================================================================

struct RGBColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};


uint32_t lastPixelColor =
    0xFFFFFFFF;

uint32_t lastControllerColor =
    0xFFFFFFFF;


// ======================================================================
// TIMERS
// ======================================================================

unsigned long lastStatusUpdate = 0;

// S2: why the ESP32 last restarted.
const char* resetReasonText = "unknown";


// ======================================================================
// FORWARD DECLARATIONS
// ======================================================================

void stopMotorsImmediate();
void updateStatusIndicators();
bool validSettings(const TankSettings& v);
BatteryState determineBatteryState(float voltage);
const char* batteryStateName(BatteryState state);
void batteryStateChanged(BatteryState newState);
RGBColor getStatusColor(bool forController);
void servicePairingSerial();
void serviceControllerState();
bool controllerInputsNeutral();
const char* controllerStateName(ControllerState state);
void printVerboseControllerDebug();


// ======================================================================
// DEFAULT SETTINGS
// ======================================================================

void setFactoryDefaults() {
    settings.driveMode = 0;
    settings.steeringMode = 0;
    settings.steeringSensitivity = 100;
    settings.maxOutputPercent = 100;

    settings.lowSpeedPercent =
        30;

    settings.normalSpeedPercent =
        50;

    settings.stickDeadzone =
        55;

    settings.accelerationMs =
        800;

    settings.decelerationMs =
        500;

    settings.leftTrimPercent =
        100;

    settings.rightTrimPercent =
        100;

    settings.invertLeft =
        false;

    settings.invertRight =
        false;

    settings.responseCurve =
        2;


    settings.pixelBrightness =
        80;


    settings.wifiTimeoutSeconds =
        300;


    settings.wifiHoldSeconds =
        3;


    // Disabled until voltage divider is installed.
    settings.batteryEnabled =
        false;

    settings.lowBatteryVoltage =
        7.20;

    settings.criticalBatteryVoltage =
        6.80;

    settings.batteryHysteresis =
        0.15;

    settings.batteryCalibration =
        1.000;

    settings.batteryConfirmSeconds =
        3;

    settings.batteryPixelWarning =
        true;

    settings.batteryControllerLight =
        true;

    settings.batteryRumble =
        true;

    settings.criticalBehavior =
        0;

    settings.criticalPowerLimitPercent =
        30;
}


// ======================================================================
// LOAD SETTINGS
// ======================================================================

// Returns nullptr when valid, otherwise a description of the first
// failing rule (M5).
const char* settingsProblem(const TankSettings& v) {
    if (!(v.lowSpeedPercent >= 5 && v.lowSpeedPercent <= 100)) return "Low-speed limit must be 5-100%";
    if (!(v.normalSpeedPercent >= 5 && v.normalSpeedPercent <= 100)) return "Normal-speed limit must be 5-100%";
    if (!(v.driveMode >= 0 && v.driveMode <= 1)) return "Invalid drive mode";
    if (!(v.steeringMode >= 0 && v.steeringMode <= 1)) return "Invalid steering behavior";
    if (!(v.steeringSensitivity >= 0 && v.steeringSensitivity <= 200)) return "Steering sensitivity must be 0-200%";
    if (!(v.maxOutputPercent >= 5 && v.maxOutputPercent <= 100)) return "Maximum motor output must be 5-100%";
    if (!(v.stickDeadzone >= 0 && v.stickDeadzone <= 200)) return "Joystick deadband must be 0-200";
    if (!(v.responseCurve >= 0 && v.responseCurve <= 3)) return "Invalid response curve";
    if (!(v.accelerationMs >= 0 && v.accelerationMs <= 5000)) return "Acceleration must be 0-5000 ms";
    if (!(v.decelerationMs >= 0 && v.decelerationMs <= 5000)) return "Deceleration must be 0-5000 ms";
    if (!(v.leftTrimPercent >= 50 && v.leftTrimPercent <= 120)) return "Left trim must be 50-120%";
    if (!(v.rightTrimPercent >= 50 && v.rightTrimPercent <= 120)) return "Right trim must be 50-120%";
    if (!(v.pixelBrightness >= 1 && v.pixelBrightness <= 255)) return "Pixel brightness must be 1-255";
    if (!(isfinite(v.lowBatteryVoltage) && v.lowBatteryVoltage >= 6.0 && v.lowBatteryVoltage <= 8.4)) return "Warning voltage must be 6.0-8.4 V";
    if (!(isfinite(v.criticalBatteryVoltage) && v.criticalBatteryVoltage >= 6.0 && v.criticalBatteryVoltage <= 8.3)) return "Critical voltage must be 6.0-8.3 V";
    if (!(isfinite(v.batteryHysteresis) && v.batteryHysteresis >= 0.01 && v.batteryHysteresis <= 0.5)) return "Recovery hysteresis must be 0.01-0.5 V";
    if (!(isfinite(v.batteryCalibration) && v.batteryCalibration >= 0.5 && v.batteryCalibration <= 1.5)) return "ADC calibration must be 0.5-1.5";
    if (!(v.batteryConfirmSeconds >= 1 && v.batteryConfirmSeconds <= 15)) return "Threshold confirmation must be 1-15 s";
    if (!(v.criticalBehavior >= 0 && v.criticalBehavior <= 2)) return "Invalid critical battery action";
    if (!(v.criticalPowerLimitPercent >= 10 && v.criticalPowerLimitPercent <= 100)) return "Critical power limit must be 10-100%";
    if (!(v.wifiTimeoutSeconds >= 30 && v.wifiTimeoutSeconds <= 3600)) return "Wi-Fi timeout must be 30-3600 s";
    if (!(v.wifiHoldSeconds >= 2 && v.wifiHoldSeconds <= 10)) return "OPTIONS + Triangle hold must be 2-10 s";
    if (!(v.criticalBatteryVoltage < v.lowBatteryVoltage)) return "Critical voltage must be below warning voltage";
    return nullptr;
}

bool validSettings(const TankSettings& v) {
    return settingsProblem(v) == nullptr;
}

void loadSettings() {

    setFactoryDefaults();

    prefs.begin(
        SETTINGS_NAMESPACE,
        true
    );


    settings.lowSpeedPercent =
        prefs.getInt(
            "lowPct",
            settings.lowSpeedPercent
        );

    settings.normalSpeedPercent =
        prefs.getInt(
            "normPct",
            settings.normalSpeedPercent
        );

    settings.stickDeadzone =
        prefs.getInt(
            "deadzone",
            settings.stickDeadzone
        );

    settings.accelerationMs =
        prefs.getInt(
            "accel",
            settings.accelerationMs
        );

    settings.decelerationMs =
        prefs.getInt(
            "decel",
            settings.decelerationMs
        );

    settings.leftTrimPercent =
        prefs.getInt(
            "trimL",
            settings.leftTrimPercent
        );

    settings.rightTrimPercent =
        prefs.getInt(
            "trimR",
            settings.rightTrimPercent
        );

    settings.invertLeft =
        prefs.getBool(
            "invL",
            settings.invertLeft
        );

    settings.invertRight =
        prefs.getBool(
            "invR",
            settings.invertRight
        );

    settings.responseCurve =
        prefs.getInt(
            "curve",
            settings.responseCurve
        );


    settings.pixelBrightness =
        prefs.getInt(
            "pixBright",
            settings.pixelBrightness
        );


    settings.wifiTimeoutSeconds =
        prefs.getInt(
            "wifiTout",
            settings.wifiTimeoutSeconds
        );


    settings.wifiHoldSeconds =
        prefs.getInt(
            "wifiHold",
            settings.wifiHoldSeconds
        );


    settings.batteryEnabled =
        prefs.getBool(
            "batEnable",
            settings.batteryEnabled
        );

    settings.lowBatteryVoltage =
        prefs.getFloat(
            "batLow",
            settings.lowBatteryVoltage
        );

    settings.criticalBatteryVoltage =
        prefs.getFloat(
            "batCrit",
            settings.criticalBatteryVoltage
        );

    settings.batteryHysteresis =
        prefs.getFloat(
            "batHyst",
            settings.batteryHysteresis
        );

    settings.batteryCalibration =
        prefs.getFloat(
            "batCal",
            settings.batteryCalibration
        );

    settings.batteryConfirmSeconds =
        prefs.getInt(
            "batDelay",
            settings.batteryConfirmSeconds
        );

    settings.batteryPixelWarning =
        prefs.getBool(
            "batPixel",
            settings.batteryPixelWarning
        );

    settings.batteryControllerLight =
        prefs.getBool(
            "batLight",
            settings.batteryControllerLight
        );

    settings.batteryRumble =
        prefs.getBool(
            "batRumble",
            settings.batteryRumble
        );

    settings.criticalBehavior =
        prefs.getInt(
            "critAct",
            settings.criticalBehavior
        );

    settings.criticalPowerLimitPercent =
        prefs.getInt(
            "critLimit",
            settings.criticalPowerLimitPercent
        );


    settings.driveMode = prefs.getInt("driveMode", settings.driveMode);
    settings.steeringMode = prefs.getInt("steerMode", settings.steeringMode);
    settings.steeringSensitivity = prefs.getInt("steerSense", settings.steeringSensitivity);
    settings.maxOutputPercent = prefs.getInt("maxOutput", settings.maxOutputPercent);
    prefs.end();

    if (!validSettings(settings)) {
        Serial.printf("Invalid saved settings (%s): restoring safe defaults, battery protection enabled.\n", settingsProblem(settings));
        Serial.println("If no battery divider is fitted, disable voltage monitoring in the web UI.");
        setFactoryDefaults();
        settings.batteryEnabled = true;
        settings.criticalBehavior = 2;
    }

    // Safety validation

    settings.lowSpeedPercent =
        constrain(
            settings.lowSpeedPercent,
            5,
            100
        );

    settings.normalSpeedPercent =
        constrain(
            settings.normalSpeedPercent,
            5,
            100
        );

    settings.stickDeadzone =
        constrain(
            settings.stickDeadzone,
            0,
            200
        );

    settings.accelerationMs =
        constrain(
            settings.accelerationMs,
            0,
            5000
        );

    settings.decelerationMs =
        constrain(
            settings.decelerationMs,
            0,
            5000
        );

    settings.leftTrimPercent =
        constrain(
            settings.leftTrimPercent,
            50,
            120
        );

    settings.rightTrimPercent =
        constrain(
            settings.rightTrimPercent,
            50,
            120
        );

    settings.responseCurve =
        constrain(
            settings.responseCurve,
            0,
            3
        );

    settings.pixelBrightness =
        constrain(
            settings.pixelBrightness,
            1,
            255
        );

    settings.wifiTimeoutSeconds =
        constrain(
            settings.wifiTimeoutSeconds,
            30,
            3600
        );


    settings.wifiHoldSeconds =
        constrain(
            settings.wifiHoldSeconds,
            2,
            10
        );

    settings.batteryConfirmSeconds =
        constrain(
            settings.batteryConfirmSeconds,
            1,
            15
        );

    settings.criticalBehavior =
        constrain(
            settings.criticalBehavior,
            0,
            2
        );

    settings.criticalPowerLimitPercent =
        constrain(
            settings.criticalPowerLimitPercent,
            10,
            100
        );
}


// ======================================================================
// SAVE SETTINGS
// ======================================================================

void saveSettings() {

    prefs.begin(
        SETTINGS_NAMESPACE,
        false
    );


    prefs.putInt("driveMode", settings.driveMode);
    prefs.putInt("steerMode", settings.steeringMode);
    prefs.putInt("steerSense", settings.steeringSensitivity);
    prefs.putInt("maxOutput", settings.maxOutputPercent);
    prefs.putInt(
        "lowPct",
        settings.lowSpeedPercent
    );

    prefs.putInt(
        "normPct",
        settings.normalSpeedPercent
    );

    prefs.putInt(
        "deadzone",
        settings.stickDeadzone
    );

    prefs.putInt(
        "accel",
        settings.accelerationMs
    );

    prefs.putInt(
        "decel",
        settings.decelerationMs
    );

    prefs.putInt(
        "trimL",
        settings.leftTrimPercent
    );

    prefs.putInt(
        "trimR",
        settings.rightTrimPercent
    );

    prefs.putBool(
        "invL",
        settings.invertLeft
    );

    prefs.putBool(
        "invR",
        settings.invertRight
    );

    prefs.putInt(
        "curve",
        settings.responseCurve
    );


    prefs.putInt(
        "pixBright",
        settings.pixelBrightness
    );


    prefs.putInt(
        "wifiTout",
        settings.wifiTimeoutSeconds
    );


    prefs.putInt(
        "wifiHold",
        settings.wifiHoldSeconds
    );


    prefs.putBool(
        "batEnable",
        settings.batteryEnabled
    );

    prefs.putFloat(
        "batLow",
        settings.lowBatteryVoltage
    );

    prefs.putFloat(
        "batCrit",
        settings.criticalBatteryVoltage
    );

    prefs.putFloat(
        "batHyst",
        settings.batteryHysteresis
    );

    prefs.putFloat(
        "batCal",
        settings.batteryCalibration
    );

    prefs.putInt(
        "batDelay",
        settings.batteryConfirmSeconds
    );

    prefs.putBool(
        "batPixel",
        settings.batteryPixelWarning
    );

    prefs.putBool(
        "batLight",
        settings.batteryControllerLight
    );

    prefs.putBool(
        "batRumble",
        settings.batteryRumble
    );

    prefs.putInt(
        "critAct",
        settings.criticalBehavior
    );

    prefs.putInt(
        "critLimit",
        settings.criticalPowerLimitPercent
    );


    prefs.end();


    pixels.setBrightness(
        settings.pixelBrightness
    );

    pixels.show();


    Serial.println(
        "Settings saved."
    );
}


// ======================================================================
// PWM INITIALIZATION
// ======================================================================

void setupPWM() {

#if ESP_ARDUINO_VERSION_MAJOR >= 3

    ledcAttach(
        M1A_PIN,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        M1B_PIN,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        M2A_PIN,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        M2B_PIN,
        PWM_FREQ,
        PWM_RESOLUTION
    );

#else

    ledcSetup(
        PWM_CH_M1A,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        PWM_CH_M1B,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        PWM_CH_M2A,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        PWM_CH_M2B,
        PWM_FREQ,
        PWM_RESOLUTION
    );


    ledcAttachPin(
        M1A_PIN,
        PWM_CH_M1A
    );

    ledcAttachPin(
        M1B_PIN,
        PWM_CH_M1B
    );

    ledcAttachPin(
        M2A_PIN,
        PWM_CH_M2A
    );

    ledcAttachPin(
        M2B_PIN,
        PWM_CH_M2B
    );

#endif
}


// ======================================================================
// PWM WRITE
// ======================================================================

void writePWM(
    uint8_t pin,
    uint8_t channel,
    int value
) {

    value =
        constrain(
            value,
            0,
            255
        );


#if ESP_ARDUINO_VERSION_MAJOR >= 3

    ledcWrite(
        pin,
        value
    );

#else

    ledcWrite(
        channel,
        value
    );

#endif
}


// ======================================================================
// MOTOR OUTPUT
// ======================================================================

void setMotor1(int speed) {

    speed =
        constrain(
            speed,
            -255,
            255
        );


    if (speed > 0) {

        writePWM(
            M1A_PIN,
            PWM_CH_M1A,
            speed
        );

        writePWM(
            M1B_PIN,
            PWM_CH_M1B,
            0
        );

    } else if (speed < 0) {

        writePWM(
            M1A_PIN,
            PWM_CH_M1A,
            0
        );

        writePWM(
            M1B_PIN,
            PWM_CH_M1B,
            -speed
        );

    } else {

        writePWM(
            M1A_PIN,
            PWM_CH_M1A,
            0
        );

        writePWM(
            M1B_PIN,
            PWM_CH_M1B,
            0
        );
    }
}


void setMotor2(int speed) {

    speed =
        constrain(
            speed,
            -255,
            255
        );


    if (speed > 0) {

        writePWM(
            M2A_PIN,
            PWM_CH_M2A,
            speed
        );

        writePWM(
            M2B_PIN,
            PWM_CH_M2B,
            0
        );

    } else if (speed < 0) {

        writePWM(
            M2A_PIN,
            PWM_CH_M2A,
            0
        );

        writePWM(
            M2B_PIN,
            PWM_CH_M2B,
            -speed
        );

    } else {

        writePWM(
            M2A_PIN,
            PWM_CH_M2A,
            0
        );

        writePWM(
            M2B_PIN,
            PWM_CH_M2B,
            0
        );
    }
}


// ======================================================================
// IMMEDIATE SAFETY STOP
// ======================================================================

void stopMotorsImmediate() {

    requestedLeftMotor = 0;
    requestedRightMotor = 0;

    appliedLeftMotor = 0;
    appliedRightMotor = 0;

    leftMotorCommand = 0;
    rightMotorCommand = 0;

    setMotor1(0);
    setMotor2(0);
}


// ======================================================================
// BATTERY PROTECTION (R6 / R7)
// ======================================================================

// True when Disable Drive currently forbids driving.
bool batteryDriveLockout() {
    if (batteryLatchedBehavior == 2) return true;
    return
        settings.batteryEnabled &&
        settings.criticalBehavior == 2 &&
        (
            batteryState == BATTERY_CRITICAL ||
            batteryState == BATTERY_UNKNOWN
        );
}

// Human-readable reason for batteryDriveLockout().
const char* batteryLockoutReason() {
    if (batteryLatchedBehavior == 2) return "critical battery (latched until power cycle)";
    if (batteryState == BATTERY_CRITICAL) return "critical battery";
    if (batteryNoReading) return "no battery voltage on GPIO 34 (check divider or disable monitoring)";
    return "battery reading not qualified yet";
}

// True when Limit Motor Power currently applies.
bool batteryPowerLimited() {
    if (batteryLatchedBehavior == 1) return true;
    return
        settings.batteryEnabled &&
        settings.criticalBehavior == 1 &&
        batteryState == BATTERY_CRITICAL;
}


// ======================================================================
// CURRENT SPEED LIMIT
// ======================================================================

int selectedSpeedPercent() {

    return lowSpeedProfile
        ? settings.lowSpeedPercent
        : settings.normalSpeedPercent;
}


int effectiveSpeedPercent() {
    int percent = min(selectedSpeedPercent(), settings.maxOutputPercent);
    if (batteryPowerLimited()) {
        percent = min(percent, settings.criticalPowerLimitPercent);
    }
    return percent;
}


// ======================================================================
// RESPONSE CURVE
// ======================================================================

float applyResponseCurve(
    float value
) {

    value =
        constrain(
            value,
            0.0f,
            1.0f
        );


    switch (
        settings.responseCurve
    ) {

        // Linear
        case 0:
            return value;

        // Soft
        case 1:
            return powf(
                value,
                1.8f
            );

        // Medium
        case 2:
            return powf(
                value,
                1.4f
            );

        // Aggressive
        case 3:
            return powf(
                value,
                0.70f
            );
    }


    return value;
}


// ======================================================================
// STICK -> MOTOR COMMAND
// ======================================================================

float normalizedStick(int rawValue) {
    rawValue = constrain(rawValue, -512, 512);
    int magnitude = abs(rawValue);
    if (magnitude <= settings.stickDeadzone) return 0.0f;
    float value = float(magnitude - settings.stickDeadzone) / (512 - settings.stickDeadzone);
    value = applyResponseCurve(value);
    return rawValue < 0 ? -value : value;
}


// ======================================================================
// MOTOR RAMPING
// ======================================================================

float rampToward(
    float current,
    float target,
    unsigned long dt
) {

    // Never instantly reverse polarity.
    // Ramp to zero first.

    if (
        current > 0 &&
        target < 0
    ) {
        target = 0;
    }

    if (
        current < 0 &&
        target > 0
    ) {
        target = 0;
    }


    bool accelerating =
        abs(target) >
        abs(current);


    int rampTime =
        accelerating
            ? settings.accelerationMs
            : settings.decelerationMs;


    if (rampTime <= 0) {
        return target;
    }


    float step =
        255.0f *
        (
            float(dt) /
            float(rampTime)
        );


    if (step < 0.25f) {
        step = 0.25f;
    }


    if (current < target) {

        current += step;

        if (current > target) {
            current = target;
        }

    } else if (current > target) {

        current -= step;

        if (current < target) {
            current = target;
        }
    }


    return current;
}


// ======================================================================
// APPLY MOTOR RAMP
// ======================================================================

void serviceMotorRamp() {

    unsigned long now = millis();
    unsigned long dt = now - lastRampUpdate;

    if (dt < 10) {
        return;
    }

    lastRampUpdate = now;

    // Avoid giant step after debugging pause.
    if (dt > 50) {
        dt = 50;
    }

    bool driveAllowed =
        controller &&
        controller->isConnected() &&
        controllerState == CONTROLLER_READY &&
        motorsArmed &&
        !emergencyAbort &&
        !wifiConfigActive;

    if (batteryDriveLockout()) {
        driveAllowed = false;
        if (motorsArmed) {
            Serial.printf("Drive state: DISARMED - %s.\n", batteryLockoutReason());
        }
        motorsArmed = false;
    }

    if (!driveAllowed) {
        stopMotorsImmediate();
        return;
    }

    // requested*Motor is already limited to the current speed cap by
    // processDrive(). When the cap drops (L1, critical power limit), the
    // applied output ramps down at the deceleration rate instead of being
    // clamped instantly (R8). Disarm, disconnect and e-stop remain instant
    // because they call stopMotorsImmediate().
    appliedLeftMotor = rampToward( appliedLeftMotor, requestedLeftMotor, dt );
    appliedRightMotor = rampToward( appliedRightMotor, requestedRightMotor, dt );

    appliedLeftMotor = constrain(appliedLeftMotor, -255.0f, 255.0f);
    appliedRightMotor = constrain(appliedRightMotor, -255.0f, 255.0f);

    leftMotorCommand = round( appliedLeftMotor );
    rightMotorCommand = round( appliedRightMotor );

    setMotor1( leftMotorCommand );
    setMotor2( rightMotorCommand );
}


// ======================================================================
// DRIVE PROCESSING
// ======================================================================

// Mix normalized inputs; positive steering means clockwise/right yaw,
// including while reversing. Proportional mode never pivots at zero throttle.
void mixDrive(float throttle, float second, float& left, float& right) {
    left = throttle;
    right = second;
    if (settings.driveMode == 1) {
        float steering = constrain(second * settings.steeringSensitivity / 100.0f, -1.0f, 1.0f);
        if (settings.steeringMode == 0) steering *= fabsf(throttle);
        left = throttle + steering;
        right = throttle - steering;
    }
    left *= settings.leftTrimPercent / 100.0f;
    right *= settings.rightTrimPercent / 100.0f;
    float scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
    left /= scale;
    right /= scale;
    if (settings.invertLeft) left = -left;
    if (settings.invertRight) right = -right;
}

void processDrive() {
    if (!controller || !controller->isConnected() || controllerState != CONTROLLER_READY ||
        !motorsArmed || emergencyAbort || wifiConfigActive) {
        requestedLeftMotor = requestedRightMotor = 0;
        return;
    }

    // S1: after a stale-data stop, hold zero output until the driver
    // returns every control to neutral, so the vehicle cannot lurch.
    if (driveNeedsNeutral) {
        requestedLeftMotor = requestedRightMotor = 0;
        if (controllerInputsNeutral()) {
            driveNeedsNeutral = false;
            Serial.println("Controls neutral - drive resumed.");
        }
        return;
    }

    float throttle = normalizedStick(-controller->axisY());
    float second = normalizedStick(settings.driveMode == 1 ? controller->axisRX() : -controller->axisRY());

    float left, right;
    mixDrive(throttle, second, left, right);
    int limit = round(effectiveSpeedPercent() * 2.55f);
    requestedLeftMotor = constrain(int(round(left * limit)), -limit, limit);
    requestedRightMotor = constrain(int(round(right * limit)), -limit, limit);
}


// ======================================================================
// RUMBLE
// ======================================================================

void queueRumble(
    int pulses,
    uint16_t durationMs,
    uint16_t gapMs,
    uint8_t weak,
    uint8_t strong
) {

    if (
        !controller ||
        !controller->isConnected()
    ) {
        return;
    }


    rumblePattern.active =
        true;

    rumblePattern.pulsesRemaining =
        pulses;

    rumblePattern.durationMs =
        durationMs;

    rumblePattern.gapMs =
        gapMs;

    rumblePattern.weak =
        weak;

    rumblePattern.strong =
        strong;

    rumblePattern.nextPulse =
        millis();
}


void serviceRumble() {

    if (
        !rumblePattern.active ||
        !controller ||
        !controller->isConnected()
    ) {
        return;
    }


    unsigned long now =
        millis();


    // Wrap-safe comparison (M4).
    if (
        (long)(now - rumblePattern.nextPulse) < 0
    ) {
        return;
    }


    controller->playDualRumble(
        0,
        rumblePattern.durationMs,
        rumblePattern.weak,
        rumblePattern.strong
    );


    rumblePattern.pulsesRemaining--;


    if (
        rumblePattern.pulsesRemaining <=
        0
    ) {

        rumblePattern.active =
            false;

    } else {

        rumblePattern.nextPulse =
            now +
            rumblePattern.durationMs +
            rumblePattern.gapMs;
    }
}


// ======================================================================
// BATTERY STATE NAME
// ======================================================================

const char* batteryStateName(
    BatteryState state
) {

    switch (state) {

        case BATTERY_NORMAL:
            return "NORMAL";

        case BATTERY_LOW:
            return "LOW";

        case BATTERY_CRITICAL:
            return "CRITICAL";

        default:
            return "UNKNOWN";
    }
}


// ======================================================================
// DETERMINE BATTERY STATE WITH HYSTERESIS
// ======================================================================

BatteryState determineBatteryState(
    float voltage
) {

    if (
        batteryState ==
        BATTERY_CRITICAL
    ) {

        if (
            voltage >=
            settings.lowBatteryVoltage +
            settings.batteryHysteresis
        ) {
            return BATTERY_NORMAL;
        }

        if (
            voltage >=
            settings.criticalBatteryVoltage +
            settings.batteryHysteresis
        ) {
            return BATTERY_LOW;
        }

        return BATTERY_CRITICAL;
    }


    if (
        batteryState ==
        BATTERY_LOW
    ) {

        if (
            voltage <=
            settings.criticalBatteryVoltage
        ) {
            return BATTERY_CRITICAL;
        }

        if (
            voltage >=
            settings.lowBatteryVoltage +
            settings.batteryHysteresis
        ) {
            return BATTERY_NORMAL;
        }

        return BATTERY_LOW;
    }


    if (
        voltage <=
        settings.criticalBatteryVoltage
    ) {
        return BATTERY_CRITICAL;
    }


    if (
        voltage <=
        settings.lowBatteryVoltage
    ) {
        return BATTERY_LOW;
    }


    return BATTERY_NORMAL;
}


// ======================================================================
// BATTERY STATE CHANGE
// ======================================================================

void batteryStateChanged( BatteryState newState ) {

    batteryState = newState;

    Serial.print( "Battery state: " );
    Serial.println( batteryStateName( newState ) );

    if ( newState == BATTERY_LOW ) {
        if ( settings.batteryRumble ) {
            queueRumble( 2, 180, 150, 90, 60 );
        }
        lastBatteryReminder = millis();
    }

    if ( newState == BATTERY_CRITICAL ) {

        // R6: resting voltage rebounds after the load is removed, so a
        // critical lockout must not clear by itself. It stays until the
        // ESP32 is power-cycled (normally with a charged battery).
        if ( settings.criticalBehavior > batteryLatchedBehavior ) {
            batteryLatchedBehavior = settings.criticalBehavior;
            Serial.println(
                settings.criticalBehavior == 2
                    ? "Critical battery: drive DISABLED until power cycle."
                    : "Critical battery: motor power LIMITED until power cycle."
            );
        }

        if ( settings.criticalBehavior == 2 ) {
            bool wasArmed = motorsArmed;
            motorsArmed = false;
            stopMotorsImmediate();
            if (wasArmed) {
                Serial.println( "Drive state: DISARMED by critical battery protection." );
            }
        }

        if ( settings.batteryRumble ) {
            queueRumble( 3, 230, 120, 180, 150 );
        }
        lastBatteryReminder = millis();
    }
}


// ======================================================================
// BATTERY MONITOR
// ======================================================================

void serviceBatteryMonitor() {

    if ( !settings.batteryEnabled ) {

        batteryNoReading = false;
        batteryState = BATTERY_UNKNOWN;

        batteryFilterInitialized = false;
        batteryCandidate = BATTERY_UNKNOWN;
        batteryCandidateSince = millis();
        batteryVoltage = batteryFilteredVoltage = 0;
        return;
    }


    unsigned long now =
        millis();


    if (
        now -
        lastBatterySample <
        100
    ) {
        return;
    }


    lastBatterySample =
        now;


    uint32_t totalMillivolts =
        0;


    const int samples =
        12;


    for (
        int i = 0;
        i < samples;
        i++
    ) {

        totalMillivolts +=
            analogReadMilliVolts(
                BATTERY_ADC_PIN
            );
    }


    float adcVolts =
        (
            totalMillivolts /
            float(samples)
        ) /
        1000.0f;


    float measured =
        adcVolts *
        BAT_DIVIDER_RATIO *
        settings.batteryCalibration;


    // Treat very low voltage as "not connected" (R7: say so once).
    // Hysteresis: under 0.5 V sets "no reading"; it clears above 0.8 V.
    if (measured < 0.50f || (batteryNoReading && measured < 0.80f)) {

        if (!batteryNoReading) {
            batteryNoReading = true;
            Serial.println("Battery monitoring: no voltage on GPIO 34 (< 0.5 V). Check the divider wiring, or disable monitoring if no divider is fitted.");
            if (settings.criticalBehavior == 2) {
                Serial.println("Arming is blocked while Disable Drive has no battery reading.");
            }
        }

        batteryState =
            BATTERY_UNKNOWN;

        batteryFilterInitialized = false;
        batteryCandidate = BATTERY_UNKNOWN;
        batteryCandidateSince = millis();
        batteryVoltage = batteryFilteredVoltage = 0;
        return;
    }


    if (batteryNoReading) {
        batteryNoReading = false;
        Serial.println("Battery monitoring: voltage reading restored.");
    }

    batteryVoltage = measured;


    // Exponential filtering
    if (
        !batteryFilterInitialized
    ) {

        batteryFilteredVoltage =
            measured;

        batteryFilterInitialized =
            true;

    } else {

        const float alpha =
            0.15f;

        batteryFilteredVoltage =
            batteryFilteredVoltage +
            alpha *
            (
                measured -
                batteryFilteredVoltage
            );
    }


    BatteryState desired =
        determineBatteryState(
            batteryFilteredVoltage
        );


    if (
        desired ==
        batteryState
    ) {

        batteryCandidate =
            desired;

        batteryCandidateSince =
            now;

    } else {

        if (
            desired !=
            batteryCandidate
        ) {

            batteryCandidate =
                desired;

            batteryCandidateSince =
                now;
        }


        unsigned long confirmMs =
            settings.batteryConfirmSeconds *
            1000UL;


        if (
            now -
            batteryCandidateSince >=
            confirmMs
        ) {

            batteryStateChanged(
                desired
            );
        }
    }


    // Periodic reminder rumble

    if (
        settings.batteryRumble &&
        controller &&
        controller->isConnected()
    ) {

        if (
            batteryState ==
            BATTERY_LOW &&
            now -
            lastBatteryReminder >
            60000UL
        ) {

            queueRumble(
                2,
                180,
                150,
                90,
                60
            );

            lastBatteryReminder =
                now;
        }


        if (
            batteryState ==
            BATTERY_CRITICAL &&
            now -
            lastBatteryReminder >
            15000UL
        ) {

            queueRumble(
                3,
                230,
                120,
                180,
                150
            );

            lastBatteryReminder =
                now;
        }
    }
}


// ======================================================================
// STATUS COLOR
// ======================================================================

// Priority order must match makeStatusLegendHtml() (U2).
RGBColor getStatusColor( bool allowBatteryWarning ) {

    RGBColor color = {0, 0, 0};
    unsigned long now = millis();

    // Emergency abort: fast red flash.
    if (emergencyAbort) {
        bool on = ( now / 200 ) % 2;
        color.r = on ? 255 : 0;
        return color;
    }

    // Wi-Fi states come before battery warnings (R5) so the driver can
    // always see that configuration mode is holding or active.
    // Purple is used so it cannot be confused with low-battery orange.
    if ( wifiComboActive && !wifiComboTriggered ) {
        bool on = ( now / 180 ) % 2;
        color.r = on ? 150 : 20;
        color.b = on ? 255 : 40;
        return color;
    }

    if ( wifiConfigActive ) {
        color.r = 150;
        color.b = 255;
        return color;
    }

    // Battery protection is stopping the vehicle: slow red flash.
    // Always shown, because it explains why the vehicle will not arm.
    if ( batteryDriveLockout() ) {
        bool on = ( now / 500 ) % 2;
        color.r = on ? 255 : 0;
        return color;
    }

    // Battery warnings
    if ( allowBatteryWarning && settings.batteryEnabled ) {
        if ( batteryState == BATTERY_CRITICAL ) {
            color.r = 255;
            return color;
        }
        if ( batteryState == BATTERY_LOW ) {
            color.r = 255;
            color.g = 75;
            return color;
        }
    }

    // Waiting for controller
    if ( !controller || !controller->isConnected() ) {
        int phase = ( now / 15 ) % 200;
        if (phase > 100) phase = 200 - phase;
        color.b = map( phase, 0, 100, 5, 150 );
        return color;
    }

    // Controller connected but not yet safe to accept commands.
    if ( controllerState == CONTROLLER_INITIALIZING ) {
        int phase = ( now / 12 ) % 200;
        if (phase > 100) phase = 200 - phase;
        color.g = map( phase, 0, 100, 15, 150 );
        color.b = map( phase, 0, 100, 20, 220 );
        return color;
    }

    // Armed
    if ( motorsArmed ) {
        color.g = 200;
        return color;
    }

    // Safe
    color.b = 180;
    return color;
}


// ======================================================================
// STATUS INDICATORS
// ======================================================================

void updateStatusIndicators() {

    if (
        millis() -
        lastStatusUpdate <
        50
    ) {
        return;
    }


    lastStatusUpdate =
        millis();


    // Pixel status

    RGBColor pixelColor =
        getStatusColor(
            settings.batteryPixelWarning
        );


    uint32_t packedPixel =
        (
            uint32_t(pixelColor.r) <<
            16
        ) |
        (
            uint32_t(pixelColor.g) <<
            8
        ) |
        pixelColor.b;


    if (
        packedPixel !=
        lastPixelColor
    ) {

        pixels.setPixelColor(
            0,
            pixels.Color(
                pixelColor.r,
                pixelColor.g,
                pixelColor.b
            )
        );

        pixels.show();

        lastPixelColor =
            packedPixel;
    }


    // DualSense lightbar

    if (
        controller &&
        controller->isConnected()
    ) {

        RGBColor controllerColor =
            getStatusColor(
                settings.batteryControllerLight
            );


        uint32_t packedController =
            (
                uint32_t(
                    controllerColor.r
                ) <<
                16
            ) |
            (
                uint32_t(
                    controllerColor.g
                ) <<
                8
            ) |
            controllerColor.b;


        if (
            packedController !=
            lastControllerColor
        ) {

            controller->setColorLED(
                controllerColor.r,
                controllerColor.g,
                controllerColor.b
            );


            lastControllerColor =
                packedController;
        }
    }
}


// ======================================================================
// EMERGENCY STOP
// ======================================================================

void emergencyStop() {

    emergencyAbort =
        true;

    motorsArmed =
        false;

    stopMotorsImmediate();


    queueRumble(
        4,
        180,
        100,
        255,
        255
    );


    Serial.println();
    Serial.println(
        "*** EMERGENCY ABORT ***"
    );
}


// ======================================================================
// ABORT BUTTON
// ======================================================================

void checkAbortButton() {

    // Non-blocking 20 ms debounce (M3).
    static unsigned long lowSince = 0;

    if (emergencyAbort) {
        return;
    }

    if ( digitalRead( ABORT_BUTTON ) == LOW ) {
        if (lowSince == 0) {
            lowSince = millis() | 1;   // never 0 while held
        } else if ( millis() - lowSince >= 20 ) {
            emergencyStop();
        }
    } else {
        lowSince = 0;
    }
}


// ======================================================================
// CONTROLLER BATTERY PERCENT
// ======================================================================

int controllerBatteryPercent() {

    if (
        !controller ||
        !controller->isConnected()
    ) {
        return -1;
    }


    int value =
        controller->battery();


    // Bluepad32:
    // 0 = unknown
    // 1 = empty
    // 255 = full

    if (value == 0) {
        return -1;
    }


    return constrain(
        map(
            value,
            1,
            255,
            0,
            100
        ),
        0,
        100
    );
}


// ======================================================================
// UPTIME STRING
// ======================================================================

String uptimeString() {

    unsigned long seconds =
        millis() /
        1000UL;


    unsigned long hours =
        seconds /
        3600UL;

    unsigned long minutes =
        (
            seconds %
            3600UL
        ) /
        60UL;

    seconds =
        seconds %
        60UL;


    char buffer[24];


    snprintf(
        buffer,
        sizeof(buffer),
        "%02lu:%02lu:%02lu",
        hours,
        minutes,
        seconds
    );


    return String(
        buffer
    );
}


// ======================================================================
// WEB ACTIVITY
// ======================================================================

void noteWebActivity() {

    wifiLastActivity =
        millis();
}


// ======================================================================
// WEB PAGE
// ======================================================================

// Reference card: status colors (U2).
// Keep this list in the same order and colors as getStatusColor().
String makeStatusLegendHtml() {
    String h;
    h.reserve(2600);
    h += R"HTML(<section class="card"><h2>Status Indicator Colors</h2>
<p>Shown on the status pixel (LED 0) and the DualSense lightbar. When more than one applies, the one higher in this list wins.</p>
<table><tr><th>Color</th><th>Meaning</th></tr>
<tr><td><span class="sw blink" style="background:#ff0000"></span>Fast flashing red</td><td>Emergency abort (PRG button). Power-cycle to clear.</td></tr>
<tr><td><span class="sw blink" style="background:#9600ff"></span>Flashing purple</td><td>OPTIONS + Triangle being held</td></tr>
<tr><td><span class="sw" style="background:#9600ff"></span>Purple</td><td>Wi-Fi configuration active (motors disabled)</td></tr>
<tr><td><span class="sw slow" style="background:#ff0000"></span>Slow flashing red</td><td>Drive locked out by battery protection: critical battery (until power cycle), no voltage reading, or reading not qualified yet</td></tr>
<tr><td><span class="sw" style="background:#ff0000"></span>Red</td><td>Critical vehicle battery</td></tr>
<tr><td><span class="sw" style="background:#ff4b00"></span>Orange</td><td>Low vehicle battery</td></tr>
<tr><td><span class="sw pulse" style="background:#0000ff"></span>Blue pulse</td><td>Waiting for controller (status LED only; the lightbar is off until a controller connects)</td></tr>
<tr><td><span class="sw pulse" style="background:#0096dc"></span>Cyan pulse</td><td>Controller initializing: center sticks and release triggers</td></tr>
<tr><td><span class="sw" style="background:#00c800"></span>Green</td><td>Motors armed</td></tr>
<tr><td><span class="sw" style="background:#0000b4"></span>Solid blue</td><td>Controller ready, motors disarmed</td></tr>
</table><p class="note">Red and orange appear only when battery monitoring and the matching warning option (status LED or controller light) are enabled, so the two can differ. The slow red flash always shows. The LED brightness setting affects the status LED only.</p></section>)HTML";
    return h;
}


// Reference card: GPIO assignment (U3). Built from the pin #defines
// so it always matches the firmware.
String makeGpioMapHtml() {
    String h;
    h.reserve(1600);
    h += R"HTML(<section class="card"><h2>GPIO Map</h2><table><tr><th>GPIO</th><th>Function</th></tr>)HTML";

    auto row = [&h](int pin, const char* fn) {
        h += "<tr><td>";
        h += String(pin);
        h += "</td><td>";
        h += fn;
        h += "</td></tr>";
    };

    row(M1A_PIN, "M1A: left motor input A (MDD3A)");
    row(M1B_PIN, "M1B: left motor input B (MDD3A)");
    row(M2A_PIN, "M2A: right motor input A (MDD3A)");
    row(M2B_PIN, "M2B: right motor input B (MDD3A)");
    row(PIXEL_PIN, "WS281x LED data (LED 0 = status)");
    row(BATTERY_ADC_PIN, "Battery voltage sense (divider, ADC input only)");
    row(ABORT_BUTTON, "BOOT / PRG button: emergency abort");
    row(SERVO1_PIN, "Reserved: servo 1");
    row(AUX1_PIN, "Reserved: auxiliary");

    h += "</table><p>Motor PWM: ";
    h += String(PWM_FREQ / 1000);
    h += " kHz, ";
    h += String(PWM_RESOLUTION);
    h += "-bit. All grounds must be common.</p></section>";
    return h;
}


// H1: 2S LiPo voltage divider wiring (inline, offline).
String makeBatteryWiringSvg() {
    return R"SVG(<svg viewBox="0 0 360 200" role="img" aria-label="Battery divider: battery positive through R1 100 kilohm to the ADC sense node on GPIO 34; R2 33 kilohm and an optional 100 nanofarad capacitor from the sense node to ground; battery negative to ESP32 ground">
<g fill="none" stroke="#c5e88a" stroke-width="2.5"><path d="M55 40V30h65M180 30h90M220 30v30M220 110v60M220 45h-50v50M170 101v69M55 160v10h215"/></g>
<g fill="#324958" stroke="#c5e88a" stroke-width="2"><rect x="20" y="40" width="70" height="120" rx="6"/><rect x="120" y="22" width="60" height="16" rx="3"/><rect x="212" y="60" width="16" height="50" rx="3"/><rect x="270" y="15" width="80" height="30" rx="5"/><rect x="270" y="155" width="80" height="30" rx="5"/></g>
<g stroke="#c5e88a" stroke-width="3"><path d="M156 95h28M156 101h28"/></g>
<g fill="#c5e88a"><circle cx="220" cy="30" r="4"/><circle cx="220" cy="45" r="3"/><circle cx="220" cy="170" r="3"/><circle cx="170" cy="170" r="3"/></g>
<g fill="#eee" font-family="Arial" font-size="11" text-anchor="middle"><text x="55" y="92">2S LiPo</text><text x="55" y="108">8.4 V max</text><text x="46" y="55">+</text><text x="46" y="154">&#8722;</text><text x="150" y="15">R1 100 k&#937;</text><text x="310" y="34">GPIO 34</text><text x="310" y="174">ESP32 GND</text><text x="224" y="22">sense</text><text x="150" y="196">common ground</text></g>
<g fill="#eee" font-family="Arial" font-size="11"><text x="234" y="82">R2</text><text x="234" y="96">33 k&#937;</text><text x="150" y="122" text-anchor="end">100 nF</text><text x="150" y="136" text-anchor="end">optional</text></g>
</svg>)SVG";
}


// H2 / U1: DualSense outline highlighting the buttons for pairing
// (Create + PS) or configuration mode (OPTIONS + Triangle).
String makeControllerSvg( bool pairing ) {
    const char* on = "#c5e88a";
    const char* off = "#324958";

    String h;
    h.reserve(2400);
    h += R"SVG(<svg viewBox="0 0 360 195" role="img" aria-label=")SVG";
    h += pairing
        ? "DualSense: hold Create, left of the touchpad, and PS, below the touchpad"
        : "DualSense: hold OPTIONS, right of the touchpad, and Triangle, top face button";
    h += R"SVG("><path d="M60 40Q160 20 260 40Q300 50 310 110Q318 170 285 178Q262 184 240 150H80Q58 184 35 178Q2 170 10 110Q20 50 60 40Z" fill="#202d37" stroke="#9fb3c0" stroke-width="2"/>
<rect x="115" y="38" width="90" height="48" rx="6" fill="#2a3a46" stroke="#9fb3c0"/>
<text x="160" y="66" fill="#9fb3c0" font-family="Arial" font-size="10" text-anchor="middle">touchpad</text>
<path d="M62 80h16M70 72v16" stroke="#9fb3c0" stroke-width="7" stroke-linecap="round"/>
<g stroke="#9fb3c0"><circle cx="115" cy="125" r="14" fill="#2a3a46"/><circle cx="205" cy="125" r="14" fill="#2a3a46"/>)SVG";

    // Create (left of touchpad), Options (right), PS (center)
    h += "<rect x=\"95\" y=\"42\" width=\"8\" height=\"14\" rx=\"4\" fill=\"";
    h += pairing ? on : off;
    h += "\"/><rect x=\"217\" y=\"42\" width=\"8\" height=\"14\" rx=\"4\" fill=\"";
    h += pairing ? off : on;
    h += "\"/><circle cx=\"160\" cy=\"112\" r=\"8\" fill=\"";
    h += pairing ? on : off;
    // Face buttons: Triangle (top), Circle, Cross, Square
    h += "\"/><circle cx=\"250\" cy=\"62\" r=\"8\" fill=\"";
    h += pairing ? off : on;
    h += R"SVG("/><circle cx="272" cy="82" r="8" fill="#324958"/><circle cx="250" cy="102" r="8" fill="#324958"/><circle cx="228" cy="82" r="8" fill="#324958"/></g>
<path d="M250 57l-4 7h8z" fill="none" stroke="#eee" stroke-width="1.2"/>
<g stroke="#c5e88a" stroke-width="1.5" fill="none">)SVG";

    if (pairing) {
        h += R"SVG(<path d="M99 42L84 20M160 120v38"/></g><g fill="#eee" font-family="Arial" font-size="12"><text x="80" y="17" text-anchor="end">Create</text><text x="160" y="172" text-anchor="middle">PS</text></g>)SVG";
    } else {
        h += R"SVG(<path d="M221 42l15-22M258 62h48"/></g><g fill="#eee" font-family="Arial" font-size="12"><text x="238" y="17">OPTIONS</text><text x="310" y="66">Triangle</text></g>)SVG";
    }

    h += "</svg>";
    return h;
}


// Controller reference card: controls, pairing (H2).
String makeControllerCardHtml() {
    String h;
    h.reserve(3600);
    h += R"HTML(<section class="card"><h2>Controller</h2>
<table><tr><th>Control</th><th>Action</th></tr>
<tr><td>OPTIONS</td><td>Arm when released (sticks neutral). Disarm when pressed.</td></tr>
<tr><td>PS</td><td>E-stop: stops the motors and disarms at once. Never arms.</td></tr>
<tr><td>L1 / R1</td><td>Low / normal speed profile</td></tr>
<tr><td>OPTIONS + Triangle</td><td>Hold to enter or leave Wi-Fi configuration (disarmed)</td></tr>
<tr><td>Create + PS</td><td>Pairing mode</td></tr>
</table>
<p class="note">Denied arming rumbles: 2 pulses = controls not neutral, 3 pulses = battery lockout, Wi-Fi mode or abort. The PS e-stop gives one long rumble.</p>
<h3>Pairing a DualSense</h3>)HTML";
    h += makeControllerSvg(true);
    h += R"HTML(<p><b>Hold Create + PS until the controller light flashes rapidly.</b> Create is the small button on the left of the touchpad. OPTIONS, on the right, is not the pairing button. Once paired, press PS to reconnect.</p></section>)HTML";
    return h;
}


String makeWebPage() {
    String html;
    html.reserve(15000);
    html += R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ESPRC</title>
<style>body{font-family:Arial,sans-serif;background:#101820;color:#eee;margin:0}main{max-width:780px;margin:auto;padding:20px}.card{background:#202d37;padding:20px;border-radius:12px;margin:16px 0}label{display:block;margin:14px 0}input,select,button{font:inherit;padding:9px;border-radius:5px}input:not([type=checkbox]),select{display:block;box-sizing:border-box;width:100%;margin-top:5px}button{cursor:pointer;background:#c5e88a;color:#182119;border:0}svg{width:100%;height:auto}p{line-height:1.5}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}@media(max-width:600px){.grid{grid-template-columns:1fr}}#status{white-space:pre-line;line-height:1.6}table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:7px 6px;border-bottom:1px solid #34444f;vertical-align:middle}th{color:#9fb3c0;font-weight:normal}.sw{display:inline-block;width:16px;height:16px;border-radius:50%;margin-right:8px;vertical-align:middle;box-shadow:0 0 0 1px #0006}.blink{animation:blink .4s steps(1) infinite}.slow{animation:blink 1s steps(1) infinite}.pulse{animation:pulse 3s ease-in-out infinite}@keyframes blink{50%{opacity:.12}}@keyframes pulse{50%{opacity:.2}}@media(prefers-reduced-motion:reduce){.blink,.slow,.pulse{animation:none}}.note{color:#9fb3c0;font-size:.92em}</style>
</head><body><main><h1>ESPRC</h1><p>Firmware )HTML";
    html += FIRMWARE_VERSION;
    html += R"HTML(</p><section class="card"><h2>Live status</h2><div id="status">Connecting...</div></section>
<form method="post" action="/save"><section class="card"><h2>Drive Configuration</h2>
<div class="grid"><div><h3>Tank drive</h3>
<svg viewBox="0 0 300 145" role="img" aria-label="Tank drive: left stick Y controls left track; right stick Y controls right track">
<g fill="#324958" stroke="#c5e88a" stroke-width="3"><circle cx="75" cy="62" r="35"/><circle cx="225" cy="62" r="35"/><path d="M75 16v92m-9-82 9-10 9 10m-18 72 9 10 9-10M225 16v92m-9-82 9-10 9 10m-18 72 9 10 9-10" fill="none"/></g><g fill="white" font-size="12" text-anchor="middle"><text x="75" y="126">Left Track</text><text x="225" y="126">Right Track</text><text x="75" y="141">Forward / Reverse</text><text x="225" y="141">Forward / Reverse</text></g></svg></div>
<div><h3>Arcade drive</h3><svg viewBox="0 0 300 145" role="img" aria-label="Arcade drive: left stick Y is throttle for both tracks; right stick X steers left and right">
<g fill="#324958" stroke="#c5e88a" stroke-width="3"><circle cx="75" cy="62" r="35"/><circle cx="225" cy="62" r="35"/><path d="M75 16v92m-9-82 9-10 9 10m-18 72 9 10 9-10M179 62h92m-82-9-10 9 10 9m72-18 10 9-10 9" fill="none"/></g><g fill="white" font-size="12" text-anchor="middle"><text x="75" y="126">Both Tracks</text><text x="75" y="141">Forward / Reverse</text><text x="225" y="126">Right Stick</text><text x="225" y="141">Left / Right Steering</text></g></svg></div></div>
<p>Arcade: left stick Y controls throttle; right stick X controls steering. Right steering commands right yaw, including in reverse. Proportional steering requires throttle. Pivot steering permits turning in place.</p>)HTML";
    html += "<label>Low-speed limit (%)";
    html += "<input type=\"number\" name=\"lowSpeed\" min=\"5\" max=\"100\" step=\"1\" required value=\"";
    html += String(settings.lowSpeedPercent);
    html += "\"></label>";
    html += "<label>Normal-speed limit (%)";
    html += "<input type=\"number\" name=\"normalSpeed\" min=\"5\" max=\"100\" step=\"1\" required value=\"";
    html += String(settings.normalSpeedPercent);
    html += "\"></label>";
    html += "<label>Drive mode";
    html += "<select name=\"driveMode\">";
    html += "<option value=\"0\"";
    if (settings.driveMode == 0) html += " selected";
    html += ">Tank drive</option>";
    html += "<option value=\"1\"";
    if (settings.driveMode == 1) html += " selected";
    html += ">Arcade drive</option>";
    html += "</select></label>";
    html += "<label>Steering behavior";
    html += "<select name=\"steeringMode\">";
    html += "<option value=\"0\"";
    if (settings.steeringMode == 0) html += " selected";
    html += ">Proportional (no pivot at zero throttle)</option>";
    html += "<option value=\"1\"";
    if (settings.steeringMode == 1) html += " selected";
    html += ">Pivot (turn in place)</option>";
    html += "</select></label>";
    html += "<label>Steering sensitivity (%)";
    html += "<input type=\"number\" name=\"steeringSensitivity\" min=\"0\" max=\"200\" step=\"1\" required value=\"";
    html += String(settings.steeringSensitivity);
    html += "\"></label>";
    html += "<label>Maximum motor output (%)";
    html += "<input type=\"number\" name=\"maxOutput\" min=\"5\" max=\"100\" step=\"1\" required value=\"";
    html += String(settings.maxOutputPercent);
    html += "\"></label>";
    html += "<label>Joystick deadband (0-512 scale)";
    html += "<input type=\"number\" name=\"deadzone\" min=\"0\" max=\"200\" step=\"1\" required value=\"";
    html += String(settings.stickDeadzone);
    html += "\"></label>";
    html += "<label>Response curve";
    html += "<select name=\"curve\">";
    html += "<option value=\"0\"";
    if (settings.responseCurve == 0) html += " selected";
    html += ">Linear</option>";
    html += "<option value=\"1\"";
    if (settings.responseCurve == 1) html += " selected";
    html += ">Soft</option>";
    html += "<option value=\"2\"";
    if (settings.responseCurve == 2) html += " selected";
    html += ">Medium</option>";
    html += "<option value=\"3\"";
    if (settings.responseCurve == 3) html += " selected";
    html += ">Aggressive</option>";
    html += "</select></label>";
    html += "<label>Acceleration (ms)";
    html += "<input type=\"number\" name=\"accel\" min=\"0\" max=\"5000\" step=\"1\" required value=\"";
    html += String(settings.accelerationMs);
    html += "\"></label>";
    html += "<label>Deceleration (ms)";
    html += "<input type=\"number\" name=\"decel\" min=\"0\" max=\"5000\" step=\"1\" required value=\"";
    html += String(settings.decelerationMs);
    html += "\"></label>";
    html += "<label>Left motor trim (%)";
    html += "<input type=\"number\" name=\"trimLeft\" min=\"50\" max=\"120\" step=\"1\" required value=\"";
    html += String(settings.leftTrimPercent);
    html += "\"></label>";
    html += "<label>Right motor trim (%)";
    html += "<input type=\"number\" name=\"trimRight\" min=\"50\" max=\"120\" step=\"1\" required value=\"";
    html += String(settings.rightTrimPercent);
    html += "\"></label>";
    html += "<label><input type=\"checkbox\" name=\"invertLeft\"";
    if (settings.invertLeft) html += " checked";
    html += ">Reverse left motor</label>";
    html += "<label><input type=\"checkbox\" name=\"invertRight\"";
    if (settings.invertRight) html += " checked";
    html += ">Reverse right motor</label>";
    html += "<label>Pixel brightness";
    html += "<input type=\"number\" name=\"brightness\" min=\"1\" max=\"255\" step=\"1\" required value=\"";
    html += String(settings.pixelBrightness);
    html += "\"></label>";
    html += R"HTML(</section><section class="card"><h2>Battery Configuration</h2>)HTML";
    html += makeBatteryWiringSvg();
    html += R"HTML(<p>Divider ratio is about 4.03:1, so 8.4 V gives about 2.08 V at the ADC. <b>Never connect the battery directly to GPIO 34.</b> Battery negative must share ground with the ESP32.</p><p class="note">Monitoring is disabled until enabled below. With Limit motor power or Disable drive, a confirmed critical battery stays in effect until the ESP32 is power-cycled, even if the voltage recovers.</p>)HTML";
    html += "<label><input type=\"checkbox\" name=\"batteryEnabled\"";
    if (settings.batteryEnabled) html += " checked";
    html += ">Enable voltage monitoring</label>";
    html += "<label>Warning voltage (V)";
    html += "<input type=\"number\" name=\"lowV\" min=\"6.0\" max=\"8.4\" step=\"0.001\" required value=\"";
    html += String(settings.lowBatteryVoltage, 3);
    html += "\"></label>";
    html += "<label>Critical voltage (V)";
    html += "<input type=\"number\" name=\"criticalV\" min=\"6.0\" max=\"8.3\" step=\"0.001\" required value=\"";
    html += String(settings.criticalBatteryVoltage, 3);
    html += "\"></label>";
    html += "<label>Recovery hysteresis (V)";
    html += "<input type=\"number\" name=\"hysteresis\" min=\"0.01\" max=\"0.5\" step=\"0.001\" required value=\"";
    html += String(settings.batteryHysteresis, 3);
    html += "\"></label>";
    html += "<label>ADC calibration multiplier";
    html += "<input type=\"number\" name=\"batteryCal\" min=\"0.5\" max=\"1.5\" step=\"0.001\" required value=\"";
    html += String(settings.batteryCalibration, 3);
    html += "\"></label>";
    html += "<label>Threshold confirmation (sec)";
    html += "<input type=\"number\" name=\"batteryDelay\" min=\"1\" max=\"15\" step=\"1\" required value=\"";
    html += String(settings.batteryConfirmSeconds);
    html += "\"></label>";
    html += "<label>Critical battery action";
    html += "<select name=\"criticalBehavior\">";
    html += "<option value=\"0\"";
    if (settings.criticalBehavior == 0) html += " selected";
    html += ">Warn only</option>";
    html += "<option value=\"1\"";
    if (settings.criticalBehavior == 1) html += " selected";
    html += ">Limit motor power</option>";
    html += "<option value=\"2\"";
    if (settings.criticalBehavior == 2) html += " selected";
    html += ">Disable drive</option>";
    html += "</select></label>";
    html += "<label>Critical motor power limit (%)";
    html += "<input type=\"number\" name=\"criticalLimit\" min=\"10\" max=\"100\" step=\"1\" required value=\"";
    html += String(settings.criticalPowerLimitPercent);
    html += "\"></label>";
    html += "<label><input type=\"checkbox\" name=\"batteryPixel\"";
    if (settings.batteryPixelWarning) html += " checked";
    html += ">Battery warning on status pixel</label>";
    html += "<label><input type=\"checkbox\" name=\"batteryLight\"";
    if (settings.batteryControllerLight) html += " checked";
    html += ">Battery warning on DualSense lightbar</label>";
    html += "<label><input type=\"checkbox\" name=\"batteryRumble\"";
    if (settings.batteryRumble) html += " checked";
    html += ">Battery warning rumble</label>";
    html += R"HTML(</section><section class="card"><h2>Wi-Fi Configuration</h2>)HTML";
    html += makeControllerSvg(false);
    html += R"HTML(<p>Hold OPTIONS + Triangle with drive disarmed and sticks neutral. The status light flashes purple during the hold, then stays purple while configuration is active. Hold again to leave. Motors stay disabled throughout; after leaving, center the sticks and press OPTIONS to re-arm. Wi-Fi: ESPRC / ESPRC123.</p><p class="note">The inactivity timeout counts from your last page load or button press here; the live status updates do not keep Wi-Fi on.</p>)HTML";
    html += "<label>Wi-Fi inactivity timeout (sec)";
    html += "<input type=\"number\" name=\"wifiTimeout\" min=\"30\" max=\"3600\" step=\"1\" required value=\"";
    html += String(settings.wifiTimeoutSeconds);
    html += "\"></label>";
    html += "<label>OPTIONS + Triangle hold (sec)";
    html += "<input type=\"number\" name=\"wifiHold\" min=\"2\" max=\"10\" step=\"1\" required value=\"";
    html += String(settings.wifiHoldSeconds);
    html += "\"></label>";
    html += R"HTML(</section><button type="submit">Save Settings</button></form>
<section class="card"><h2>System</h2>
<form method="post" action="/defaults"><button>Restore Factory Defaults</button></form>
<form method="post" action="/pair-reset" onsubmit="return confirm('Forget saved controller pairing?')"><button>Clear controller pairing</button></form>
<p>Bluepad32 retains pairing keys across restarts. Press PS to reconnect. After clearing pairing, turn off the old controller and hold Create + PS on the replacement. Motors remain disarmed.</p>
<form method="post" action="/wifi-off"><button>Shut Down Wi-Fi</button></form></section>)HTML";
    html += makeControllerCardHtml();
    html += makeStatusLegendHtml();
    html += makeGpioMapHtml();
    html += R"HTML(<script>async function updateStatus(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error();const d=await r.json();document.getElementById('status').textContent='Controller: '+d.controller+' ('+d.controllerBattery+')\nDrive: '+d.drive+' / '+d.profile+'\nMotor output: L '+d.left+'%, R '+d.right+'%\nVehicle battery: '+d.vehicleVoltage+' / '+d.batteryState+'\nProtection: '+d.protection+'\nWi-Fi clients: '+d.clients+'\nUptime: '+d.uptime+'\nLast reset: '+d.resetReason;}catch(e){document.getElementById('status').textContent='Connection lost. Reconnect to ESPRC Wi-Fi.';}}updateStatus();setInterval(updateStatus,2000);</script>
</main></body></html>)HTML";
    return html;
}


// ======================================================================
// WEB STATUS JSON
// ======================================================================

void handleStatus() {

    // Polling is not user activity (R10).


    String json =
        "{";


    json +=
        "\"controller\":\"";

    if (
        !controller ||
        !controller->isConnected()
    ) {

        json +=
            "WAITING";

    } else if (
        controllerState ==
        CONTROLLER_INITIALIZING
    ) {

        json +=
            "INITIALIZING";

    } else {

        json +=
            "READY";
    }

    json +=
        "\",";


    int controllerBattery =
        controllerBatteryPercent();


    json +=
        "\"controllerBattery\":\"";

    if (
        controllerBattery < 0
    ) {

        json +=
            "Unknown";

    } else {

        json +=
            String(
                controllerBattery
            );

        json +=
            "%";
    }

    json +=
        "\",";


    json +=
        "\"drive\":\"";

    if (emergencyAbort) {

        json +=
            "ABORTED";

    } else if (
        controllerState ==
        CONTROLLER_INITIALIZING
    ) {

        json +=
            "INITIALIZING";

    } else if (
        motorsArmed
    ) {

        json +=
            "ARMED";

    } else {

        json +=
            "SAFE";
    }

    json +=
        "\",";


    json +=
        "\"profile\":\"";

    json +=
        lowSpeedProfile
            ? "LOW"
            : "NORMAL";

    json +=
        "\",";


    json +=
        "\"left\":" +
        String(
            leftMotorCommand *
            100 /
            255
        ) +
        ",";


    json +=
        "\"right\":" +
        String(
            rightMotorCommand *
            100 /
            255
        ) +
        ",";


    json +=
        "\"vehicleVoltage\":\"";

    if (
        !settings.batteryEnabled ||
        batteryState ==
            BATTERY_UNKNOWN
    ) {

        json +=
            !settings.batteryEnabled ? "Disabled" :
            batteryNoReading ? "No reading on GPIO 34" : "Qualifying...";

    } else {

        json +=
            String(
                batteryFilteredVoltage,
                2
            );

        json +=
            " V";
    }

    json +=
        "\",";


    json +=
        "\"batteryState\":\"";

    json +=
        batteryStateName(
            batteryState
        );

    json +=
        "\",";


    // Protection summary (R6, R7, S1)
    json += "\"protection\":\"";
    if (emergencyAbort) {
        json += "Emergency abort - power-cycle to clear";
    } else if (batteryDriveLockout()) {
        json += "Drive locked out: ";
        json += batteryLockoutReason();
    } else if (batteryPowerLimited()) {
        json += "Power limited to ";
        json += String(settings.criticalPowerLimitPercent);
        json += "% (critical battery";
        json += batteryLatchedBehavior ? ", until power cycle)" : ")";
    } else if (controllerStaleStop) {
        json += "Motors stopped: controller data stale";
    } else if (driveNeedsNeutral && motorsArmed) {
        json += "Drive held: return controls to neutral";
    } else {
        json += "None";
    }
    json += "\",";

    json += "\"resetReason\":\"";
    json += resetReasonText;
    json += "\",";

    json +=
        "\"clients\":" +
        String(
            WiFi.softAPgetStationNum()
        ) +
        ",";


    json +=
        "\"uptime\":\"" +
        uptimeString() +
        "\"";


    json +=
        "}";


    server.send(
        200,
        "application/json",
        json
    );
}


// ======================================================================
// WEB SAVE
// ======================================================================

bool readNumber(const char* name, double low, double high, bool integer, double& value) {
    if (!server.hasArg(name)) return false;
    String input = server.arg(name);
    input.trim();
    if (!input.length()) return false;
    char* end = nullptr;
    value = strtod(input.c_str(), &end);
    return end != input.c_str() && *end == 0 && isfinite(value) &&
        value >= low && value <= high && (!integer || floor(value) == value);
}

void resetBatteryMonitor() {
    batteryState = batteryCandidate = BATTERY_UNKNOWN;
    batteryFilterInitialized = false;
    batteryVoltage = batteryFilteredVoltage = 0;
    batteryCandidateSince = lastBatteryReminder = millis();
    rumblePattern.active = false;
    if (controller && controller->isConnected()) controller->playDualRumble(0, 0, 0, 0);
    lastPixelColor = lastControllerColor = 0xFFFFFFFF;
}

void handleSave() {
    noteWebActivity();
    if (!wifiConfigActive || motorsArmed) {
        server.send(409, "text/plain", "Configuration mode required.");
        return;
    }
    stopMotorsImmediate();
    TankSettings candidate = settings;
    double value;
    if (!readNumber("lowSpeed", 5, 100, true, value)) {
        server.send(400, "text/plain", "Invalid lowSpeed; settings were not saved.");
        return;
    }
    candidate.lowSpeedPercent = value;
    if (!readNumber("normalSpeed", 5, 100, true, value)) {
        server.send(400, "text/plain", "Invalid normalSpeed; settings were not saved.");
        return;
    }
    candidate.normalSpeedPercent = value;
    if (!readNumber("driveMode", 0, 1, true, value)) {
        server.send(400, "text/plain", "Invalid driveMode; settings were not saved.");
        return;
    }
    candidate.driveMode = value;
    if (!readNumber("steeringMode", 0, 1, true, value)) {
        server.send(400, "text/plain", "Invalid steeringMode; settings were not saved.");
        return;
    }
    candidate.steeringMode = value;
    if (!readNumber("steeringSensitivity", 0, 200, true, value)) {
        server.send(400, "text/plain", "Invalid steeringSensitivity; settings were not saved.");
        return;
    }
    candidate.steeringSensitivity = value;
    if (!readNumber("maxOutput", 5, 100, true, value)) {
        server.send(400, "text/plain", "Invalid maxOutput; settings were not saved.");
        return;
    }
    candidate.maxOutputPercent = value;
    if (!readNumber("deadzone", 0, 200, true, value)) {
        server.send(400, "text/plain", "Invalid deadzone; settings were not saved.");
        return;
    }
    candidate.stickDeadzone = value;
    if (!readNumber("curve", 0, 3, true, value)) {
        server.send(400, "text/plain", "Invalid curve; settings were not saved.");
        return;
    }
    candidate.responseCurve = value;
    if (!readNumber("accel", 0, 5000, true, value)) {
        server.send(400, "text/plain", "Invalid accel; settings were not saved.");
        return;
    }
    candidate.accelerationMs = value;
    if (!readNumber("decel", 0, 5000, true, value)) {
        server.send(400, "text/plain", "Invalid decel; settings were not saved.");
        return;
    }
    candidate.decelerationMs = value;
    if (!readNumber("trimLeft", 50, 120, true, value)) {
        server.send(400, "text/plain", "Invalid trimLeft; settings were not saved.");
        return;
    }
    candidate.leftTrimPercent = value;
    if (!readNumber("trimRight", 50, 120, true, value)) {
        server.send(400, "text/plain", "Invalid trimRight; settings were not saved.");
        return;
    }
    candidate.rightTrimPercent = value;
    if (!readNumber("brightness", 1, 255, true, value)) {
        server.send(400, "text/plain", "Invalid brightness; settings were not saved.");
        return;
    }
    candidate.pixelBrightness = value;
    if (!readNumber("lowV", 6.0, 8.4, false, value)) {
        server.send(400, "text/plain", "Invalid lowV; settings were not saved.");
        return;
    }
    candidate.lowBatteryVoltage = value;
    if (!readNumber("criticalV", 6.0, 8.3, false, value)) {
        server.send(400, "text/plain", "Invalid criticalV; settings were not saved.");
        return;
    }
    candidate.criticalBatteryVoltage = value;
    if (!readNumber("hysteresis", 0.01, 0.5, false, value)) {
        server.send(400, "text/plain", "Invalid hysteresis; settings were not saved.");
        return;
    }
    candidate.batteryHysteresis = value;
    if (!readNumber("batteryCal", 0.5, 1.5, false, value)) {
        server.send(400, "text/plain", "Invalid batteryCal; settings were not saved.");
        return;
    }
    candidate.batteryCalibration = value;
    if (!readNumber("batteryDelay", 1, 15, true, value)) {
        server.send(400, "text/plain", "Invalid batteryDelay; settings were not saved.");
        return;
    }
    candidate.batteryConfirmSeconds = value;
    if (!readNumber("criticalBehavior", 0, 2, true, value)) {
        server.send(400, "text/plain", "Invalid criticalBehavior; settings were not saved.");
        return;
    }
    candidate.criticalBehavior = value;
    if (!readNumber("criticalLimit", 10, 100, true, value)) {
        server.send(400, "text/plain", "Invalid criticalLimit; settings were not saved.");
        return;
    }
    candidate.criticalPowerLimitPercent = value;
    if (!readNumber("wifiTimeout", 30, 3600, true, value)) {
        server.send(400, "text/plain", "Invalid wifiTimeout; settings were not saved.");
        return;
    }
    candidate.wifiTimeoutSeconds = value;
    if (!readNumber("wifiHold", 2, 10, true, value)) {
        server.send(400, "text/plain", "Invalid wifiHold; settings were not saved.");
        return;
    }
    candidate.wifiHoldSeconds = value;
    candidate.invertLeft = server.hasArg("invertLeft");
    candidate.invertRight = server.hasArg("invertRight");
    candidate.batteryEnabled = server.hasArg("batteryEnabled");
    candidate.batteryPixelWarning = server.hasArg("batteryPixel");
    candidate.batteryControllerLight = server.hasArg("batteryLight");
    candidate.batteryRumble = server.hasArg("batteryRumble");
    if (!validSettings(candidate)) {
        String message = settingsProblem(candidate);
        message += "; settings were not saved.";
        server.send(400, "text/plain", message);
        return;
    }
    settings = candidate;
    saveSettings();
    resetBatteryMonitor();
    armButtonReleased = false;
    previousOptions = true;
    pixels.setBrightness(settings.pixelBrightness);
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "Saved");
}


// ======================================================================
// RESTORE DEFAULTS
// ======================================================================

void handleDefaults() {
    if (!wifiConfigActive || motorsArmed) {
        server.send(409, "text/plain", "Configuration mode required.");
        return;
    }
    stopMotorsImmediate();
    resetBatteryMonitor();

    noteWebActivity();


    setFactoryDefaults();

    saveSettings();


    lowSpeedProfile =
        false;


    batteryState =
        BATTERY_UNKNOWN;

    batteryFilterInitialized =
        false;


    server.sendHeader(
        "Location",
        "/"
    );

    server.send(
        303,
        "text/plain",
        "Defaults restored"
    );
}


// ======================================================================
// WI-FI OFF ROUTE
// ======================================================================

void handleWifiOff() {

    server.send(
        200,
        "text/html",
        R"HTML(<html><body style="font-family:Arial;background:#111;color:white;padding:30px"><h2>ESPRC</h2><p>Configuration Wi-Fi is shutting down.</p><p>The vehicle remains SAFE.</p></body></html>)HTML"
    );

    // Give the response time to reach the browser before the server stops.
    requestWiFiShutdown("web page", 500);
}


// ======================================================================
// WEB ROUTES
// ======================================================================

void resetControllerPairing() {
    if (motorsArmed) return;
    stopMotorsImmediate();
    armButtonReleased = false;
    previousOptions = true;
    BP32.forgetBluetoothKeys();
    // With a controller active, onDisconnectedController() re-enables
    // new connections once it has gone (R2).
    if (controller) controller->disconnect();
    else BP32.enableNewBluetoothConnections(true);
    Serial.println("Bluetooth keys cleared. Turn off old controller; hold Create + PS on replacement.");
}

void servicePairingSerial() {
    static char command[32];
    static size_t length = 0;
    static bool overflow = false;
    for (int i = 0; i < 32 && Serial.available(); ++i) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            command[length] = 0;
            if (!overflow && strcmp(command, "PAIR RESET") == 0) {
                if (motorsArmed) Serial.println("Pairing reset blocked while ARMED.");
                else resetControllerPairing();
            }
            length = 0;
            overflow = false;
        } else if (length < sizeof(command) - 1) command[length++] = c;
        else overflow = true;
    }
}

void configureWebRoutes() {
    // Handlers execute only while configuration mode locks out drive.


    static bool configured =
        false;


    if (configured) {
        return;
    }


    configured = true;
    server.on("/pair-reset", HTTP_POST, []() {
        if (!wifiConfigActive || motorsArmed) {
            server.send(409, "text/plain", "Disarm and enter configuration mode first.");
            return;
        }
        noteWebActivity();
        resetControllerPairing();
        server.send(200, "text/html", "<p>Pairing cleared. Turn off the old controller, then hold Create + PS on the replacement.</p><a href='/'>Back</a>");
    });


    server.on(
        "/",
        HTTP_GET,
        []() {

            noteWebActivity();

            server.send(
                200,
                "text/html",
                makeWebPage()
            );
        }
    );


    server.on(
        "/status",
        HTTP_GET,
        handleStatus
    );


    server.on(
        "/save",
        HTTP_POST,
        handleSave
    );


    server.on(
        "/defaults",
        HTTP_POST,
        handleDefaults
    );


    server.on(
        "/wifi-off",
        HTTP_POST,
        handleWifiOff
    );


    // Phones probe URLs such as /generate_204 while connected; those are
    // not user activity and must not keep Wi-Fi on (R10).
    server.onNotFound(
        []() {

            server.send(
                404,
                "text/plain",
                "Not found"
            );
        }
    );
}


// ======================================================================
// START CONFIG WI-FI
// ======================================================================

void startConfigWiFi() {
    if (motorsArmed || emergencyAbort) {
        Serial.println("Wi-Fi activation blocked: vehicle must be disarmed and not aborted.");
        return;
    }

    if ( wifiConfigActive ) {
        return;
    }

    if ( wifiShutdownState != WIFI_SHUTDOWN_NONE ) {
        Serial.println("Wi-Fi activation blocked: shutdown still in progress.");
        return;
    }

    Serial.println("Starting ESPRC configuration AP...");

    motorsArmed = false;
    stopMotorsImmediate();

    WiFi.mode( WIFI_AP );

    bool success = WiFi.softAP( AP_SSID, AP_PASSWORD );

    if (!success) {
        WiFi.mode( WIFI_OFF );
        Serial.println( "Wi-Fi AP start failed." );
        return;
    }

    configureWebRoutes();
    server.begin();

    wifiConfigActive = true;
    wifiLastActivity = millis();

    Serial.println();
    Serial.println( "Configuration Wi-Fi ON" );
    Serial.print( "SSID: " );
    Serial.println( AP_SSID );
    Serial.print( "Address: " );
    Serial.println( WiFi.softAPIP() );

    queueRumble( 2, 120, 100, 60, 60 );
}


// ======================================================================
// STOP CONFIG WI-FI
// ======================================================================

// Every exit path (controller combo, web button, idle timeout) calls this.
// It only records the request; serviceWiFiShutdown() does the work from
// loop(), one step at a time (H3). Motors stay locked out throughout
// because wifiConfigActive remains true until the last step.
void requestWiFiShutdown( const char* reason, unsigned long delayMs ) {

    if ( !wifiConfigActive || wifiShutdownState != WIFI_SHUTDOWN_NONE ) {
        return;
    }

    armButtonReleased = false;
    previousOptions = true;
    motorsArmed = false;
    stopMotorsImmediate();

    wifiShutdownState = WIFI_SHUTDOWN_PENDING;
    wifiShutdownStageStart = millis();
    wifiShutdownStageDelay = delayMs;

    Serial.printf(
        "Wi-Fi shutdown requested (%s). Free heap: %u\n",
        reason,
        ESP.getFreeHeap()
    );
}


void serviceWiFiShutdown() {

    if ( wifiShutdownState == WIFI_SHUTDOWN_NONE ) {
        return;
    }

    if ( millis() - wifiShutdownStageStart < wifiShutdownStageDelay ) {
        return;
    }

    switch ( wifiShutdownState ) {

        case WIFI_SHUTDOWN_PENDING:
            Serial.println( "Wi-Fi shutdown: stopping server" );
            server.stop();
            wifiShutdownState = WIFI_SHUTDOWN_DISCONNECT_AP;
            break;

        case WIFI_SHUTDOWN_DISCONNECT_AP:
            Serial.println( "Wi-Fi shutdown: disconnecting AP" );
            // Radio stays on here; the next stage turns it off.
            WiFi.softAPdisconnect( false );
            wifiShutdownState = WIFI_SHUTDOWN_DISABLE_RADIO;
            break;

        case WIFI_SHUTDOWN_DISABLE_RADIO:
            Serial.println( "Wi-Fi shutdown: disabling radio" );
            WiFi.mode( WIFI_OFF );
            finishWiFiShutdown();
            return;

        default:
            wifiShutdownState = WIFI_SHUTDOWN_NONE;
            return;
    }

    wifiShutdownStageStart = millis();
    wifiShutdownStageDelay = WIFI_SHUTDOWN_STAGE_MS;
}


void finishWiFiShutdown() {

    wifiShutdownState = WIFI_SHUTDOWN_NONE;
    wifiConfigActive = false;

    armButtonReleased = false;
    previousOptions = true;
    motorsArmed = false;
    stopMotorsImmediate();

    Serial.println( "Wi-Fi configuration OFF" );

    // Require a fresh settle / neutral check before the vehicle can be
    // armed again after configuration mode.
    if ( controller && controller->isConnected() ) {
        controllerState = CONTROLLER_INITIALIZING;
        controllerConnectedAt = millis();
        controllerNeutralSince = 0;
        controllerWaitingForNeutralLogged = false;
        Serial.println( "Controller revalidation required" );
    }

    queueRumble( 1, 160, 0, 50, 50 );
}


// ======================================================================
// SERVICE WEB SERVER
// ======================================================================

void serviceConfigWiFi() {

    if ( wifiShutdownState != WIFI_SHUTDOWN_NONE ) {
        serviceWiFiShutdown();
        return;
    }

    if ( !wifiConfigActive ) {
        return;
    }

    static int lastClients = -1;
    int clients = WiFi.softAPgetStationNum();
    if (clients != lastClients) {
        Serial.printf("Wi-Fi clients: %d (previous %d)\n", clients, lastClients);
        lastClients = clients;
    }

    server.handleClient();

    // Only user actions count as activity; /status polling does not (R10).
    unsigned long timeout = settings.wifiTimeoutSeconds * 1000UL;

    if ( millis() - wifiLastActivity > timeout ) {
        Serial.println( "Wi-Fi idle timeout." );
        requestWiFiShutdown( "idle timeout", 0 );
    }
}


// ======================================================================
// CHECK STICK CENTERING
// ======================================================================

const char* controllerStateName(
    ControllerState state
) {

    switch (state) {

        case CONTROLLER_INITIALIZING:
            return "INITIALIZING";

        case CONTROLLER_READY:
            return "READY";

        default:
            return "DISCONNECTED";
    }
}


bool controllerInputsNeutral() {

    if (
        !controller ||
        !controller->isConnected()
    ) {
        return false;
    }

    return
        abs(controller->axisX()) <= settings.stickDeadzone &&
        abs(controller->axisY()) <= settings.stickDeadzone &&
        abs(controller->axisRX()) <= settings.stickDeadzone &&
        abs(controller->axisRY()) <= settings.stickDeadzone &&
        controller->brake() <= CONTROLLER_TRIGGER_NEUTRAL_MAX &&
        controller->throttle() <= CONTROLLER_TRIGGER_NEUTRAL_MAX;
}


bool sticksCentered() {
    return controllerInputsNeutral();
}


// ======================================================================
// CONTROLLER STARTUP / RECONNECT STATE
// ======================================================================

void serviceControllerState() {

    if (
        !controller ||
        !controller->isConnected()
    ) {

        controllerState =
            CONTROLLER_DISCONNECTED;

        controllerNeutralSince =
            0;

        controllerWaitingForNeutralLogged =
            false;

        return;
    }


    if (
        controllerState !=
        CONTROLLER_INITIALIZING
    ) {
        return;
    }


    // Do not accept a controller whose input stream has timed out.
    if (
        controllerTimeoutActive ||
        millis() -
        lastControllerReport >
        CONTROLLER_TIMEOUT_MS
    ) {
        return;
    }


    unsigned long now =
        millis();


    if (
        now -
        controllerConnectedAt <
        CONTROLLER_SETTLE_MS
    ) {
        return;
    }


    if (
        !controllerInputsNeutral()
    ) {

        controllerNeutralSince =
            0;

        if (
            !controllerWaitingForNeutralLogged
        ) {

            Serial.println(
                "Controller initialized; waiting for sticks/triggers to return to neutral."
            );

            controllerWaitingForNeutralLogged =
                true;
        }

        return;
    }


    if (
        controllerNeutralSince ==
        0
    ) {

        controllerNeutralSince =
            now;

        return;
    }


    if (
        now -
        controllerNeutralSince <
        CONTROLLER_NEUTRAL_STABLE_MS
    ) {
        return;
    }


    // Establish fresh button-edge baselines after Bluetooth settles.
    previousOptions =
        controller->miscStart();

    previousL1 =
        controller->l1();

    previousR1 = controller->r1();

    previousPS = controller->miscSystem();
    driveNeedsNeutral = false;
    optionsPressValid = false;
    triangleDuringOptions = false;

    armButtonReleased =
        !controller->miscStart() &&
        !controller->y();

    wifiComboActive =
        false;

    wifiComboTriggered =
        false;

    wifiComboStart =
        0;


    controllerState =
        CONTROLLER_READY;

    controllerWaitingForNeutralLogged =
        false;


    Serial.println(
        "Controller READY - vehicle remains DISARMED."
    );


    queueRumble(
        1,
        120,
        0,
        45,
        35
    );


    // If battery was already low before controller became ready,
    // notify now.

    if (
        settings.batteryEnabled &&
        batteryState ==
        BATTERY_LOW &&
        settings.batteryRumble
    ) {

        queueRumble(
            2,
            180,
            150,
            90,
            60
        );
    }


    if (
        settings.batteryEnabled &&
        batteryState ==
        BATTERY_CRITICAL &&
        settings.batteryRumble
    ) {

        queueRumble(
            3,
            230,
            120,
            180,
            150
        );
    }
}


// ======================================================================
// CONFIG ACCESS WINDOW
// ======================================================================




// ======================================================================
// WI-FI BUTTON COMBINATION
// ======================================================================

void processWiFiCombo() {

    if (!controller) {

        wifiComboActive =
            false;

        wifiComboTriggered =
            false;

        wifiComboStart =
            0;

        return;
    }


    bool combo =
        controller->y() &&
        controller->miscStart();


    if (!combo) {

        wifiComboActive =
            false;

        wifiComboTriggered =
            false;

        wifiComboStart =
            0;

        return;
    }


    // Cannot ENTER config mode while armed.
    // We do allow the combo while Wi-Fi is already active
    // so that it can be turned off.

    if (
        !wifiConfigActive &&
        motorsArmed
    ) {

        wifiComboActive =
            false;

        return;
    }


    // Require centered sticks.

    if (
        !sticksCentered()
    ) {

        wifiComboActive =
            false;

        return;
    }


    if (
        !wifiComboActive
    ) {

        wifiComboActive =
            true;

        wifiComboTriggered =
            false;

        wifiComboStart =
            millis();
    }


    unsigned long required =
        settings.wifiHoldSeconds *
        1000UL;


    if ( !wifiComboTriggered && millis() - wifiComboStart >= required ) {

        wifiComboTriggered =
            true;


        if (
            wifiConfigActive
        ) {

            requestWiFiShutdown( "controller OPTIONS + Triangle", 0 );

        } else {

            startConfigWiFi();
        }
    }
}


// ======================================================================
// CONTROLLER CONNECTION
// ======================================================================

void onConnectedController(
    ControllerPtr ctl
) {

    // R2: only one controller may drive. An extra one is disconnected so
    // it cannot occupy a slot and strand the vehicle if the first drops.
    if ( controller != nullptr ) {

        Serial.println( "Second controller rejected - disconnecting it." );
        ctl->disconnect();
        return;
    }


    if (!ctl->isGamepad()) {
        ctl->disconnect();
        return;
    }


    controller = ctl;

    // Stop scanning for new pairings while this controller is active (R2).
    // A previously paired pad can still connect; the check above
    // disconnects it. Re-enabled in onDisconnectedController().
    BP32.enableNewBluetoothConnections(false);


    // A Bluetooth connection is NOT permission to drive.  Enter an
    // initialization state until reports are stable and every control
    // input has returned to neutral.
    controllerState =
        CONTROLLER_INITIALIZING;

    controllerConnectedAt =
        millis();

    controllerNeutralSince =
        0;

    controllerWaitingForNeutralLogged =
        false;

    controllerTimeoutActive =
        false;

    lastControllerReport =
        millis();


    armButtonReleased = false;

    previousOptions = true;

    previousPS = true;
    controllerStaleStop = false;
    driveNeedsNeutral = false;
    optionsPressValid = false;
    triangleDuringOptions = false;

    previousL1 = false;

    previousR1 =
        false;


    wifiComboActive =
        false;

    wifiComboTriggered =
        false;

    wifiComboStart =
        0;


    motorsArmed =
        false;

    stopMotorsImmediate();


    rumblePattern.active =
        false;


    lastControllerColor =
        0xFFFFFFFF;


    Serial.println();
    Serial.println(
        "Controller connected - INITIALIZING; motors locked out."
    );
}


// ======================================================================
// CONTROLLER DISCONNECT
// ======================================================================

void onDisconnectedController(
    ControllerPtr ctl
) {

    if (
        ctl !=
        controller
    ) {
        return;
    }


    controller = nullptr;

    controllerState =
        CONTROLLER_DISCONNECTED;

    controllerConnectedAt =
        0;

    controllerNeutralSince =
        0;

    controllerWaitingForNeutralLogged =
        false;

    controllerTimeoutActive =
        false;

    armButtonReleased = false;
    previousOptions = true;
    BP32.enableNewBluetoothConnections(true);


    motorsArmed =
        false;


    stopMotorsImmediate();


    wifiComboActive =
        false;

    wifiComboTriggered =
        false;


    wifiComboStart = 0;
    controllerStaleStop = false;
    driveNeedsNeutral = false;
    optionsPressValid = false;
    triangleDuringOptions = false;

    rumblePattern.active = false;


    Serial.println( "Controller disconnected - motors stopped; vehicle DISARMED." );
}


// ======================================================================
// ARMING
// ======================================================================

// Called on a valid OPTIONS release while disarmed (R1).
// Every refusal is logged and rumbles so the driver knows (R3).
void tryArmFromButton() {

    if (motorsArmed) {
        return;
    }

    if (emergencyAbort || wifiConfigActive) {

        Serial.println(
            emergencyAbort
                ? "Arming blocked: emergency abort (power-cycle to clear)."
                : "Arming blocked: Wi-Fi configuration active or shutting down."
        );

        queueRumble(3, 80, 80, 30, 30);
        return;
    }


    if (batteryDriveLockout()) {

        Serial.printf("Arming blocked: %s.\n", batteryLockoutReason());

        // Three pulses: battery lockout.
        queueRumble(
            3,
            80,
            80,
            30,
            30
        );

        return;
    }


    if (!sticksCentered()) {

        Serial.println(
            "Arming blocked: sticks/triggers must be neutral."
        );

        // Two pulses: controls not neutral.
        queueRumble(
            2,
            80,
            80,
            30,
            30
        );

        return;
    }


    motorsArmed = true;
    stopMotorsImmediate();
    Serial.println("Drive state: ARMED");

    queueRumble(
        1,
        120,
        0,
        60,
        40
    );
}


// ======================================================================
// BUTTON PROCESSING
// ======================================================================

void processButtons() {

    if (
        !controller ||
        controllerState != CONTROLLER_READY
    ) {
        optionsPressValid = false;
        triangleDuringOptions = false;
        return;
    }


    // --------------------------------------------------------------
    // PS = controller e-stop (R9). Stop only: it never arms.
    // --------------------------------------------------------------

    bool ps =
        controller->miscSystem();

    if (ps && !previousPS) {

        // A PS press also cancels any OPTIONS press in progress.
        optionsPressValid = false;

        if (motorsArmed) {
            motorsArmed = false;
            stopMotorsImmediate();
            Serial.println("E-stop (PS): motors stopped; vehicle DISARMED.");
            // One long strong pulse, distinct from the refusal patterns.
            queueRumble(1, 450, 0, 220, 220);
        }
    }

    previousPS = ps;


    // Wi-Fi combo gets priority because it uses OPTIONS.
    processWiFiCombo();


    bool triangle =
        controller->y();


    // --------------------------------------------------------------
    // OPTIONS = Arm / Disarm (R1)
    //
    // Armed:    disarm immediately on the OPTIONS press.
    // Disarmed: arm on OPTIONS release, and only if Triangle was not
    //           pressed at any point during that press. This keeps the
    //           OPTIONS + Triangle Wi-Fi combo from arming the vehicle,
    //           whichever button goes down first.
    // --------------------------------------------------------------

    bool options =
        controller->miscStart();

    bool optionsPressed =
        options && !previousOptions;

    bool optionsReleased =
        !options && previousOptions;


    if (optionsPressed) {

        optionsPressValid =
            armButtonReleased && !triangle;

        triangleDuringOptions =
            triangle;

        if (motorsArmed) {

            // Disarm never waits for release.
            motorsArmed = false;
            stopMotorsImmediate();
            Serial.println("Drive state: DISARMED");

            // This press is used up; releasing it must not re-arm.
            optionsPressValid = false;
        }
    }


    if (options && triangle) {
        triangleDuringOptions = true;
    }


    if (optionsReleased) {

        if (
            optionsPressValid &&
            !triangleDuringOptions &&
            armButtonReleased
        ) {
            tryArmFromButton();
        }

        optionsPressValid = false;
        triangleDuringOptions = false;
    }


    if (!options && !triangle) armButtonReleased = true;
    previousOptions = options;


    // --------------------------------------------------------------
    // L1 = low profile
    // --------------------------------------------------------------

    bool l1 =
        controller->l1();


    if (
        l1 &&
        !previousL1
    ) {

        lowSpeedProfile =
            true;

        Serial.println(
            "Speed profile: LOW"
        );
    }


    previousL1 =
        l1;


    // --------------------------------------------------------------
    // R1 = normal profile
    // --------------------------------------------------------------

    bool r1 =
        controller->r1();


    if (
        r1 &&
        !previousR1
    ) {

        lowSpeedProfile =
            false;

        Serial.println(
            "Speed profile: NORMAL"
        );
    }


    previousR1 =
        r1;
}


// ======================================================================
// OPTIONAL VERBOSE CONTROLLER DEBUG
// ======================================================================
//
// Normal USB Serial output is event-driven and prints only meaningful
// state changes.  Set VERBOSE_CONTROLLER_DEBUG=true near the controller
// globals when live input diagnostics are needed.

void printVerboseControllerDebug() {

    if (
        !VERBOSE_CONTROLLER_DEBUG ||
        !controller ||
        !controller->isConnected()
    ) {
        return;
    }


    unsigned long now =
        millis();


    if (
        now -
        lastVerboseControllerDebug <
        VERBOSE_CONTROLLER_DEBUG_MS
    ) {
        return;
    }


    lastVerboseControllerDebug =
        now;


    Serial.printf(
        "CTRL:%s LX:%d LY:%d RX:%d RY:%d L2:%d R2:%d BTN:0x%04X ARM:%s L:%d R:%d\n",
        controllerStateName(
            controllerState
        ),
        controller->axisX(),
        controller->axisY(),
        controller->axisRX(),
        controller->axisRY(),
        controller->brake(),
        controller->throttle(),
        controller->buttons(),
        motorsArmed
            ? "YES"
            : "NO",
        leftMotorCommand,
        rightMotorCommand
    );
}


// ======================================================================
// STALE CONTROLLER DATA STOP (S1)
// ======================================================================
//
// The 2 s CONTROLLER_TIMEOUT_MS disarms. This faster check only stops
// the motors, so a short Bluetooth stall does not leave the vehicle
// driving on its last command.

void serviceStaleStop() {

    if ( !controller || !controller->isConnected() ||
         controllerState != CONTROLLER_READY || controllerTimeoutActive ) {
        return;
    }

    bool stale = millis() - lastControllerReport > CONTROLLER_STALE_STOP_MS;

    if ( controllerStaleStop ) {
        if ( !stale ) {
            controllerStaleStop = false;
            Serial.println( "Controller reports resumed - drive held at zero until controls are neutral." );
        }
        return;
    }

    if ( stale ) {

        controllerStaleStop = true;
        driveNeedsNeutral = true;
        stopMotorsImmediate();

        Serial.println(
            motorsArmed
                ? "Controller data stale (> 300 ms) - motors stopped; still ARMED."
                : "Controller data stale (> 300 ms)."
        );
    }
}


// ======================================================================
// RESET REASON (S2)
// ======================================================================

const char* describeResetReason( esp_reset_reason_t reason ) {
    switch ( reason ) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external reset pin";
        case ESP_RST_SW:        return "software restart";
        case ESP_RST_PANIC:     return "crash (panic)";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "wake from deep sleep";
        case ESP_RST_BROWNOUT:  return "brownout (supply voltage dropped)";
        case ESP_RST_SDIO:      return "SDIO reset";
        default:                return "unknown";
    }
}


// ======================================================================
// SETUP
// ======================================================================

void setup() {

    // Establish zero motor outputs before Serial, NVS, or Bluetooth startup.
    setupPWM();
    stopMotorsImmediate();

    Serial.begin(
        115200
    );


    delay(
        500
    );


    Serial.println();
    Serial.println(
        "================================="
    );

    Serial.println(
        "ESPRC RC Tank"
    );

    Serial.print(
        "Firmware "
    );

    Serial.println( FIRMWARE_VERSION );

    resetReasonText = describeResetReason( esp_reset_reason() );
    Serial.printf( "Reset reason: %s\n", resetReasonText );

    Serial.println(
        "================================="
    );


    // Emergency input

    pinMode(
        ABORT_BUTTON,
        INPUT_PULLUP
    );


    // Settings

    loadSettings();


    // Pixels

    pixels.begin();

    pixels.setBrightness(
        settings.pixelBrightness
    );

    pixels.clear();

    pixels.show();


    // Battery ADC

    analogReadResolution(
        12
    );

    analogSetPinAttenuation(
        BATTERY_ADC_PIN,
        ADC_11db
    );


    // Wi-Fi begins OFF

    WiFi.mode(
        WIFI_OFF
    );


    // Bluepad32

    BP32.setup(
        &onConnectedController,
        &onDisconnectedController
    );


    // H4: disable virtual devices (touchpad-as-mouse) before accepting
    // connections. "DS5: Failed to create virtual device" is harmless;
    // the gamepad itself still connects.
    BP32.enableVirtualDevice(false);
    BP32.enableNewBluetoothConnections(true);
    Serial.printf("Bluepad32: %s\n", BP32.firmwareVersion());
    const uint8_t* bt = BP32.localBdAddress();
    Serial.printf("Bluetooth address: %02X:%02X:%02X:%02X:%02X:%02X\n",
        bt[0], bt[1], bt[2], bt[3], bt[4], bt[5]);
    Serial.println("Saved Bluetooth keys retained. Press PS to reconnect; Create + PS for first pairing.");
    Serial.println("Send PAIR RESET on Serial while disarmed to forget pairing.");

    // IMPORTANT:
    //
    // Do NOT call:
    //
    // BP32.forgetBluetoothKeys();
    //
    // during normal boot.


    Serial.println();
    Serial.println(
        "Waiting for controller..."
    );

    Serial.println(
        "On connection: 1 second settle + neutral validation before READY."
    );

    Serial.println(
        "OPTIONS = Arm (on release) / Disarm (on press)"
    );

    Serial.println(
        "PS = E-stop (stop and disarm)"
    );

    Serial.println(
        "L1 = Low speed"
    );

    Serial.println(
        "R1 = Normal speed"
    );

    Serial.println(
        "OPTIONS + TRIANGLE = Configuration"
    );


    lastRampUpdate =
        millis();


    updateStatusIndicators();
}


// ======================================================================
// MAIN LOOP
// ======================================================================

void loop() {

    // --------------------------------------------------------------
    // Emergency safety
    // --------------------------------------------------------------

    checkAbortButton();
    servicePairingSerial();


    // --------------------------------------------------------------
    // Bluepad32
    // --------------------------------------------------------------

    bool dataUpdated =
        BP32.update();


    if (
        dataUpdated &&
        controller &&
        controller->hasData()
    ) {

        lastControllerReport =
            millis();

        if (
            controllerTimeoutActive
        ) {

            controllerTimeoutActive =
                false;

            controllerState =
                CONTROLLER_INITIALIZING;

            controllerConnectedAt =
                millis();

            controllerNeutralSince =
                0;

            controllerWaitingForNeutralLogged =
                false;

            Serial.println(
                "Controller reports resumed - reinitializing before control is allowed."
            );
        }
    }


    // The Bluetooth stack can take time to announce a lost link.  Treat
    // stale reports as a safety fault even if isConnected() is still true.
    if (
        controller &&
        controller->isConnected() &&
        millis() -
        lastControllerReport >
        CONTROLLER_TIMEOUT_MS
    ) {

        if (
            !controllerTimeoutActive
        ) {

            controllerTimeoutActive =
                true;

            bool wasArmed =
                motorsArmed;

            motorsArmed =
                false;

            stopMotorsImmediate();

            controllerState = CONTROLLER_INITIALIZING;

            controllerNeutralSince = 0;

            armButtonReleased = false;

            previousOptions = true;

            // R4: clear the Wi-Fi combo so the status does not keep
            // flashing the hold color.
            wifiComboActive = false;
            wifiComboTriggered = false;
            wifiComboStart = 0;

            controllerStaleStop = false;
            driveNeedsNeutral = false;
            optionsPressValid = false;
            triangleDuringOptions = false;

            Serial.println(
                wasArmed
                    ? "Controller data timeout - motors stopped; vehicle DISARMED."
                    : "Controller data timeout - waiting for reports to resume."
            );
        }
    }


    serviceControllerState();

    serviceStaleStop();


    // --------------------------------------------------------------
    // Battery monitoring
    // --------------------------------------------------------------

    serviceBatteryMonitor();


    // --------------------------------------------------------------
    // Controller
    // --------------------------------------------------------------

    if (
        controller &&
        controller->isConnected() &&
        controller->isGamepad() &&
        controllerState == CONTROLLER_READY &&
        !controllerTimeoutActive &&
        millis() - lastControllerReport <= CONTROLLER_TIMEOUT_MS
    ) {

        processButtons();

        if ( controllerStaleStop ) {
            requestedLeftMotor = 0;
            requestedRightMotor = 0;
        } else {
            processDrive();
        }

    } else {

        requestedLeftMotor =
            0;

        requestedRightMotor =
            0;
    }


    // --------------------------------------------------------------
    // Motor output
    // --------------------------------------------------------------

    serviceMotorRamp();


    // --------------------------------------------------------------
    // Wi-Fi / web server
    // --------------------------------------------------------------

    serviceConfigWiFi();


    // --------------------------------------------------------------
    // Rumble patterns
    // --------------------------------------------------------------

    serviceRumble();


    // --------------------------------------------------------------
    // Status LEDs / controller lightbar
    // --------------------------------------------------------------

    updateStatusIndicators();


    // --------------------------------------------------------------
    // USB diagnostics
    // --------------------------------------------------------------
    //
    // Normal output is event-driven.  This optional diagnostic stream
    // is disabled by default.

    printVerboseControllerDebug();


    delay(
        3
    );
}
