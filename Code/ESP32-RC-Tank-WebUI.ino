/*
 ======================================================================
 ESP RC TANK CONTROLLER
 Firmware 0.5

 ESP32-WROOM-32 / ESP32-WROOM-32E
 Cytron MDD3A
 PS5 DualSense via Bluepad32
 WS2811 / WS2812 status & lighting
 Wi-Fi SoftAP configuration
 Vehicle battery monitoring

 ----------------------------------------------------------------------
 PIN ASSIGNMENTS
 ----------------------------------------------------------------------

 GPIO 17  -> MDD3A M1A
 GPIO 18  -> MDD3A M1B

 GPIO 22  -> MDD3A M2A
 GPIO 23  -> MDD3A M2B

 GPIO 25  -> WS2811 / WS2812 data
             Pixel 0 reserved for SYSTEM STATUS

 GPIO 26  -> Reserved future Servo 1
 GPIO 27  -> Reserved future Servo 2

 GPIO 34  -> Vehicle battery ADC

 GPIO 0   -> Emergency abort button on development board

 ----------------------------------------------------------------------
 PS5 CONTROLS
 ----------------------------------------------------------------------

 LEFT STICK Y    Tank: left track / Arcade: both tracks throttle
 RIGHT STICK Y   Tank: right track
 RIGHT STICK X   Arcade: steering

 OPTIONS         Arm / Disarm

 L1              Low-speed profile
 R1              Normal-speed profile

 OPTIONS +
 TRIANGLE
 held 3 seconds  Toggle configuration Wi-Fi

 ----------------------------------------------------------------------
 WI-FI
 ----------------------------------------------------------------------

 SSID:       ESPRC
 Password:   ESPRC123
 Address:    http://192.168.4.1

 ----------------------------------------------------------------------
 STATUS COLORS
 ----------------------------------------------------------------------

 Blue pulse      Waiting for PS5 controller
 Blue            Connected / SAFE
 Green           Motors ARMED
 Yellow          Configuration Wi-Fi
 Flash yellow    Wi-Fi activation hold in progress
 Orange          LOW vehicle battery
 Red             CRITICAL vehicle battery
 Flash red       Emergency abort

 Battery status overrides normal operating colors.

 ======================================================================
*/

#include <Arduino.h>
#include <Bluepad32.h>

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include <Adafruit_NeoPixel.h>

#include <esp_arduino_version.h>

#include <math.h>


// ======================================================================
// VERSION
// ======================================================================

#define FIRMWARE_VERSION "0.5"


// ======================================================================
// HARDWARE
// ======================================================================

// MDD3A
#define M1A_PIN 17
#define M1B_PIN 18
#define M2A_PIN 22
#define M2B_PIN 23

// WS281x
#define PIXEL_PIN 25
#define PIXEL_COUNT 8

Adafruit_NeoPixel pixels(
    PIXEL_COUNT,
    PIXEL_PIN,
    NEO_GRB + NEO_KHZ800
);

// Vehicle battery
#define BATTERY_ADC_PIN 34

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

#define PWM_FREQ 10000
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
bool wifiShutdownRequested = false;

unsigned long wifiLastActivity = 0;
unsigned long wifiShutdownRequestTime = 0;


// ======================================================================
// SETTINGS
// ======================================================================

Preferences prefs;


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

bool motorsArmed = false;
bool emergencyAbort = false;


// ======================================================================
// BUTTON EDGE / HOLD STATE
// ======================================================================

bool previousOptions = true;
bool armButtonReleased = false;
unsigned long lastControllerReport = 0;
const unsigned long CONTROLLER_TIMEOUT_MS = 1000;
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
unsigned long lastSerialTelemetry = 0;


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

bool validSettings(const TankSettings& v) {
    return
        v.lowSpeedPercent >= 5 && v.lowSpeedPercent <= 100 &&
        v.normalSpeedPercent >= 5 && v.normalSpeedPercent <= 100 &&
        v.driveMode >= 0 && v.driveMode <= 1 &&
        v.steeringMode >= 0 && v.steeringMode <= 1 &&
        v.steeringSensitivity >= 0 && v.steeringSensitivity <= 200 &&
        v.maxOutputPercent >= 5 && v.maxOutputPercent <= 100 &&
        v.stickDeadzone >= 0 && v.stickDeadzone <= 200 &&
        v.responseCurve >= 0 && v.responseCurve <= 3 &&
        v.accelerationMs >= 0 && v.accelerationMs <= 5000 &&
        v.decelerationMs >= 0 && v.decelerationMs <= 5000 &&
        v.leftTrimPercent >= 50 && v.leftTrimPercent <= 120 &&
        v.rightTrimPercent >= 50 && v.rightTrimPercent <= 120 &&
        v.pixelBrightness >= 1 && v.pixelBrightness <= 255 &&
        isfinite(v.lowBatteryVoltage) && v.lowBatteryVoltage >= 6.0 && v.lowBatteryVoltage <= 8.4 &&
        isfinite(v.criticalBatteryVoltage) && v.criticalBatteryVoltage >= 6.0 && v.criticalBatteryVoltage <= 8.3 &&
        isfinite(v.batteryHysteresis) && v.batteryHysteresis >= 0.01 && v.batteryHysteresis <= 0.5 &&
        isfinite(v.batteryCalibration) && v.batteryCalibration >= 0.5 && v.batteryCalibration <= 1.5 &&
        v.batteryConfirmSeconds >= 1 && v.batteryConfirmSeconds <= 15 &&
        v.criticalBehavior >= 0 && v.criticalBehavior <= 2 &&
        v.criticalPowerLimitPercent >= 10 && v.criticalPowerLimitPercent <= 100 &&
        v.wifiTimeoutSeconds >= 30 && v.wifiTimeoutSeconds <= 3600 &&
        v.wifiHoldSeconds >= 2 && v.wifiHoldSeconds <= 10 &&
        v.criticalBatteryVoltage < v.lowBatteryVoltage;
}

void loadSettings() {

    setFactoryDefaults();

    prefs.begin(
        "gravedig",
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
        Serial.println("Invalid saved settings: restoring safe defaults, battery protection enabled.");
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
        "gravedig",
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
// CURRENT SPEED LIMIT
// ======================================================================

int selectedSpeedPercent() {

    return lowSpeedProfile
        ? settings.lowSpeedPercent
        : settings.normalSpeedPercent;
}


int effectiveSpeedPercent() {

    int percent =
        min(selectedSpeedPercent(), settings.maxOutputPercent);


    if (
        settings.batteryEnabled &&
        batteryState ==
            BATTERY_CRITICAL &&
        settings.criticalBehavior == 1
    ) {

        percent =
            min(
                percent,
                settings.criticalPowerLimitPercent
            );
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

    unsigned long now =
        millis();


    unsigned long dt =
        now -
        lastRampUpdate;


    if (dt < 10) {
        return;
    }


    lastRampUpdate =
        now;


    // Avoid giant step after debugging pause.
    if (dt > 50) {
        dt = 50;
    }


    bool driveAllowed =
        controller &&
        controller->isConnected() &&
        motorsArmed &&
        !emergencyAbort &&
        !wifiConfigActive;


    if (
        settings.batteryEnabled &&
        (batteryState == BATTERY_CRITICAL || batteryState == BATTERY_UNKNOWN) &&
        settings.criticalBehavior == 2
    ) {
        driveAllowed = false;
        motorsArmed = false;
    }


    if (!driveAllowed) {

        stopMotorsImmediate();

        return;
    }


    appliedLeftMotor =
        rampToward(
            appliedLeftMotor,
            requestedLeftMotor,
            dt
        );


    appliedRightMotor =
        rampToward(
            appliedRightMotor,
            requestedRightMotor,
            dt
        );


    int limit = round(effectiveSpeedPercent() * 2.55f);
    appliedLeftMotor = constrain(appliedLeftMotor, -float(limit), float(limit));
    appliedRightMotor = constrain(appliedRightMotor, -float(limit), float(limit));
    leftMotorCommand =
        round(
            appliedLeftMotor
        );

    rightMotorCommand =
        round(
            appliedRightMotor
        );


    setMotor1(
        leftMotorCommand
    );

    setMotor2(
        rightMotorCommand
    );
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
    if (!controller || !controller->isConnected() || !motorsArmed ||
        emergencyAbort || wifiConfigActive) {
        requestedLeftMotor = requestedRightMotor = 0;
        return;
    }
    float throttle = normalizedStick(-controller->axisY());
    float second = normalizedStick(settings.driveMode == 1 ?
        controller->axisRX() : -controller->axisRY());
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


    if (
        now <
        rumblePattern.nextPulse
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

void batteryStateChanged(
    BatteryState newState
) {

    batteryState =
        newState;


    Serial.print(
        "Battery state: "
    );

    Serial.println(
        batteryStateName(
            newState
        )
    );


    if (
        newState ==
        BATTERY_LOW
    ) {

        if (
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


        lastBatteryReminder =
            millis();
    }


    if (
        newState ==
        BATTERY_CRITICAL
    ) {

        if (
            settings.criticalBehavior ==
            2
        ) {

            motorsArmed =
                false;

            stopMotorsImmediate();
        }


        if (
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


        lastBatteryReminder =
            millis();
    }
}


// ======================================================================
// BATTERY MONITOR
// ======================================================================

void serviceBatteryMonitor() {

    if (
        !settings.batteryEnabled
    ) {

        batteryState =
            BATTERY_UNKNOWN;

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


    // Treat very low voltage as "not connected".
    if (measured < 0.50f) {

        batteryState =
            BATTERY_UNKNOWN;

        batteryFilterInitialized = false;
        batteryCandidate = BATTERY_UNKNOWN;
        batteryCandidateSince = millis();
        batteryVoltage = batteryFilteredVoltage = 0;
        return;
    }


    batteryVoltage =
        measured;


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

RGBColor getStatusColor(
    bool allowBatteryWarning
) {

    RGBColor color;


    // Emergency
    if (emergencyAbort) {

        bool on =
            (
                millis() /
                200
            ) %
            2;

        color.r =
            on ? 255 : 0;

        color.g = 0;
        color.b = 0;

        return color;
    }


    // Battery warnings

    if (
        allowBatteryWarning &&
        settings.batteryEnabled
    ) {

        if (
            batteryState ==
            BATTERY_CRITICAL
        ) {

            color.r = 255;
            color.g = 0;
            color.b = 0;

            return color;
        }


        if (
            batteryState ==
            BATTERY_LOW
        ) {

            color.r = 255;
            color.g = 75;
            color.b = 0;

            return color;
        }
    }


    // Wi-Fi combo hold

    if (
        wifiComboActive &&
        !wifiComboTriggered
    ) {

        bool on =
            (
                millis() /
                180
            ) %
            2;

        color.r =
            on ? 255 : 30;

        color.g =
            on ? 140 : 10;

        color.b = 0;

        return color;
    }


    // Wi-Fi active

    if (
        wifiConfigActive
    ) {

        color.r = 255;
        color.g = 150;
        color.b = 0;

        return color;
    }


    // Waiting for controller

    if (
        !controller ||
        !controller->isConnected()
    ) {

        int phase =
            (
                millis() /
                15
            ) %
            200;

        if (phase > 100) {
            phase =
                200 -
                phase;
        }


        color.r = 0;
        color.g = 0;

        color.b =
            map(
                phase,
                0,
                100,
                5,
                150
            );

        return color;
    }


    // Armed

    if (
        motorsArmed
    ) {

        color.r = 0;
        color.g = 200;
        color.b = 0;

        return color;
    }


    // Safe

    color.r = 0;
    color.g = 0;
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

    if (emergencyAbort) {
        return;
    }


    if (
        digitalRead(
            ABORT_BUTTON
        ) ==
        LOW
    ) {

        delay(20);


        if (
            digitalRead(
                ABORT_BUTTON
            ) ==
            LOW
        ) {

            emergencyStop();
        }
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

String makeWebPage() {
    String html;
    html.reserve(15000);
    html += R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ESPRC</title>
<style>body{font-family:Arial,sans-serif;background:#101820;color:#eee;margin:0}main{max-width:780px;margin:auto;padding:20px}.card{background:#202d37;padding:20px;border-radius:12px;margin:16px 0}label{display:block;margin:14px 0}input,select,button{font:inherit;padding:9px;border-radius:5px}input:not([type=checkbox]),select{display:block;box-sizing:border-box;width:100%;margin-top:5px}button{cursor:pointer;background:#c5e88a;color:#182119;border:0}svg{width:100%;height:auto}p{line-height:1.5}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}@media(max-width:600px){.grid{grid-template-columns:1fr}}#status{white-space:pre-line;line-height:1.6}</style>
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
    html += R"HTML(</section><section class="card"><h2>Battery Configuration</h2><p>Battery: 2S LiPo (8.4 V full). GPIO34: 100 kOhm from battery + to ADC, 33 kOhm from ADC to ground; optional 100 nF to ground. Monitoring is disabled until enabled below.</p>)HTML";
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
    html += R"HTML(</section><section class="card"><h2>Wi-Fi Configuration</h2><p>Hold OPTIONS + Triangle with drive disarmed and sticks neutral at any time. Motors stay disabled throughout configuration. Release buttons and center sticks before manually rearming. Wi-Fi: ESPRC / ESPRC123. Close this page to start the inactivity timeout.</p>)HTML";
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
<form method="post" action="/wifi-off"><button>Shut Down Wi-Fi</button></form></section>
<script>async function updateStatus(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error();const d=await r.json();document.getElementById('status').textContent='Controller: '+d.controller+' ('+d.controllerBattery+')\nDrive: '+d.drive+' / '+d.profile+'\nMotor output: L '+d.left+'%, R '+d.right+'%\nVehicle battery: '+d.vehicleVoltage+' / '+d.batteryState+'\nWi-Fi clients: '+d.clients+'\nUptime: '+d.uptime;}catch(e){document.getElementById('status').textContent='Connection lost. Reconnect to ESPRC Wi-Fi.';}}updateStatus();setInterval(updateStatus,2000);</script>
</main></body></html>)HTML";
    return html;
}


// ======================================================================
// WEB STATUS JSON
// ======================================================================

void handleStatus() {

    noteWebActivity();


    String json =
        "{";


    json +=
        "\"controller\":\"";

    json +=
        (
            controller &&
            controller->isConnected()
        )
        ? "CONNECTED"
        : "WAITING";

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
            "Disabled / Unknown";

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
        server.send(400, "text/plain", "Critical voltage must be below warning voltage; settings were not saved.");
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

    noteWebActivity();


    server.send(
        200,
        "text/html",
        R"HTML(
<html>

<body
style="
font-family:Arial;
background:#111;
color:white;
padding:30px;
">

<h2>ESPRC</h2>

<p>
Configuration Wi-Fi is shutting down.
</p>

<p>
The vehicle remains SAFE.
</p>

</body>

</html>
)HTML"
    );


    wifiShutdownRequested =
        true;

    wifiShutdownRequestTime =
        millis();
}


// ======================================================================
// WEB ROUTES
// ======================================================================

void resetControllerPairing() {
    if (motorsArmed) return;
    stopMotorsImmediate();
    armButtonReleased = false;
    previousOptions = true;
    if (controller) controller->disconnect();
    BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
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


    server.onNotFound(
        []() {

            noteWebActivity();

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
    Serial.println("Starting ESPRC configuration AP...");

    if (
        wifiConfigActive
    ) {
        return;
    }


    motorsArmed =
        false;

    stopMotorsImmediate();


    WiFi.mode(
        WIFI_AP
    );


    bool success =
        WiFi.softAP(
            AP_SSID,
            AP_PASSWORD
        );


    if (!success) {

        WiFi.mode(
            WIFI_OFF
        );

        Serial.println(
            "Wi-Fi AP start failed."
        );

        return;
    }


    configureWebRoutes();

    server.begin();


    wifiConfigActive =
        true;

    wifiShutdownRequested =
        false;

    wifiLastActivity =
        millis();


    Serial.println();
    Serial.println(
        "Configuration Wi-Fi ON"
    );

    Serial.print(
        "SSID: "
    );

    Serial.println(
        AP_SSID
    );

    Serial.print(
        "Address: "
    );

    Serial.println(
        WiFi.softAPIP()
    );


    queueRumble(
        2,
        120,
        100,
        60,
        60
    );
}


// ======================================================================
// STOP CONFIG WI-FI
// ======================================================================

void stopConfigWiFi() {
    armButtonReleased = false;
    previousOptions = true;

    if (
        !wifiConfigActive
    ) {
        return;
    }


    server.stop();


    WiFi.softAPdisconnect(
        true
    );

    WiFi.mode(
        WIFI_OFF
    );


    wifiConfigActive =
        false;

    wifiShutdownRequested =
        false;


    motorsArmed =
        false;

    stopMotorsImmediate();


    Serial.println(
        "Configuration Wi-Fi OFF"
    );


    queueRumble(
        1,
        160,
        0,
        50,
        50
    );
}


// ======================================================================
// SERVICE WEB SERVER
// ======================================================================

void serviceConfigWiFi() {

    if (
        !wifiConfigActive
    ) {
        return;
    }


    static int lastClients = -1;
    int clients = WiFi.softAPgetStationNum();
    if (clients != lastClients) {
        Serial.printf("Wi-Fi clients: %d (previous %d)\n", clients, lastClients);
        lastClients = clients;
    }
    server.handleClient();


    if (
        wifiShutdownRequested
    ) {

        if (
            millis() -
            wifiShutdownRequestTime >
            500
        ) {

            stopConfigWiFi();
        }

        return;
    }


    unsigned long timeout =
        settings.wifiTimeoutSeconds *
        1000UL;


    if (
        millis() -
        wifiLastActivity >
        timeout
    ) {

        Serial.println(
            "Wi-Fi idle timeout."
        );

        stopConfigWiFi();
    }
}


// ======================================================================
// CHECK STICK CENTERING
// ======================================================================

bool sticksCentered() {
    if (!controller || !controller->isConnected()) return false;
    return abs(controller->axisY()) <= settings.stickDeadzone &&
        abs(settings.driveMode == 1 ? controller->axisRX() : controller->axisRY()) <= settings.stickDeadzone;
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


    if (
        !wifiComboTriggered &&
        millis() -
        wifiComboStart >=
        required
    ) {

        wifiComboTriggered =
            true;


        if (
            wifiConfigActive
        ) {

            stopConfigWiFi();

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

    if (
        controller != nullptr
    ) {

        return;
    }


    if (!ctl->isGamepad()) return;
    controller = ctl;
    armButtonReleased = false;
    previousOptions = true;
    previousL1 = previousR1 = false;
    lastControllerReport = millis();


    motorsArmed =
        false;

    stopMotorsImmediate();


    lastControllerColor =
        0xFFFFFFFF;


    Serial.println();
    Serial.println(
        "DualSense connected."
    );


    queueRumble(
        1,
        150,
        0,
        60,
        40
    );


    // If battery was already low before
    // controller connected, notify now.

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


    rumblePattern.active =
        false;


    Serial.println(
        "DualSense disconnected - motors stopped."
    );
}


// ======================================================================
// BUTTON PROCESSING
// ======================================================================

void processButtons() {

    if (!controller) {
        return;
    }


    // Wi-Fi combo gets priority because it uses OPTIONS.
    processWiFiCombo();


    bool triangle =
        controller->y();


    // --------------------------------------------------------------
    // OPTIONS = Arm / Disarm
    //
    // Suppress normal OPTIONS action while Triangle is also held.
    // --------------------------------------------------------------

    bool options =
        controller->miscStart();


    if (
        options &&
        !previousOptions &&
        armButtonReleased &&
        !triangle
    ) {

        if (
            !emergencyAbort &&
            !wifiConfigActive
        ) {

            bool canArm =
                true;


            if (
                settings.batteryEnabled &&
                (batteryState == BATTERY_CRITICAL || batteryState == BATTERY_UNKNOWN) &&
                settings.criticalBehavior ==
                    2
            ) {

                canArm =
                    false;
            }


            if (
                !motorsArmed &&
                canArm
            ) {

                // Require sticks centered before arming.
                if (
                    sticksCentered()
                ) {

                    motorsArmed =
                        true;

                    stopMotorsImmediate();


                    queueRumble(
                        1,
                        120,
                        0,
                        60,
                        40
                    );

                } else {

                    queueRumble(
                        2,
                        80,
                        80,
                        30,
                        30
                    );
                }

            } else {

                motorsArmed =
                    false;

                stopMotorsImmediate();
            }
        }
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
    }


    previousR1 =
        r1;
}


// ======================================================================
// SERIAL TELEMETRY
// ======================================================================

void printTelemetry() {

    if (
        millis() -
        lastSerialTelemetry <
        1000
    ) {
        return;
    }


    lastSerialTelemetry =
        millis();


    Serial.printf(
        "BT:%s ARM:%s WIFI:%s "
        "L:%d R:%d "
        "BAT:%.2f %s\n",

        (
            controller &&
            controller->isConnected()
        )
            ? "OK"
            : "WAIT",

        motorsArmed
            ? "YES"
            : "NO",

        wifiConfigActive
            ? "ON"
            : "OFF",

        leftMotorCommand,

        rightMotorCommand,

        batteryFilteredVoltage,

        batteryStateName(
            batteryState
        )
    );
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

    Serial.println(
        FIRMWARE_VERSION
    );

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


    BP32.enableNewBluetoothConnections(true);
    BP32.enableVirtualDevice(false);
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
        "Waiting for DualSense..."
    );

    Serial.println(
        "OPTIONS = Arm / Disarm"
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

    bool dataUpdated = BP32.update();
    if (dataUpdated && controller && controller->hasData()) lastControllerReport = millis();
    // The Bluetooth stack may take seconds to report a lost radio link.
    if (controller && millis() - lastControllerReport > CONTROLLER_TIMEOUT_MS) {
        motorsArmed = false;
        stopMotorsImmediate();
        armButtonReleased = false;
        previousOptions = true;
    }


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
        millis() - lastControllerReport <= CONTROLLER_TIMEOUT_MS
    ) {

        processButtons();

        processDrive();

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

    printTelemetry();


    delay(
        3
    );
}
