/*
 ======================================================================
 ESP RC TANK CONTROLLER
 Firmware 0.4

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

 LEFT STICK Y    Left track
 RIGHT STICK Y   Right track

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

#define FIRMWARE_VERSION "0.4"


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
// Battery + -> 150K -> ADC -> 33K -> Ground
//
const float BAT_R_TOP = 150000.0;
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
    "GraveDig";

const char* AP_PASSWORD =
    "GraveDig32";

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

    // Drive
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

    // 0 = configuration mode can be started anytime
    int wifiAccessWindowSeconds;

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

bool previousOptions = false;
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


// ======================================================================
// DEFAULT SETTINGS
// ======================================================================

void setFactoryDefaults() {

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

    // Default:
    // Wi-Fi may only be STARTED during
    // first 60 seconds after boot.
    //
    // 0 = anytime.
    settings.wifiAccessWindowSeconds =
        60;

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

    settings.wifiAccessWindowSeconds =
        prefs.getInt(
            "wifiWin",
            settings.wifiAccessWindowSeconds
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


    prefs.end();


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

    settings.wifiAccessWindowSeconds =
        constrain(
            settings.wifiAccessWindowSeconds,
            0,
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
        "wifiWin",
        settings.wifiAccessWindowSeconds
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
        selectedSpeedPercent();


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

int stickToMotor(
    int rawValue
) {

    const int stickMax =
        512;


    rawValue =
        constrain(
            rawValue,
            -stickMax,
            stickMax
        );


    int magnitude =
        abs(rawValue);


    if (
        magnitude <=
        settings.stickDeadzone
    ) {
        return 0;
    }


    float normalized =
        (
            float(
                magnitude -
                settings.stickDeadzone
            )
        ) /
        (
            float(
                stickMax -
                settings.stickDeadzone
            )
        );


    normalized =
        constrain(
            normalized,
            0.0f,
            1.0f
        );


    normalized =
        applyResponseCurve(
            normalized
        );


    int maxPWM =
        round(
            effectiveSpeedPercent() *
            2.55f
        );


    int result =
        round(
            normalized *
            maxPWM
        );


    if (rawValue < 0) {
        result =
            -result;
    }


    return result;
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
        batteryState ==
            BATTERY_CRITICAL &&
        settings.criticalBehavior == 2
    ) {
        driveAllowed = false;
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

void processDrive() {

    if (
        !controller ||
        !controller->isConnected() ||
        !motorsArmed ||
        emergencyAbort ||
        wifiConfigActive
    ) {

        requestedLeftMotor = 0;
        requestedRightMotor = 0;

        return;
    }


    int leftRaw =
        -controller->axisY();

    int rightRaw =
        -controller->axisRY();


    int left =
        stickToMotor(
            leftRaw
        );

    int right =
        stickToMotor(
            rightRaw
        );


    // Motor trim

    left =
        round(
            left *
            settings.leftTrimPercent /
            100.0f
        );

    right =
        round(
            right *
            settings.rightTrimPercent /
            100.0f
        );


    left =
        constrain(
            left,
            -255,
            255
        );

    right =
        constrain(
            right,
            -255,
            255
        );


    // Software reversal

    if (
        settings.invertLeft
    ) {
        left =
            -left;
    }


    if (
        settings.invertRight
    ) {
        right =
            -right;
    }


    requestedLeftMotor =
        left;

    requestedRightMotor =
        right;
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

        batteryFilterInitialized =
            false;

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

        batteryFilterInitialized =
            false;

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


    // Initialize immediately
    if (
        batteryState ==
        BATTERY_UNKNOWN
    ) {

        batteryState =
            desired;

        batteryCandidate =
            desired;

        batteryCandidateSince =
            now;

        return;
    }


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

    html.reserve(12000);


    html += R"HTML(
<!DOCTYPE html>
<html>

<head>

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>GraveDig</title>

<style>

body {
    font-family: Arial, sans-serif;
    margin: 0;
    background: #101010;
    color: #eeeeee;
}

.container {
    max-width: 760px;
    margin: auto;
    padding: 18px;
}

.card {
    background: #222;
    border-radius: 10px;
    padding: 18px;
    margin-bottom: 16px;
}

h1 {
    margin-bottom: 4px;
}

h2 {
    margin-top: 0;
    font-size: 19px;
}

label {
    display: block;
    margin-top: 14px;
}

input,
select {
    width: 100%;
    padding: 9px;
    margin-top: 5px;
    box-sizing: border-box;
    background: #333;
    color: white;
    border: 1px solid #555;
    border-radius: 4px;
}

input[type=checkbox] {
    width: auto;
}

button {
    border: 0;
    border-radius: 6px;
    padding: 12px 18px;
    margin-top: 12px;
    font-size: 16px;
}

.save {
    background: #238636;
    color: white;
}

.warning {
    background: #9a6700;
    color: white;
}

.danger {
    background: #b62324;
    color: white;
}

.status {
    font-family: monospace;
    line-height: 1.65;
}

.small {
    font-size: 13px;
    opacity: .75;
}

.grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
}

@media(max-width:600px) {
    .grid {
        grid-template-columns: 1fr;
    }
}

</style>

</head>

<body>

<div class="container">

<h1>GraveDig</h1>
<div class="small">
Firmware )HTML";


    html +=
        FIRMWARE_VERSION;


    html += R"HTML(
</div>


<div class="card">

<h2>Live Status</h2>

<div
class="status"
id="status">
Loading...
</div>

</div>


<form method="POST"
action="/save">


<div class="card">

<h2>Drive</h2>


<div class="grid">

<label>
Low-speed limit (%)
<input
type="number"
name="lowSpeed"
min="5"
max="100"
value=")HTML";

    html +=
        String(
            settings.lowSpeedPercent
        );


    html += R"HTML(">
</label>


<label>
Normal-speed limit (%)
<input
type="number"
name="normalSpeed"
min="5"
max="100"
value=")HTML";

    html +=
        String(
            settings.normalSpeedPercent
        );


    html += R"HTML(">
</label>


<label>
Stick dead zone
<input
type="number"
name="deadzone"
min="0"
max="200"
value=")HTML";

    html +=
        String(
            settings.stickDeadzone
        );


    html += R"HTML(">
</label>


<label>
Response curve

<select name="curve">

<option value="0")HTML";

    if (
        settings.responseCurve ==
        0
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Linear
</option>

<option value="1")HTML";

    if (
        settings.responseCurve ==
        1
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Soft
</option>

<option value="2")HTML";

    if (
        settings.responseCurve ==
        2
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Medium
</option>

<option value="3")HTML";

    if (
        settings.responseCurve ==
        3
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Aggressive
</option>

</select>

</label>


<label>
Acceleration time (ms)
<input
type="number"
name="accel"
min="0"
max="5000"
value=")HTML";

    html +=
        String(
            settings.accelerationMs
        );


    html += R"HTML(">
</label>


<label>
Deceleration time (ms)
<input
type="number"
name="decel"
min="0"
max="5000"
value=")HTML";

    html +=
        String(
            settings.decelerationMs
        );


    html += R"HTML(">
</label>


<label>
Left motor trim (%)
<input
type="number"
name="trimL"
min="50"
max="120"
value=")HTML";

    html +=
        String(
            settings.leftTrimPercent
        );


    html += R"HTML(">
</label>


<label>
Right motor trim (%)
<input
type="number"
name="trimR"
min="50"
max="120"
value=")HTML";

    html +=
        String(
            settings.rightTrimPercent
        );


    html += R"HTML(">
</label>

</div>


<label>

<input
type="checkbox"
name="invertLeft")HTML";

    if (
        settings.invertLeft
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Reverse left motor

</label>


<label>

<input
type="checkbox"
name="invertRight")HTML";

    if (
        settings.invertRight
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Reverse right motor

</label>

</div>


<div class="card">

<h2>Lighting</h2>

<label>
Master pixel brightness
<input
type="number"
name="brightness"
min="1"
max="255"
value=")HTML";

    html +=
        String(
            settings.pixelBrightness
        );


    html += R"HTML(">
</label>

<p class="small">
Pixel 0 is reserved for system status.
Pixels 1 and above remain available for lighting effects.
</p>

</div>


<div class="card">

<h2>Battery Monitoring</h2>


<label>

<input
type="checkbox"
name="batteryEnabled")HTML";

    if (
        settings.batteryEnabled
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Enable vehicle battery monitoring

</label>


<div class="grid">


<label>
Low warning voltage
<input
type="number"
step="0.01"
name="lowV"
value=")HTML";

    html +=
        String(
            settings.lowBatteryVoltage,
            2
        );


    html += R"HTML(">
</label>


<label>
Critical voltage
<input
type="number"
step="0.01"
name="criticalV"
value=")HTML";

    html +=
        String(
            settings.criticalBatteryVoltage,
            2
        );


    html += R"HTML(">
</label>


<label>
Recovery hysteresis
<input
type="number"
step="0.01"
name="hysteresis"
value=")HTML";

    html +=
        String(
            settings.batteryHysteresis,
            2
        );


    html += R"HTML(">
</label>


<label>
ADC calibration multiplier
<input
type="number"
step="0.001"
name="batteryCal"
value=")HTML";

    html +=
        String(
            settings.batteryCalibration,
            3
        );


    html += R"HTML(">
</label>


<label>
Threshold confirmation time (sec)
<input
type="number"
min="1"
max="15"
name="batteryDelay"
value=")HTML";

    html +=
        String(
            settings.batteryConfirmSeconds
        );


    html += R"HTML(">
</label>


<label>
Critical power limit (%)
<input
type="number"
min="10"
max="100"
name="criticalLimit"
value=")HTML";

    html +=
        String(
            settings.criticalPowerLimitPercent
        );


    html += R"HTML(">
</label>

</div>


<label>
Critical battery action

<select name="criticalBehavior">

<option value="0")HTML";

    if (
        settings.criticalBehavior ==
        0
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Warn only
</option>

<option value="1")HTML";

    if (
        settings.criticalBehavior ==
        1
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Limit motor power
</option>

<option value="2")HTML";

    if (
        settings.criticalBehavior ==
        2
    ) {
        html +=
            " selected";
    }


    html += R"HTML(>
Disable drive
</option>

</select>

</label>


<label>

<input
type="checkbox"
name="batteryPixel")HTML";

    if (
        settings.batteryPixelWarning
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Battery warning on status pixel

</label>


<label>

<input
type="checkbox"
name="batteryLight")HTML";

    if (
        settings.batteryControllerLight
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Battery warning on DualSense lightbar

</label>


<label>

<input
type="checkbox"
name="batteryRumble")HTML";

    if (
        settings.batteryRumble
    ) {
        html +=
            " checked";
    }


    html += R"HTML(>

Battery warning using DualSense rumble

</label>


</div>


<div class="card">

<h2>Configuration Access</h2>


<div class="grid">


<label>
Wi-Fi idle timeout (sec)
<input
type="number"
name="wifiTimeout"
min="30"
max="3600"
value=")HTML";

    html +=
        String(
            settings.wifiTimeoutSeconds
        );


    html += R"HTML(">
</label>


<label>
Wi-Fi startup access window (sec)
<input
type="number"
name="wifiWindow"
min="0"
max="3600"
value=")HTML";

    html +=
        String(
            settings.wifiAccessWindowSeconds
        );


    html += R"HTML(">
</label>


<label>
OPTIONS + Triangle hold (sec)
<input
type="number"
name="wifiHold"
min="2"
max="10"
value=")HTML";

    html +=
        String(
            settings.wifiHoldSeconds
        );


    html += R"HTML(">
</label>


</div>


<p class="small">

Set startup access window to 0 to allow configuration mode
to be entered anytime while the vehicle is SAFE.

</p>

</div>


<div class="card">

<button
class="save"
type="submit">
Save Settings
</button>

</form>


<form
method="POST"
action="/defaults">

<button
class="warning"
type="submit">
Restore Factory Defaults
</button>

</form>


<form
method="POST"
action="/wifi-off">

<button
class="danger"
type="submit">
Shut Down Wi-Fi
</button>

</form>

</div>


</div>


<script>

function updateStatus() {

fetch('/status')

.then(
response =>
response.json()
)

.then(data => {

let s = '';

s +=
'Controller: ' +
data.controller +
'<br>';

s +=
'Controller Battery: ' +
data.controllerBattery +
'<br>';

s +=
'Drive: ' +
data.drive +
'<br>';

s +=
'Speed Profile: ' +
data.profile +
'<br>';

s +=
'Left Motor: ' +
data.left +
'%<br>';

s +=
'Right Motor: ' +
data.right +
'%<br>';

s +=
'Vehicle Battery: ' +
data.vehicleVoltage +
'<br>';

s +=
'Battery State: ' +
data.batteryState +
'<br>';

s +=
'Wi-Fi Clients: ' +
data.clients +
'<br>';

s +=
'Uptime: ' +
data.uptime +
'<br>';

document.getElementById(
'status'
).innerHTML = s;

})

.catch(
() => {}
);

}


updateStatus();

setInterval(
updateStatus,
2000
);

</script>

</body>

</html>
)HTML";


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

void handleSave() {

    noteWebActivity();


    if (
        server.hasArg(
            "lowSpeed"
        )
    ) {

        settings.lowSpeedPercent =
            constrain(
                server.arg(
                    "lowSpeed"
                ).toInt(),
                5,
                100
            );
    }


    if (
        server.hasArg(
            "normalSpeed"
        )
    ) {

        settings.normalSpeedPercent =
            constrain(
                server.arg(
                    "normalSpeed"
                ).toInt(),
                5,
                100
            );
    }


    if (
        server.hasArg(
            "deadzone"
        )
    ) {

        settings.stickDeadzone =
            constrain(
                server.arg(
                    "deadzone"
                ).toInt(),
                0,
                200
            );
    }


    if (
        server.hasArg(
            "curve"
        )
    ) {

        settings.responseCurve =
            constrain(
                server.arg(
                    "curve"
                ).toInt(),
                0,
                3
            );
    }


    if (
        server.hasArg(
            "accel"
        )
    ) {

        settings.accelerationMs =
            constrain(
                server.arg(
                    "accel"
                ).toInt(),
                0,
                5000
            );
    }


    if (
        server.hasArg(
            "decel"
        )
    ) {

        settings.decelerationMs =
            constrain(
                server.arg(
                    "decel"
                ).toInt(),
                0,
                5000
            );
    }


    if (
        server.hasArg(
            "trimL"
        )
    ) {

        settings.leftTrimPercent =
            constrain(
                server.arg(
                    "trimL"
                ).toInt(),
                50,
                120
            );
    }


    if (
        server.hasArg(
            "trimR"
        )
    ) {

        settings.rightTrimPercent =
            constrain(
                server.arg(
                    "trimR"
                ).toInt(),
                50,
                120
            );
    }


    settings.invertLeft =
        server.hasArg(
            "invertLeft"
        );

    settings.invertRight =
        server.hasArg(
            "invertRight"
        );


    if (
        server.hasArg(
            "brightness"
        )
    ) {

        settings.pixelBrightness =
            constrain(
                server.arg(
                    "brightness"
                ).toInt(),
                1,
                255
            );
    }


    // Battery

    settings.batteryEnabled =
        server.hasArg(
            "batteryEnabled"
        );


    if (
        server.hasArg(
            "lowV"
        )
    ) {

        settings.lowBatteryVoltage =
            server.arg(
                "lowV"
            ).toFloat();
    }


    if (
        server.hasArg(
            "criticalV"
        )
    ) {

        settings.criticalBatteryVoltage =
            server.arg(
                "criticalV"
            ).toFloat();
    }


    if (
        server.hasArg(
            "hysteresis"
        )
    ) {

        settings.batteryHysteresis =
            server.arg(
                "hysteresis"
            ).toFloat();
    }


    if (
        server.hasArg(
            "batteryCal"
        )
    ) {

        settings.batteryCalibration =
            server.arg(
                "batteryCal"
            ).toFloat();
    }


    if (
        server.hasArg(
            "batteryDelay"
        )
    ) {

        settings.batteryConfirmSeconds =
            constrain(
                server.arg(
                    "batteryDelay"
                ).toInt(),
                1,
                15
            );
    }


    settings.batteryPixelWarning =
        server.hasArg(
            "batteryPixel"
        );

    settings.batteryControllerLight =
        server.hasArg(
            "batteryLight"
        );

    settings.batteryRumble =
        server.hasArg(
            "batteryRumble"
        );


    if (
        server.hasArg(
            "criticalBehavior"
        )
    ) {

        settings.criticalBehavior =
            constrain(
                server.arg(
                    "criticalBehavior"
                ).toInt(),
                0,
                2
            );
    }


    if (
        server.hasArg(
            "criticalLimit"
        )
    ) {

        settings.criticalPowerLimitPercent =
            constrain(
                server.arg(
                    "criticalLimit"
                ).toInt(),
                10,
                100
            );
    }


    // Wi-Fi

    if (
        server.hasArg(
            "wifiTimeout"
        )
    ) {

        settings.wifiTimeoutSeconds =
            constrain(
                server.arg(
                    "wifiTimeout"
                ).toInt(),
                30,
                3600
            );
    }


    if (
        server.hasArg(
            "wifiWindow"
        )
    ) {

        settings.wifiAccessWindowSeconds =
            constrain(
                server.arg(
                    "wifiWindow"
                ).toInt(),
                0,
                3600
            );
    }


    if (
        server.hasArg(
            "wifiHold"
        )
    ) {

        settings.wifiHoldSeconds =
            constrain(
                server.arg(
                    "wifiHold"
                ).toInt(),
                2,
                10
            );
    }


    saveSettings();


    server.sendHeader(
        "Location",
        "/"
    );

    server.send(
        303,
        "text/plain",
        "Saved"
    );
}


// ======================================================================
// RESTORE DEFAULTS
// ======================================================================

void handleDefaults() {

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

<h2>GraveDig</h2>

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

void configureWebRoutes() {

    static bool configured =
        false;


    if (configured) {
        return;
    }


    configured =
        true;


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

    if (!controller) {
        return false;
    }


    int margin =
        settings.stickDeadzone +
        30;


    return
        abs(
            controller->axisY()
        ) <
        margin
        &&
        abs(
            controller->axisRY()
        ) <
        margin;
}


// ======================================================================
// CONFIG ACCESS WINDOW
// ======================================================================

bool wifiAccessWindowOpen() {

    if (
        settings.wifiAccessWindowSeconds ==
        0
    ) {
        return true;
    }


    return
        millis() <=
        settings.wifiAccessWindowSeconds *
        1000UL;
}


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


    // Startup access window only affects ENTRY,
    // never shutdown.

    if (
        !wifiConfigActive &&
        !wifiAccessWindowOpen()
    ) {

        if (
            !wifiComboTriggered
        ) {

            wifiComboTriggered =
                true;

            Serial.println(
                "Wi-Fi configuration access window closed."
            );


            queueRumble(
                2,
                100,
                80,
                40,
                40
            );
        }


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


    controller =
        ctl;


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


    controller =
        nullptr;


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
                batteryState ==
                    BATTERY_CRITICAL &&
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


    previousOptions =
        options;


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
        "GraveDig RC Tank"
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


    // Motors

    setupPWM();

    stopMotorsImmediate();


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


    // --------------------------------------------------------------
    // Bluepad32
    // --------------------------------------------------------------

    BP32.update();


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
        controller->isGamepad()
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
