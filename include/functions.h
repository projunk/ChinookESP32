#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <arduino.h>
#include <Wire.h>
#include <NonBlockingRtttl.h>
#include <NeoPixelBus.h>
#include <ArduinoUniqueID.h>
#include <ESP32Servo.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <driver\adc.h>
#include <TinyGPS++.h>
#include <mpu6050.h>
#include <..\..\MyCommon\Credentials.h>


#define eps                         0.00001

#define RACECAR_VERSION             "2.3c"

// CORE0 is used by WIFI
#define CORE0                       0
#define CORE1                       1

#define STACK_SIZE_CORE             4096

#define FORMAT_SPIFFS_IF_FAILED     true

#define LOOP_TIME_TASK1             40000    // microseconds; 25Hz
#define LOOP_TIME_TASK2             10000    // microseconds; 100Hz
#define LOOP_TIME_TASK3             4000     // microseconds; 250Hz
#define LOOP_TIME_TASK4             10000    // microseconds; 100Hz

#define NR_OF_RECEIVER_CHANNELS     3

#define SIGNAL_TIMEOUT              24000    // microseconds


#define MIN_GPS_SATELLITES          4
#define MIN_GPS_DISTANCE            50.0  // m
#define MIN_GPS_SPEED               8.0   // km/h

#define MID_CHANNEL              1500
#define DEADBAND_HALF            8
#define MIN_PULSE                1000
#define MAX_PULSE                2000

#define ESC_ARM_SIGNAL           1000
#define ESC_ARM_DELAY            2000

#define MAX_DELTA_SIGNAL          500
#define THRESHOLD_SIGNAL          250
#define HALF_SPEED_DELTA_SIGNAL   250
#define MIN_VALID_SIGNAL_VALUE    750
#define MAX_VALID_SIGNAL_VALUE    2250
#define SIGNALS_COUNT_THRESHOLD   20 

#define MIN_GYRO                  -6000
#define MAX_GYRO                  6000

#define MIN_SPEED                 -100
#define MAX_SPEED                 100

#define MIN_STEER                 -100
#define MAX_STEER                 100

#define POWER_OFF                 0

#define MIN_STEER_FACTOR          0.1

#define SPEEDOMETER_MAX_SPEED     100
#define SPEEDOMETER_SCALE         2.0

#define PWM_FREQUENCY_SERVO       50
#define PWM_RESOLUTION_SERVO      12
#define PWM_FREQUENCY_ESC        500
#define PWM_RESOLUTION_ESC        12

// channel 0 is in use by buzzer
#define MOTOR_STEERING_PWM_CHANNEL  3
#define MOTOR_LB_PWM_CHANNEL        4
#define MOTOR_RB_PWM_CHANNEL        5
#define MOTOR_LF_PWM_CHANNEL        6
#define MOTOR_RF_PWM_CHANNEL        7

#define BUZZER_PIN               23
#define MOTOR_STEERING_PIN       15
#define MOTOR_LB_PIN             14 // yellow
#define MOTOR_RB_PIN             27 // green
#define MOTOR_LF_PIN             26 // white
#define MOTOR_RF_PIN             25 // orange
#define RECEIVER_STEER_PIN        2
#define RECEIVER_THROTTLE_PIN     4
#define RECEIVER_SPEED_MODE_PIN   5   // using PIN 18 leads to crash in combination with double LED strip, PIN 5 works just fine
#define LED_PIN                  19
#define VOLTAGE_SENSOR_PIN       32
#define CURRENT_SENSOR_PIN       33


#define PIXEL_COLOR_FEATURE       NeoGrbFeature
#define PIXEL_T_METHOD            Neo800KbpsMethod
#define NR_OF_PIXELS_PER_LEDSTRIP 8

#define VOLTAGE_NOICE_FILTER      0.92
#define CURRENT_NOICE_FILTER      0.92

#define SSID_BASE                 "RACECAR_ESP32_"

#define NAME_TAB_SPEEDOMETER        "Speedo"
#define NAME_TAB_TELEMETRY          "Telemetry"
#define NAME_TAB_GPS                "GPS"
#define NAME_TAB_SETTINGS           "Settings"

#define NAME_TAB_BUTTON_SPEEDOMETER "BTN_" NAME_TAB_SPEEDOMETER
#define NAME_TAB_BUTTON_TELEMETRY   "BTN_" NAME_TAB_TELEMETRY
#define NAME_TAB_BUTTON_GPS         "BTN_" NAME_TAB_GPS
#define NAME_TAB_BUTTON_SETTINGS    "BTN_" NAME_TAB_SETTINGS 

#define NAME_SPEEDOMETER_GAUGE      "SpeedoMeterGauge"

#define NAME_MODEL                  "Model"
#define NAME_VERSION                "Version"
#define NAME_WIFI_SIGNAL_STRENGTH   "WiFi Signal Strength [dB]"
#define NAME_RESPONSE_TIME          "Response Time [ms]"
#define NAME_CHANNEL_1              "Channel 1 (Steer)"
#define NAME_CHANNEL_2              "Channel 2 (Speed)"
#define NAME_CHANNEL_3              "Channel 3 (Aux)"
#define NAME_SIGNAL_DETECTED        "Signal Detected"
#define NAME_ARMED                  "Armed"
#define NAME_DRIVING_MODE           "Driving Mode"
#define NAME_VOLTAGE                "Voltage [V]"
#define NAME_VOLTAGE_PROGRESS       "Voltage"
#define NAME_CURRENT                "Current [A]"
#define NAME_CURRENT_PROGRESS       "Current"
#define NAME_USED_UP_LOOPTIME_PROGRESS_1 "Used up Looptime task 1 [us]"
#define NAME_USED_UP_LOOPTIME_PROGRESS_2 "Used up Looptime task 2 [us]"
#define NAME_USED_UP_LOOPTIME_PROGRESS_3 "Used up Looptime task 3 [us]"
#define NAME_USED_UP_LOOPTIME_PROGRESS_4 "Used up Looptime task 4 [us]"
#define NAME_GYRO_X                 "gyro_x"
#define NAME_GYRO_Y                 "gyro_y"
#define NAME_GYRO_Z                 "gyro_z"
#define NAME_ACC_X                  "acc_x"
#define NAME_ACC_Y                  "acc_y"
#define NAME_ACC_Z                  "acc_z"
#define NAME_TEMPERATURE            "Temperature [C]"
#define NAME_ANGLE_ROLL_ACC         "angle_roll_acc"
#define NAME_ANGLE_PITCH_ACC        "angle_pitch_acc"
#define NAME_ANGLE_YAW_ACC          "angle_yaw_acc"
#define NAME_STEER_EXPO             "Steer Expo Factor"
#define NAME_STEER_SERVO_CENTER_OFFSET "Steer Servo Center Offset"
#define NAME_SPEED_ESC_CENTER_OFFSET "Speed Esc Center Offset"
#define NAME_VOLTAGE_CORRECTION     "Voltage Correction Factor"
#define NAME_CURRENT_CORRECTION     "Current Correction Factor"
#define NAME_CALIBRATED_ACCX        "Calibrated AccX"
#define NAME_CALIBRATED_ACCY        "Calibrated AccY"
#define NAME_CALIBRATED_ACCZ        "Calibrated AccZ"

#define NAME_PID_SETTINGS_YAW       "Yaw"

#define NAME_TELEMETRY_ROLL         "Roll"
#define NAME_TELEMETRY_PITCH        "Pitch"
#define NAME_TELEMETRY_YAW          "Yaw"
#define NAME_ANGLE_ROLL             "Angle Roll"
#define NAME_ANGLE_PITCH            "Angle Pitch"
#define NAME_ANGLE_YAW              "Angle Yaw"
#define NAME_ROLL_LEVEL_ADJUST      "Roll Level Adjust"
#define NAME_PITCH_LEVEL_ADJUST     "Pitch Level Adjust"
#define NAME_YAW_LEVEL_ADJUST       "Yaw Level Adjust"
#define NAME_GYRO_ROLL_INPUT        "Gyro Roll Input"
#define NAME_GYRO_PITCH_INPUT       "Gyro Pitch Input"
#define NAME_GYRO_YAW_INPUT         "Gyro Yaw Input"
#define NAME_PID_ROLL_SETPOINT      "PID Roll Setpoint"
#define NAME_PID_PITCH_SETPOINT     "PID Pitch Setpoint"
#define NAME_PID_YAW_SETPOINT       "PID Yaw Setpoint"

#define NAME_PID_OUTPUT_ROLL_ERROR  "PID Output Roll Error"
#define NAME_PID_OUTPUT_PITCH_ERROR "PID Output Pitch Error"
#define NAME_PID_OUTPUT_YAW_ERROR   "PID Output Yaw Error"
#define NAME_PID_OUTPUT_ROLL_P      "PID Output Roll P"
#define NAME_PID_OUTPUT_PITCH_P     "PID Output Pitch P"
#define NAME_PID_OUTPUT_YAW_P       "PID Output Yaw P"
#define NAME_PID_OUTPUT_ROLL_I      "PID Output Roll I"
#define NAME_PID_OUTPUT_PITCH_I     "PID Output Pitch I"
#define NAME_PID_OUTPUT_YAW_I       "PID Output Yaw I"
#define NAME_PID_OUTPUT_ROLL_D      "PID Output Roll D"
#define NAME_PID_OUTPUT_PITCH_D     "PID Output Pitch D"
#define NAME_PID_OUTPUT_YAW_D       "PID Output Yaw D"
#define NAME_PID_OUTPUT_ROLL        "PID Output Roll"
#define NAME_PID_OUTPUT_PITCH       "PID Output Pitch"
#define NAME_PID_OUTPUT_YAW         "PID Output Yaw"


#define NAME_GPS_INFO               "GPS"
#define NAME_SATELLITES             "Satellites"
#define NAME_LATITUDE               "Latitude"
#define NAME_LONGITUDE              "Longitude"
#define NAME_DATE                   "Date"
#define NAME_TIME                   "Time"
#define NAME_SPEED                  "Speed [km/h]"
#define NAME_MAX_SPEED              "Max Speed [km/h]"
#define NAME_TOTAL_DISTANCE         "Total Distance [km]"

#define NAME_STEER_SERVO            "Steer Servo"
#define NAME_LEFT_ESCS              "Left Escs"
#define NAME_RIGHT_ESCS             "Right Escs"

#define ID_PROGRESS_RESP_TIME       "progressID_resp_time"
#define ID_SPAN_PROGRESS_RESP_TIME  "progressSpanID_resp_time"
#define ID_PROGRESS_WIFI_SS         "progressID_wifi_ss"
#define ID_SPAN_PROGRESS_WIFI_SS    "progressSpanID_wifi_ss"
#define ID_PROGRESS_VOLTAGE         "progressID_voltage"
#define ID_PROGRESS_CURRENT         "progressID_current"
#define ID_PROGRESS_CHANNEL_1       "progressID_ch1"
#define ID_PROGRESS_CHANNEL_2       "progressID_ch2"
#define ID_PROGRESS_CHANNEL_3       "progressID_ch3"
#define ID_SPAN_PROGRESS_CHANNEL_1  "progressSpanID_ch1"
#define ID_SPAN_PROGRESS_CHANNEL_2  "progressSpanID_ch2"
#define ID_SPAN_PROGRESS_CHANNEL_3  "progressSpanID_ch3"
#define ID_PROGRESS_LOOPTIME_1      "progressID_looptime1"
#define ID_PROGRESS_LOOPTIME_2      "progressID_looptime2"
#define ID_PROGRESS_LOOPTIME_3      "progressID_looptime3"
#define ID_PROGRESS_LOOPTIME_4      "progressID_looptime4"
#define ID_SPAN_PROGRESS_LOOPTIME_1 "progressSpanID_looptime1"
#define ID_SPAN_PROGRESS_LOOPTIME_2 "progressSpanID_looptime2"
#define ID_SPAN_PROGRESS_LOOPTIME_3 "progressSpanID_looptime3"
#define ID_SPAN_PROGRESS_LOOPTIME_4 "progressSpanID_looptime4"

#define ID_BUZZER_BUTTON            "buzzerButtonID"

#define INVISIBLE_STYLE             "style=\"display: none;\""

#define TELEMETRY_REFRESH_INTERVAL    "250"
#define TELEMETRY_RECEIVE_TIMEOUT     "1000"
#define IS_ALIVE_REFRESH_INTERVAL     "100"

#define ROW_HEIGHT_TH                 "20px"
#define ROW_HEIGHT_TD                 "18px"
#define LINE_HEIGHT_TD                "18px"
#define FONT_SIZE_TH                  "15px"
#define FONT_SIZE_TD                  "12px"
#define FONT_SIZE_BUTTON              "20px"


enum DrivingMode {
  dmHalfSpeed,
  dmFullSpeed
};


extern volatile int32_t wiFiSignalStrength;
extern volatile unsigned long usedUpLoopTimeTask1, usedUpLoopTimeTask2, usedUpLoopTimeTask3, usedUpLoopTimeTask4;
extern volatile int channel[];
extern volatile int prevAuxChannel;
extern volatile unsigned long timer_1, timer_2, timer_3;
extern volatile bool signal_detected, prev_signal_detected;
extern volatile bool is_armed;
extern volatile float voltage, current;
extern volatile int nrOfCells;
extern MPU6050 mpu6050;
extern volatile double angle_roll;
extern volatile double angle_pitch;
extern volatile double angle_yaw;
extern volatile double yaw_level_adjust;
extern volatile double gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
extern volatile double pid_yaw_setpoint;

extern volatile unsigned long signal_micros_timer_value;
extern volatile int steerServo, leftEscs, rightEscs;
extern int speed_input, steer_input, leftSpeedSteer, rightSpeedSteer;
extern bool isVoltageAlarmEnabled;
extern bool hasGPSSensor;
extern bool hasCurrentSensor;
extern DrivingMode drivingMode;
extern NeoPixelBus<PIXEL_COLOR_FEATURE, PIXEL_T_METHOD> *strip;
extern unsigned int nrOfLEDStrips;
extern String robotName;
extern HardwareSerial hs;

extern double maxSpeed;
extern double totalDistance;

extern volatile bool buzzerDisabled;
extern volatile bool buzzerOff;
extern volatile double steerExpoFactor;
extern volatile int steerServoCenterOffset;
extern volatile int speedEscCenterOffset;
extern volatile double voltageCorrectionFactor;
extern volatile double currentCorrectionFactor;



const uint8_t BREADBOARD_RACECAR[UniqueIDsize] = {0xF0, 0x08, 0xD1, 0xD2, 0x7F, 0x3C};
const uint8_t GREEN_RACECAR[UniqueIDsize] = {0x24, 0x62, 0xAB, 0xD5, 0x21, 0x18};
const uint8_t RED_RACECAR[UniqueIDsize] = {0xF0, 0x08, 0xD1, 0xD2, 0x7C, 0xAC};
const uint8_t GO_KART_ROENIE[UniqueIDsize] = {0x7C, 0x9E, 0xBD, 0xF1, 0x97, 0x74};
const uint8_t GO_KART_PROJUNK[UniqueIDsize] = {0x7C, 0x9E, 0xBD, 0xF1, 0xC8, 0xA8};
const uint8_t GO_KART_Thomas[UniqueIDsize] = {0x7C, 0x9E, 0xBD, 0xF0, 0x89, 0x98};
const uint8_t GO_KART_ProJunk[UniqueIDsize] = {0x7C, 0x9E, 0xBD, 0xF9, 0xFB, 0x7C};


#define colorSaturation 128
const RgbColor red(colorSaturation, 0, 0);
const RgbColor green(0, colorSaturation, 0);
const RgbColor blue(0, 0, colorSaturation);
const RgbColor white(colorSaturation);
const RgbColor orange(255, 69, 0);
const RgbColor grey(10, 10, 10);
const RgbColor black(0);


const double defaultCalibratedAccX = 0.0;
const double defaultCalibratedAccY = 0.0;
const double defaultCalibratedAccZ = 0.0;

const boolean throttleChannelReversed = false;
const boolean yawChannelReversed = false;
const boolean auxChannelReversed = false;


// [0..1] // 0-> no expo, 1-> max expo
const double defaultSteerExpoFactor = 0.8;
const int defaultSteerServoCenterOffset = 0;
const int defaultSpeedEscCenterOffset = 0;
const double defaultVoltageCorrectionFactor = 1.0;
const double defaultCurrentCorrectionFactor = 1.0;

const double defaultYawP = 1.5;
const double defaultYawI = 0.0;
const double defaultYawD = 0.0;

const int MIN_RESPONSE_TIME = 0;
const int WARNING_RESPONSE_TIME = 500;
const int BAD_RESPONSE_TIME = 1000;
const int MAX_RESPONSE_TIME = 1500;

const int32_t MIN_SIGNAL_STRENGTH = -90;
const int32_t LOW_SIGNAL_STRENGTH = -80;
const int32_t WARNING_SIGNAL_STRENGTH = -67;
const int32_t MAX_SIGNAL_STRENGTH = -30;

const float LOW_VOLTAGE_ALARM = 3.5;
const float FULLY_CHARGED_VOLTAGE = 4.2;
const float WARNING_VOLTAGE = 3.8;

const float SAFE_CURRENT = 120.0;
const float WARNING_CURRENT = 160.0;
const float FULL_SCALE_CURRENT = 200.0;


class PID {
    private:
        double defaultP;
        double defaultI;
        double defaultD;
        double defaultMax;
        double P;
        double I;
        double D;
        double max;
        double loopTime;
        String fname;
    public:
        PID() {}
        PID(double prmP, double prmI, double prmD, double prmMax, double prmLoopTime, String prmFName) : defaultP(prmP), defaultI(prmI), defaultD(prmD), defaultMax(prmMax), P(prmP), I(prmI), D(prmD), max(prmMax), loopTime(prmLoopTime), fname(prmFName) {}
        void set(String prmP, String prmI, String prmD, String prmMax) { P = prmP.toDouble(); I = prmI.toDouble() ; D = prmD.toDouble(); max = prmMax.toDouble(); }
        double getP() { return P; }
        double getI() { return I; }
        double getD() { return D; }
        double getMax() { return max; }
        double getLoopTime() { return loopTime; }
        void resetToDefault() { P = defaultP; I = defaultI, D = defaultD, max = defaultMax; }
        void load();
        void save();
        void print() { Serial.print(P); Serial.print("\t"); Serial.print(I); Serial.print("\t"); Serial.print(D); Serial.print("\t"); Serial.print(max); Serial.println(); }
};

class PIDOutput {
    private:
        PID *pid;
        double error;
        double prevError;
        double prevI;
        double P;
        double I;
        double D;
        double output;
    public:
        PIDOutput(PID *prmPid) : pid(prmPid), prevError(0.0), prevI(0.0) {}
        double getError() { return error; }
        double getPrevError() { return prevError; }
        double getP() { return P; }
        double getI() { return I; }
        double getD() { return D; }
        double getOutput() { return output; }
        void calc(double prmGyroAxisInput, double prmSetPoint);
        void reset() { prevError = 0.0; prevI = 0.0; }
};


extern PID yawPID;
extern PIDOutput yawOutputPID;

extern double LowPassFilter(const double prmAlpha, const double prmCurrentValue, const double prmPreviousValue);
extern double checkExpo(const double prmExpoFactor);
extern int checkCenterOffset(const int prmCenterOffset);
extern void printProps();
extern void loadProps();
extern void saveProps();
extern int getExpo(const int prmPulse, const double prmExpoFactor);
extern void IRAM_ATTR handler_channel_1();
extern void IRAM_ATTR handler_channel_2();
extern void IRAM_ATTR handler_channel_3();
extern void writeServoPWM(uint8_t prmChannel, uint32_t prmMicroSeconds);
extern void writeEscPWM(uint8_t prmChannel, uint32_t prmMicroSeconds);
extern void armEsc();
extern String identifyRobot();
extern double toDegrees(double prmRadians);
extern String getSSID();
extern void playShortBeep();
extern unsigned int getNrOfLEDStrips();
extern boolean getHasGPSSensor();
extern boolean getHasCurrentSensor();
extern DrivingMode getDefaultDrivingMode();
extern int fixChannelDirection(int prmChannel, boolean prmReversed);
extern double getLoopTimeHz(int prmLoopTime);
extern double calcPidSetPoint(int prmChannel);
extern int limitServo(int prmPulse);
extern void checkIsArmed();
extern bool isArmed();
extern void initValues();
extern void playCalibrate();
extern void waitPlayReady();
extern void playShortBeep();
extern void playLowVoltageAlarm();
extern void playArmed();
extern void playSignalDetected();
extern void playSignalLost();
extern bool isValidSignalValue(int prmSignalValue);
extern bool isZeroSpeed(int prmSpeedSignal);
extern int getMaxSpeed();
extern int getMinSpeed();
extern void calcMotorValues(int prmYawChannel, int prmThrottleChannel);
extern float readCurrentSensor();
extern int getNrOfCells(float prmVTotal);
extern int32_t getWiFiSignalStrength();
extern float readVoltage();
extern String getVoltageStr();
extern float readCurrent();
extern String getCurrentStr();
extern int getSpeed();
extern void incDrivingMode();
extern void playDrivingMode();
extern void initLEDs();
extern void updateLEDs();
extern void print_signals();
extern void print_calculated_values();
extern uint8_t* getChannelWithStrongestSignal(String prmSSID[], int prmNrOfSSIDs, int32_t *prmStrongestChannel, int *prmStrongsetSSIDIndex);
extern void task1(void *parameter);
extern void task2(void *parameter);
extern void task3(void *parameter);
extern void task4(void *parameter);
extern void getGPSData();
extern String getIdFromName(String prmName);
extern bool isAPStarted();
extern void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info);
extern String getWebPage(String prmToBeClickedTabButton);
extern String getLatestData();

extern String getSpeedoMeterStyle();
extern String getSpeedoMeterFunctions();


#endif
