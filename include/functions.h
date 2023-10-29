#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PPMReceiver.h>
#include <pid.h>
#include <esp_wifi.h>
#include <NonBlockingRtttl.h>
#include <ArduinoUniqueID.h>
#include <Adafruit_PWMServoDriver.h>
#include <FS.h>
#include <SPIFFS.h>
#include <mpu6050.h>
#include <..\..\MyCommon\Credentials.h>


#define eps                     0.00001

#define CHINOOK_VERSION         "1.3"

// CORE0 is used by WIFI
#define CORE0                       0
#define CORE1                       1

#define STACK_SIZE_CORE             4096

#define FORMAT_SPIFFS_IF_FAILED true

#define LOOP_TIME_TASK1             40000    // microseconds; 25Hz
#define LOOP_TIME_TASK2             10000    // microseconds; 100Hz
#define LOOP_TIME_TASK3             4000     // microseconds; 250Hz
#define LOOP_TIME_TASK4             10000    // microseconds; 100Hz


#define NR_OF_RECEIVER_CHANNELS 8      //FS-IA6B

#define ROLL_CHANNEL              1
#define PITCH_CHANNEL             2
#define THROTTLE_CHANNEL          3
#define YAW_CHANNEL               4
#define SWA_CHANNEL               5
#define SWB_CHANNEL               7
#define SWC_CHANNEL               6
#define SWD_CHANNEL               8

#define RECEIVER_PPM_PIN          33
#define BUZZER_PIN                23
#define VOLTAGE_SENSOR_PIN        32

#define PWM_FREQUENCY_SERVO       50
#define PWM_RESOLUTION_SERVO      12
#define PWM_FREQUENCY_ESC         250
#define PWM_RESOLUTION_ESC        12

// channel 0 is in use by buzzer; channel 4 does not seem to work correct
#define SERVO_FRONT_PWM_CHANNEL   3
#define SERVO_BACK_PWM_CHANNEL    8
#define MOTOR_FRONT_PWM_CHANNEL   5
#define MOTOR_BACK_PWM_CHANNEL    6

#define SERVO_FRONT_PIN          14
#define SERVO_BACK_PIN           26
#define MOTOR_FRONT_PIN          27
#define MOTOR_BACK_PIN           25

#define GYRO_CALIBRATION_COUNT   250

#define MID_CHANNEL              1500
#define DEADBAND_HALF            8
#define MAX_THROTTLE             1800
#define MIN_THROTTLE             1200
#define MIN_PULSE                1000
#define MAX_PULSE                2000
#define SIGNAL_LOST_PULSE         950
#define INVALID_SIGNAL_PULSE      800
#define SIGNALS_DETECTED_LOST_THRESHOLD   50 

#define FLIGHT_MODE_MAX_ROLL_ANGLE     50.0
#define FLIGHT_MODE_MAX_PITCH_ANGLE    50.0

#define VOLTAGE_NOICE_FILTER        0.92

#define SSID_BASE                   "CHINOOK_ESP32_"

#define NAME_TAB_TELEMETRY          "Telemetry"
#define NAME_TAB_PID                "Pid"
#define NAME_TAB_SETTINGS           "Settings"

#define NAME_TAB_BUTTON_TELEMETRY   "BTN_" NAME_TAB_TELEMETRY
#define NAME_TAB_BUTTON_PID         "BTN_" NAME_TAB_PID
#define NAME_TAB_BUTTON_SETTINGS    "BTN_" NAME_TAB_SETTINGS 

#define NAME_MODEL                  "Model"
#define NAME_VERSION                "Version"
#define NAME_WIFI_SIGNAL_STRENGTH   "WiFi Signal Strength [dB]"
#define NAME_RESPONSE_TIME          "Response Time [ms]"
#define NAME_CHANNEL_1              "Channel 1 (Roll)"
#define NAME_CHANNEL_2              "Channel 2 (Pitch)"
#define NAME_CHANNEL_3              "Channel 3 (Throttle)"
#define NAME_CHANNEL_4              "Channel 4 (Yaw)"
#define NAME_CHANNEL_5              "Channel 5 (Aux 1)"
#define NAME_CHANNEL_6              "Channel 6 (Aux 2)"
#define NAME_CHANNEL_7              "Channel 7 (Aux 3)"
#define NAME_CHANNEL_8              "Channel 8 (Aux 4)"
#define NAME_SIGNAL_DETECTED        "Signal Detected"
#define NAME_ARMED                  "Armed"
#define NAME_FLIGHT_MODE            "Flight Mode"
#define NAME_VOLTAGE                "Voltage [V]"
#define NAME_VOLTAGE_PROGRESS       "Voltage"
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
#define NAME_ANGLE_ROLL_ACC_COPY    "angle_roll_acc_copy"
#define NAME_ANGLE_PITCH_ACC_COPY   "angle_pitch_acc_copy"
#define NAME_ANGLE_YAW_ACC          "angle_yaw_acc"
#define NAME_SETTINGS               "Settings"
#define NAME_ROLL_EXPO              "Roll Expo"
#define NAME_PITCH_EXPO             "Pitch Expo"
#define NAME_YAW_EXPO               "Yaw Expo"
#define NAME_FRONT_SERVO_CENTER_OFFSET "Front Servo Center Offset"
#define NAME_BACK_SERVO_CENTER_OFFSET  "Back Servo Center Offset"
#define NAME_VOLTAGE_CORRECTION     "Voltage Correction Factor"
#define NAME_CALIBRATED_ACCX        "Calibrated AccX"
#define NAME_CALIBRATED_ACCY        "Calibrated AccY"
#define NAME_CALIBRATED_ACCZ        "Calibrated AccZ"

#define NAME_PID_SETTINGS_ROLL_ANGLE   "Roll Angle"
#define NAME_PID_SETTINGS_PITCH_ANGLE  "Pitch Angle"

#define NAME_PID_SETTINGS_ROLL_RATE    "Roll Rate"
#define NAME_PID_SETTINGS_PITCH_RATE   "Pitch Rate"
#define NAME_PID_SETTINGS_YAW_RATE     "Yaw Rate"

#define NAME_TELEMETRY_ROLL_ANGLE   "Roll Angle"
#define NAME_TELEMETRY_PITCH_ANGLE  "Pitch Angle"
#define NAME_TELEMETRY_ROLL_RATE    "Roll Rate"
#define NAME_TELEMETRY_PITCH_RATE   "Pitch Rate"
#define NAME_TELEMETRY_YAW_RATE     "Yaw Rate"
#define NAME_ANGLE_ROLL             "Angle Roll"
#define NAME_ANGLE_PITCH            "Angle Pitch"
#define NAME_ANGLE_ROLL_COPY        "Angle Roll Copy"
#define NAME_ANGLE_PITCH_COPY       "Angle Pitch Copy"
#define NAME_ANGLE_YAW              "Angle Yaw"
#define NAME_KALMAN_ROLL_ANGLE_INPUT   "Kalman Roll Angle Input"
#define NAME_KALMAN_PITCH_ANGLE_INPUT  "Kalman Pitch Angle Input"
#define NAME_GYRO_ROLL_RATE_INPUT   "Gyro Roll Rate Input"
#define NAME_GYRO_PITCH_RATE_INPUT  "Gyro Pitch Rate Input"
#define NAME_GYRO_YAW_RATE_INPUT    "Gyro Yaw Rate Input"
#define NAME_PID_ROLL_ANGLE_DESIRED  "PID Roll Angle Desired"
#define NAME_PID_PITCH_ANGLE_DESIRED "PID Pitch Angle Desired"
#define NAME_PID_ROLL_RATE_DESIRED  "PID Roll Rate Desired"
#define NAME_PID_PITCH_RATE_DESIRED "PID Pitch Rate Desired"
#define NAME_PID_YAW_RATE_DESIRED   "PID Yaw Rate Desired"

#define NAME_PID_OUTPUT_ROLL_ANGLE_ERROR  "PID Output Roll Angle Error"
#define NAME_PID_OUTPUT_PITCH_ANGLE_ERROR "PID Output Pitch Angle Error"
#define NAME_PID_OUTPUT_ROLL_ANGLE_P      "PID Output Roll Angle P"
#define NAME_PID_OUTPUT_PITCH_ANGLE_P     "PID Output Pitch Angle P"
#define NAME_PID_OUTPUT_ROLL_ANGLE_I      "PID Output Roll Angle I"
#define NAME_PID_OUTPUT_PITCH_ANGLE_I     "PID Output Pitch Angle I"
#define NAME_PID_OUTPUT_ROLL_ANGLE_D      "PID Output Roll Angle D"
#define NAME_PID_OUTPUT_PITCH_ANGLE_D     "PID Output Pitch Angle D"
#define NAME_PID_OUTPUT_ROLL_ANGLE        "PID Output Roll Angle"
#define NAME_PID_OUTPUT_PITCH_ANGLE       "PID Output Pitch Angle"

#define NAME_PID_OUTPUT_ROLL_RATE_ERROR  "PID Output Roll Rate Error"
#define NAME_PID_OUTPUT_PITCH_RATE_ERROR "PID Output Pitch Rate Error"
#define NAME_PID_OUTPUT_YAW_RATE_ERROR   "PID Output Yaw Rate Error"
#define NAME_PID_OUTPUT_ROLL_RATE_P      "PID Output Roll Rate P"
#define NAME_PID_OUTPUT_PITCH_RATE_P     "PID Output Pitch Rate P"
#define NAME_PID_OUTPUT_YAW_RATE_P       "PID Output Yaw Rate P"
#define NAME_PID_OUTPUT_ROLL_RATE_I      "PID Output Roll Rate I"
#define NAME_PID_OUTPUT_PITCH_RATE_I     "PID Output Pitch Rate I"
#define NAME_PID_OUTPUT_YAW_RATE_I       "PID Output Yaw Rate I"
#define NAME_PID_OUTPUT_ROLL_RATE_D      "PID Output Roll Rate D"
#define NAME_PID_OUTPUT_PITCH_RATE_D     "PID Output Pitch Rate D"
#define NAME_PID_OUTPUT_YAW_RATE_D       "PID Output Yaw Rate D"
#define NAME_PID_OUTPUT_ROLL_RATE        "PID Output Roll Rate"
#define NAME_PID_OUTPUT_PITCH_RATE       "PID Output Pitch Rate"
#define NAME_PID_OUTPUT_YAW_RATE         "PID Output Yaw Rate"

#define NAME_FRONT_ESC              "Front Esc"
#define NAME_BACK_ESC               "Back Esc"
#define NAME_FRONT_SERVO            "Front Servo"
#define NAME_BACK_SERVO             "Back Servo"

#define ID_PROGRESS_RESP_TIME       "progressID_resp_time"
#define ID_SPAN_PROGRESS_RESP_TIME  "progressSpanID_resp_time"
#define ID_PROGRESS_WIFI_SS         "progressID_wifi_ss"
#define ID_SPAN_PROGRESS_WIFI_SS    "progressSpanID_wifi_ss"
#define ID_PROGRESS_VOLTAGE         "progressID_voltage"
#define ID_PROGRESS_CHANNEL_1       "progressID_ch1"
#define ID_PROGRESS_CHANNEL_2       "progressID_ch2"
#define ID_PROGRESS_CHANNEL_3       "progressID_ch3"
#define ID_PROGRESS_CHANNEL_4       "progressID_ch4"
#define ID_PROGRESS_CHANNEL_5       "progressID_ch5"
#define ID_PROGRESS_CHANNEL_6       "progressID_ch6"
#define ID_PROGRESS_CHANNEL_7       "progressID_ch7"
#define ID_PROGRESS_CHANNEL_8       "progressID_ch8"
#define ID_SPAN_PROGRESS_CHANNEL_1  "progressSpanID_ch1"
#define ID_SPAN_PROGRESS_CHANNEL_2  "progressSpanID_ch2"
#define ID_SPAN_PROGRESS_CHANNEL_3  "progressSpanID_ch3"
#define ID_SPAN_PROGRESS_CHANNEL_4  "progressSpanID_ch4"
#define ID_SPAN_PROGRESS_CHANNEL_5  "progressSpanID_ch5"
#define ID_SPAN_PROGRESS_CHANNEL_6  "progressSpanID_ch6"
#define ID_SPAN_PROGRESS_CHANNEL_7  "progressSpanID_ch7"
#define ID_SPAN_PROGRESS_CHANNEL_8  "progressSpanID_ch8"
#define ID_PROGRESS_LOOPTIME_1      "progressID_looptime1"
#define ID_PROGRESS_LOOPTIME_2      "progressID_looptime2"
#define ID_PROGRESS_LOOPTIME_3      "progressID_looptime3"
#define ID_PROGRESS_LOOPTIME_4      "progressID_looptime4"
#define ID_SPAN_PROGRESS_LOOPTIME_1 "progressSpanID_looptime1"
#define ID_SPAN_PROGRESS_LOOPTIME_2 "progressSpanID_looptime2"
#define ID_SPAN_PROGRESS_LOOPTIME_3 "progressSpanID_looptime3"
#define ID_SPAN_PROGRESS_LOOPTIME_4 "progressSpanID_looptime4"

#define ID_BUZZER_BUTTON            "buzzerButtonID"

#define TELEMETRY_REFRESH_INTERVAL    "250"
#define TELEMETRY_RECEIVE_TIMEOUT     "1000"
#define IS_ALIVE_REFRESH_INTERVAL     "100"

#define ROW_HEIGHT_TH                 "20px"
#define ROW_HEIGHT_TD                 "18px"
#define LINE_HEIGHT_TD                "18px"
#define FONT_SIZE_TH                  "15px"
#define FONT_SIZE_TD                  "12px"
#define FONT_SIZE_BUTTON              "20px"

const uint8_t CHINOOK[UniqueIDsize] = {0xF0, 0x08, 0xD1, 0xD2, 0x4F, 0xFC};

// [0..1] // 0-> no expo, 1-> max expo
const double defaultRollExpoFactor = 0.10f; 
const double defaultPitchExpoFactor = 0.10f; 
const double defaultYawExpoFactor = 0.30f; 

const int defaultFrontServoCenterOffset = 0;
const int defaultBackServoCenterOffset = 0;
const double defaultVoltageCorrectionFactor = 1.0;

const boolean rollChannelReversed = false;
const boolean pitchChannelReversed = true;
const boolean throttleChannelReversed = false;
const boolean yawChannelReversed = true;

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


extern PPMReceiver *ppm;
extern volatile int32_t wiFiSignalStrength;
extern volatile unsigned long usedUpLoopTimeTask1, usedUpLoopTimeTask2, usedUpLoopTimeTask3, usedUpLoopTimeTask4;
extern MPU6050 mpu6050;
extern volatile double kalmanPitchAngle, kalmanRollAngle, yawAngle;
extern volatile double gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
extern volatile double desiredRollAngle, desiredPitchAngle;
extern volatile double desiredRollRate, desiredPitchRate, desiredYawRate;
extern String robotName;
extern volatile bool signal_detected;
extern int signal_detected_count;
extern int signal_lost_count;
extern Adafruit_PWMServoDriver pwm;
extern volatile float voltage;
extern volatile int frontEsc;
extern volatile int backEsc;
extern volatile int frontServo;
extern volatile int backServo;
extern volatile double rollExpoFactor;
extern volatile double pitchExpoFactor;
extern volatile double yawExpoFactor;
extern volatile int frontServoCenterOffset;
extern volatile int backServoCenterOffset;
extern volatile double voltageCorrectionFactor;
extern volatile bool buzzerDisabled;
extern volatile bool buzzerOff;
extern volatile bool currentIsArmed;
extern bool isVoltageAlarmEnabled;


enum FlightMode { fmAutoLevel, fmAngleLimit, fmNone };

extern FlightMode flightMode;



class TwoPosSwitch {
    private:
        int channelNr;
    public:
        TwoPosSwitch() {}
        TwoPosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (ppm->getValue(channelNr) < MID_CHANNEL) ? 1 : 2; }
};

class ThreePosSwitch {
    private:
        int channelNr;
    public:
        ThreePosSwitch() {}
        ThreePosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (ppm->getValue(channelNr) < 1250) ? 1 : (ppm->getValue(channelNr) > 1750) ? 3 : 2; }
};


extern PID rollAnglePID;
extern PID pitchAnglePID;

extern PIDOutput rollAngleOutputPID;
extern PIDOutput pitchAngleOutputPID;


extern PID rollRatePID;
extern PID pitchRatePID;
extern PID yawRatePID;

extern PIDOutput rollRateOutputPID;
extern PIDOutput pitchRateOutputPID;
extern PIDOutput yawRateOutputPID;


extern TwoPosSwitch switchA;
extern TwoPosSwitch switchB;
extern ThreePosSwitch switchC;
extern TwoPosSwitch switchD;


extern double LowPassFilter(const double prmAlpha, const double prmCurrentValue, const double prmPreviousValue);
extern double checkExpo(const double prmExpoFactor);
extern int checkCenterOffset(const int prmCenterOffset);
extern void printProps();
extern void loadProps();
extern void saveProps();
extern int getExpo(const int prmPulse, const double prmExpoFactor);
extern void initReceiver();
extern void IRAM_ATTR ppmInterruptHandler();
extern String identifyRobot();
extern double toDegrees(double prmRadians);
extern bool isValidSignal(int prmPulse);
extern int32_t getWiFiSignalStrength();
extern float readVoltage();
extern String getVoltageStr();
extern int fixChannelDirection(int prmChannel, boolean prmReversed);
extern void printSetPoints(double prmRollSetPoint, double prmPitchSetPoint, double prmYawSetPoint);
extern void printPIDOutputs(double prmOutputRoll, double prmOutputPitch, double prmOutputYaw);
extern void printMotorOutputs(int prmFrontEsc, int prmBackEsc, int prmFrontServo, int prmBackServo);
extern void delayEx(uint32_t prmMilisec);
extern double getLoopTimeHz(int prmLoopTime);
extern bool isBootButtonPressed();
extern bool isBootButtonReleased();
extern void waitForBootButtonClicked();
extern void calibrateESCs();
extern int limitEsc(int prmPulse);
extern int limitServo(int prmPulse);
extern bool isArmingSwitchTriggered();
extern bool isArmed();
extern bool isArmingAllowed();
extern void initValues();
extern FlightMode getFlightMode();
extern void playVeryShortBeep();
extern void playShortBeep();
extern void playLongBeep();
extern void playLowVoltageAlarm();
extern void playArmed();
extern void playDisarmed();
extern void playCalibrated();
extern void playSignalDetected();
extern void playSignalLost();
extern void printChannels();
extern void runOnCore2(void *parameter);
extern void writeServoPWM(uint8_t prmChannel, uint32_t prmMicroSeconds);
extern void writeEscPWM(uint8_t prmChannel, uint32_t prmMicroSeconds);
extern String getSSID();
extern uint8_t* getChannelWithStrongestSignal(String prmSSID[], int prmNrOfSSIDs, int32_t *prmStrongestChannel, int *prmStrongsetSSIDIndex);
extern void task1(void *parameter);
extern void task2(void *parameter);
extern void task3(void *parameter);
extern void task4(void *parameter);
extern String getIdFromName(String prmName);
extern bool isAPStarted();
extern void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info);
extern String getWebPage(String prmToBeClickedTabButton);
extern String getLatestData();


#endif
