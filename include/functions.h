#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NonBlockingRtttl.h>
#include <ArduinoUniqueID.h>
#include <Adafruit_PWMServoDriver.h>
#include <FS.h>
#include <SPIFFS.h>
#include <..\..\MyCommon\Credentials.h>


#define eps                     0.00001

#define CHINOOK_VERSION         "1.2"

// CORE0 is used by WIFI
#define CORE0                       0
#define CORE1                       1

#define STACK_SIZE_CORE             4096

#define FORMAT_SPIFFS_IF_FAILED true

#define LOOP_TIME_TASK1             40000    // microseconds; 25Hz
#define LOOP_TIME_TASK2             10000    // microseconds; 100Hz
#define LOOP_TIME_TASK3             4000     // microseconds; 250Hz
#define LOOP_TIME_TASK4             10000    // microseconds; 100Hz


#define PPM_PULSE_TRAIN_PERIOD  5000   // microsec
#define NR_OF_RECEIVER_CHANNELS 8      //FS-IA6B
#define MAX_PEAK_COUNT          (NR_OF_RECEIVER_CHANNELS+1)

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

#define FLIGHT_MODE_MAX_ANGLE       30

#define VOLTAGE_NOICE_FILTER        0.92

#define SSID_BASE                   "CHINOOK_ESP32_"

#define NAME_TAB_TELEMETRY          "Telemetry"
#define NAME_TAB_SETTINGS           "Settings"

#define NAME_TAB_BUTTON_TELEMETRY   "BTN_" NAME_TAB_TELEMETRY
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
#define NAME_ANGLE_YAW_ACC          "angle_yaw_acc"
#define NAME_SETTINGS               "Settings"
#define NAME_ROLL_EXPO              "Roll Expo"
#define NAME_PITCH_EXPO             "Pitch Expo"
#define NAME_YAW_EXPO               "Yaw Expo"
#define NAME_FRONT_SERVO_CENTER_OFFSET "Front Servo Center Offset"
#define NAME_BACK_SERVO_CENTER_OFFSET  "Back Servo Center Offset"
#define NAME_VOLTAGE_CORRECTION     "Voltage Correction Factor"
#define NAME_CALIBRATED_ROLL_ANGLE  "Calibrated Roll Angle"
#define NAME_CALIBRATED_PITCH_ANGLE "Calibrated Pitch Angle"

#define NAME_PID_SETTINGS_ROLL      "Roll"
#define NAME_PID_SETTINGS_PITCH     "Pitch"
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

const int defaultFrontServoCenterOffset = 110;
const int defaultBackServoCenterOffset = 100;
const double defaultVoltageCorrectionFactor = 1.0;
const double defaultCalibratedRollAngleAcc = -0.68;
const double defaultCalibratedPitchAngleAcc = 12.32;

const double driftCorrectionFactor = 0.001; //0.004

const boolean rollChannelReversed = false;
const boolean pitchChannelReversed = true;
const boolean throttleChannelReversed = false;
const boolean yawChannelReversed = false;

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


extern volatile int32_t wiFiSignalStrength;
extern volatile unsigned long usedUpLoopTimeTask1, usedUpLoopTimeTask2, usedUpLoopTimeTask3, usedUpLoopTimeTask4;
extern volatile short gyro_x, gyro_y, gyro_z;
extern volatile short acc_x, acc_y, acc_z;
extern volatile short temperature;
extern long gyro_x_cal, gyro_y_cal, gyro_z_cal;
extern volatile double angle_roll_acc, angle_pitch_acc, angle_yaw_acc;
extern volatile double angle_pitch, angle_roll, angle_yaw;
extern volatile double calibrated_angle_roll_acc, calibrated_angle_pitch_acc;
extern volatile double roll_level_adjust, pitch_level_adjust, yaw_level_adjust;
extern volatile double gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
extern volatile double pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
extern bool mpu_6050_found;
extern String robotName;
extern volatile bool signal_detected;
extern int signal_detected_count;
extern int signal_lost_count;
extern Adafruit_PWMServoDriver pwm;
extern volatile int channel[];
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


enum FlightMode { fmAutoLevel, fmAngleLimit, fmNone };

extern FlightMode flightMode;


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
        String fname;
    public:
        PID() {}
        PID(double prmP, double prmI, double prmD, double prmMax, String prmFName) : defaultP(prmP), defaultI(prmI), defaultD(prmD), defaultMax(prmMax), P(prmP), I(prmI), D(prmD), max(prmMax), fname(prmFName) {}
        void set(String prmP, String prmI, String prmD, String prmMax) { P = prmP.toDouble(); I = prmI.toDouble() ; D = prmD.toDouble(); max = prmMax.toDouble(); }
        double getP() { return P; }
        double getI() { return I; }
        double getD() { return D; }
        double getMax() { return max; }
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

class GYROAxis {
    private:
        volatile short *axis;
        bool isReversed;
    public:
        GYROAxis(volatile short *prmAxis, bool prmIsReversed) : axis(prmAxis), isReversed(prmIsReversed) {}
        double get() { return isReversed ? -1*(*axis) : *axis; }
};

class TwoPosSwitch {
    private:
        int channelNr;
    public:
        TwoPosSwitch() {}
        TwoPosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (channel[channelNr] < MID_CHANNEL) ? 1 : 2; }
};

class ThreePosSwitch {
    private:
        int channelNr;
    public:
        ThreePosSwitch() {}
        ThreePosSwitch(int prmChannenNr) : channelNr(prmChannenNr)  {}
        int readPos() { return (channel[channelNr] < 1250) ? 1 : (channel[channelNr] > 1750) ? 3 : 2; }
};


extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;

extern PIDOutput rollOutputPID;
extern PIDOutput pitchOutputPID;
extern PIDOutput yawOutputPID;

extern GYROAxis gyro_roll;
extern GYROAxis gyro_pitch;
extern GYROAxis gyro_yaw;
extern GYROAxis acc_roll;
extern GYROAxis acc_pitch;
extern GYROAxis acc_yaw;

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
extern bool is_mpu_6050_found();
extern void setup_mpu_6050_registers();
extern void read_mpu_6050_data();
extern void calibrate_mpu_6050();
extern float getTempCelsius();
extern void print_gyro_values();
extern void print_axis_values(double prmGyroRoll, double prmGyroPitch, double prmGyroYaw);
extern void printSetPoints(double prmRollSetPoint, double prmPitchSetPoint, double prmYawSetPoint);
extern void printPIDOutputs(double prmOutputRoll, double prmOutputPitch, double prmOutputYaw);
extern void printMotorOutputs(int prmFrontEsc, int prmBackEsc, int prmFrontServo, int prmBackServo);
extern double calcDegreesPerSecond(double prmGyroAxisInput, double prmGyroAxis);
extern void calibrateAcc();
extern void calcAngles();
extern void calcLevelAdjust(FlightMode prmFlightMode);
extern double calcPidSetPoint(int prmChannel, double prmLevelAdjust);
extern void delayEx(uint32_t prmMilisec);
extern bool isBootButtonPressed();
extern bool isBootButtonReleased();
extern void waitForBootButtonClicked();
extern void calibrateESCs();
extern int limitEsc(int prmPulse);
extern int limitServo(int prmPulse);
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
extern uint8_t* getChannelWithStrongestSignal(String prmSSID, int32_t *prmStrongestChannel);
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
