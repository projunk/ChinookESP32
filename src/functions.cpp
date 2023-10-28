#include <functions.h>


volatile int32_t wiFiSignalStrength = MIN_SIGNAL_STRENGTH;
volatile bool APStarted = false;
volatile int channel[NR_OF_RECEIVER_CHANNELS];
volatile int prevAuxChannel;
volatile unsigned long timer_1, timer_2, timer_3;
volatile bool signal_detected, prev_signal_detected;
volatile bool is_armed = false;
volatile float voltage, current;
volatile int nrOfCells = 0;
volatile unsigned long signal_micros_timer_value;
volatile int steerServo, leftEscs, rightEscs;
int speed_input, steer_input, leftSpeedSteer, rightSpeedSteer;
bool isVoltageAlarmEnabled = true;
bool hasGPSSensor;
bool hasCurrentSensor;
DrivingMode drivingMode;
NeoPixelBus<PIXEL_COLOR_FEATURE, PIXEL_T_METHOD> *strip;
unsigned int nrOfLEDStrips;
String robotName;
volatile bool buzzerDisabled = false;
volatile bool buzzerOff = false;
volatile double steerExpoFactor = defaultSteerExpoFactor;
volatile int steerServoCenterOffset = defaultSteerServoCenterOffset;
volatile int speedEscCenterOffset = defaultSpeedEscCenterOffset;
volatile double voltageCorrectionFactor = defaultVoltageCorrectionFactor;
volatile double currentCorrectionFactor = defaultCurrentCorrectionFactor;

volatile double yaw_level_adjust;
volatile double gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
volatile double pid_yaw_setpoint;

boolean startingPointFound = false;
float latitude, longitude, prev_latitude, prev_longitude;
String sat_str, date_str, time_str, lat_str, lng_str, speed_str, max_speed_str, total_distance_str;
double maxSpeed = 0.0;
double totalDistance = 0.0;

uint32_t cp0_regs1[18], cp0_regs2[18], cp0_regs3[18]; 

TinyGPSPlus gps;

HardwareSerial hs(2);


PID yawPID(defaultYawP, defaultYawI, defaultYawD, 500.0, (LOOP_TIME_TASK3/1000000.0), "yaw");
PIDOutput yawOutputPID(&yawPID);



void PID::load() {
  Serial.println("PID::load;" + fname);
  buzzerDisabled = true;
  if (SPIFFS.exists("/" + fname)) {
    File f = SPIFFS.open("/" + fname, "r");
    if (f) {
      set(f.readStringUntil('\n'), f.readStringUntil('\n'), f.readStringUntil('\n'), f.readStringUntil('\n'));
      f.close();
      print();
    }
  }
  buzzerDisabled = false;
}


void PID::save() {
  Serial.println("PID::save;" + fname);
  buzzerDisabled = true;
  File f = SPIFFS.open("/" + fname, "w");
  if (f) {
    f.println(String(P, 2));
    f.println(String(I, 2));
    f.println(String(D, 2));
    f.println(String(max, 2));
    f.close();
  }
  buzzerDisabled = false;
}


double LowPassFilter(const double prmAlpha, const double prmCurrentValue, const double prmPreviousValue) {
  // https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter
  return prmAlpha * prmPreviousValue + (1.0 - prmAlpha) * prmCurrentValue;
}


double checkExpo(const double prmExpoFactor) {
  if (prmExpoFactor > 1.0) {
    return 1.0;
  }
  if (prmExpoFactor < 0.0) {
    return 0.0;
  }  
  return prmExpoFactor;
}


int checkCenterOffset(const int prmCenterOffset) {
  if (prmCenterOffset > (MAX_PULSE - MID_CHANNEL)) {
    return (MAX_PULSE - MID_CHANNEL);
  }
  if (prmCenterOffset < (MIN_PULSE - MID_CHANNEL)) {
    return (MIN_PULSE - MID_CHANNEL);
  }  
  return prmCenterOffset;
}


void printProps() {
  Serial.print(steerExpoFactor);
  Serial.print("\t");
  Serial.print(steerServoCenterOffset);
  Serial.print("\t");
  Serial.print(speedEscCenterOffset);
  Serial.print("\t");
  Serial.print(voltageCorrectionFactor);
  Serial.print("\t");
  Serial.print(currentCorrectionFactor);
  Serial.print("\t");
  Serial.print(mpu6050.getCalibrationAccX());  
  Serial.print("\t");
  Serial.print(mpu6050.getCalibrationAccY());  
  Serial.print("\t");
  Serial.print(mpu6050.getCalibrationAccZ());  
  Serial.println();  
}


void loadProps() {
  Serial.println("loadProps");
  buzzerDisabled = true;
  if (SPIFFS.exists("/props")) {
    File f = SPIFFS.open("/props", "r");  
    if (f) {
      steerExpoFactor = f.readStringUntil('\n').toDouble();
      steerServoCenterOffset = f.readStringUntil('\n').toInt();
      speedEscCenterOffset = f.readStringUntil('\n').toInt();
      voltageCorrectionFactor = f.readStringUntil('\n').toDouble();
      currentCorrectionFactor = f.readStringUntil('\n').toDouble();
      mpu6050.setCalibrationAccX(f.readStringUntil('\n').toDouble());
      mpu6050.setCalibrationAccY(f.readStringUntil('\n').toDouble());
      mpu6050.setCalibrationAccZ(f.readStringUntil('\n').toDouble());
      f.close();
    }
  }
  buzzerDisabled = false;
}


void saveProps() {
  Serial.println("saveProps");
  buzzerDisabled = true;
  File f = SPIFFS.open("/props", "w");
  if (f) {
    f.println(String(steerExpoFactor, 2));
    f.println(String(steerServoCenterOffset));
    f.println(String(speedEscCenterOffset));
    f.println(String(voltageCorrectionFactor, 2));
    f.println(String(currentCorrectionFactor, 2));
    f.println(String(mpu6050.getCalibrationAccX(), 2));
    f.println(String(mpu6050.getCalibrationAccY(), 2));
    f.println(String(mpu6050.getCalibrationAccZ(), 2));
    f.close();
  }
  buzzerDisabled = false;
}


int getExpo(const int prmInPulse, const double prmExpoFactor) {
  // convert to factor between -1.0 and 1.0
  double in = map(prmInPulse, MIN_PULSE, MAX_PULSE, -1000, 1000)/ 1000.0;
  double out = prmExpoFactor * in * in * in + (1.0 - prmExpoFactor) * in;
  int outPulse = map(round(1000.0 * out), -1000, 1000, MIN_PULSE, MAX_PULSE);
  /*
  Serial.print(prmInPulse);
  Serial.print("->");  
  Serial.print(in);
  Serial.print("->");
  Serial.print(out);
  Serial.print("->");
  Serial.println(outPulse);  
  */
  return outPulse;  
}


void IRAM_ATTR handler_channel_1() {
  vTaskSuspendAll(); 
  xthal_set_cpenable(1);
  xthal_save_cp0(cp0_regs1);

  unsigned long timer_value = micros();
  signal_micros_timer_value = timer_value;
  if (digitalRead(RECEIVER_STEER_PIN)) {
    timer_1 = timer_value;
  } else {
    channel[0] = timer_value - timer_1;
  }

  xthal_restore_cp0(cp0_regs1);
  xthal_set_cpenable(0);
  xTaskResumeAll();
}


void IRAM_ATTR handler_channel_2() {
  vTaskSuspendAll(); 
  xthal_set_cpenable(1);
  xthal_save_cp0(cp0_regs2);

  // throttle channel keeps receiving its pulse also when signal is lost
  unsigned long timer_value = micros();
  if (digitalRead(RECEIVER_THROTTLE_PIN)) {
    timer_2 = timer_value;
  } else {
    channel[1] = timer_value - timer_2;
  }

  // check for signal
  signal_detected = ((timer_value - signal_micros_timer_value) < SIGNAL_TIMEOUT);

  xthal_restore_cp0(cp0_regs2);
  xthal_set_cpenable(0);
  xTaskResumeAll();
}


void IRAM_ATTR handler_channel_3() {
  vTaskSuspendAll(); 
  xthal_set_cpenable(1);
  xthal_save_cp0(cp0_regs3);

  unsigned long timer_value = micros();
  if (digitalRead(RECEIVER_SPEED_MODE_PIN)) {
    timer_3 = timer_value;
  } else {
    channel[2] = timer_value - timer_3;
  }

  xthal_restore_cp0(cp0_regs3);
  xthal_set_cpenable(0);
  xTaskResumeAll();
}


void writeServoPWM(uint8_t prmChannel, uint32_t prmMicroSeconds) {
  uint32_t valueMax = 1000000/PWM_FREQUENCY_SERVO;
  uint32_t m = pow(2, PWM_RESOLUTION_SERVO) - 1;
  uint32_t duty = (m * min(prmMicroSeconds, valueMax)/valueMax);
  ledcWrite(prmChannel, duty);
}


void writeEscPWM(uint8_t prmChannel, uint32_t prmMicroSeconds) {
  uint32_t valueMax = 1000000/PWM_FREQUENCY_ESC;
  uint32_t m = pow(2, PWM_RESOLUTION_ESC) - 1;
  uint32_t duty = (m * min(prmMicroSeconds, valueMax)/valueMax);
  ledcWrite(prmChannel, duty);
}


void armEsc() {
  writeEscPWM(MOTOR_LB_PWM_CHANNEL, ESC_ARM_SIGNAL);
  writeEscPWM(MOTOR_RB_PWM_CHANNEL, ESC_ARM_SIGNAL);
  writeEscPWM(MOTOR_LF_PWM_CHANNEL, ESC_ARM_SIGNAL);
  writeEscPWM(MOTOR_RF_PWM_CHANNEL, ESC_ARM_SIGNAL);
  delayMicroseconds(ESC_ARM_DELAY);
  writeEscPWM(MOTOR_LB_PWM_CHANNEL, MID_CHANNEL);
  writeEscPWM(MOTOR_RB_PWM_CHANNEL, MID_CHANNEL);
  writeEscPWM(MOTOR_LF_PWM_CHANNEL, MID_CHANNEL);
  writeEscPWM(MOTOR_RF_PWM_CHANNEL, MID_CHANNEL);    
}


bool isEqualID(const uint8_t prmID[UniqueIDsize]) {
  for (size_t i = 0; i < UniqueIDsize; i++) {
			if (UniqueID[i] != prmID[i]) {
        return false;
      }
  }
  return true;
}


String identifyRobot() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return "Breadboard";
  } else if (isEqualID(GREEN_RACECAR)) {
    return "Green";
  } else if (isEqualID(RED_RACECAR)) {    
    return "Red";
  } else if (isEqualID(GO_KART_ROENIE)) {    
    return "Go-Kart ROENIE";
  } else if (isEqualID(GO_KART_PROJUNK)) {    
    return "Go-Kart Projunk";
  } else if (isEqualID(GO_KART_Thomas)) {    
    return "Go-Kart Thomas";
  } else if (isEqualID(GO_KART_ProJunk)) {    
    return "Go-Kart ProJunk";
  } else {
    return "Unknown";
  }
}


double toDegrees(double prmRadians) {
  return prmRadians * 180.0 / PI;
}


String getSSID() {
  String ssid = SSID_BASE + robotName;
  ssid.toUpperCase();
  return ssid;
}


unsigned int getNrOfLEDStrips() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return 2;
  } else if (isEqualID(GREEN_RACECAR)) {
    return 1;
  } else if (isEqualID(RED_RACECAR)) {
    return 1;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return 1;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return 1;
  } else if (isEqualID(GO_KART_Thomas)) {
    return 2;
  } else if (isEqualID(GO_KART_ProJunk)) {
    return 2;
  } else {
    return 0;
  }
}


boolean getHasGPSSensor() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return true;
  } else if (isEqualID(GREEN_RACECAR)) {
    return true;
  } else if (isEqualID(RED_RACECAR)) {
    return true;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return false;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return false;    
  } else if (isEqualID(GO_KART_Thomas)) {
    return false;    
  } else if (isEqualID(GO_KART_ProJunk)) {
    return false;    
  } else {
    return false;
  }
}


boolean getHasCurrentSensor() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return false;
  } else if (isEqualID(GREEN_RACECAR)) {
    return false;
  } else if (isEqualID(RED_RACECAR)) {
    return false;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return false;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return true;
  } else if (isEqualID(GO_KART_Thomas)) {
    return true;
  } else if (isEqualID(GO_KART_ProJunk)) {
    return true;
  } else {
    return false;
  }
}


int getKV() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return 850;
  } else if (isEqualID(GREEN_RACECAR)) {
    return 850;
  } else if (isEqualID(RED_RACECAR)) {
    return 850;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return 850;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return 850;
  } else if (isEqualID(GO_KART_Thomas)) {
    return 850;
  } else if (isEqualID(GO_KART_ProJunk)) {
    return 360;
  } else {
    return 0;
  }
}


int getTireDiameter() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return 72;
  } else if (isEqualID(GREEN_RACECAR)) {
    return 72;
  } else if (isEqualID(RED_RACECAR)) {
    return 72;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return 70;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return 70;
  } else if (isEqualID(GO_KART_Thomas)) {
    return 84;
  } else if (isEqualID(GO_KART_ProJunk)) {
    return 84;
  } else {
    return 0;
  }
}


DrivingMode getDefaultDrivingMode() {
  if (isEqualID(BREADBOARD_RACECAR)) {
    return dmHalfSpeed;
  } else if (isEqualID(GREEN_RACECAR)) {
    return dmHalfSpeed;
  } else if (isEqualID(RED_RACECAR)) {
    return dmHalfSpeed;
  } else if (isEqualID(GO_KART_ROENIE)) {
    return dmHalfSpeed;
  } else if (isEqualID(GO_KART_PROJUNK)) {
    return dmHalfSpeed;
  } else if (isEqualID(GO_KART_Thomas)) {
    return dmHalfSpeed;
  } else if (isEqualID(GO_KART_ProJunk)) {
    return dmHalfSpeed;
  } else {
    return dmFullSpeed;
  }
}


int fixChannelDirection(int prmChannel, boolean prmReversed) {
  if (prmReversed) {
    return map(prmChannel, MIN_PULSE, MAX_PULSE, MAX_PULSE, MIN_PULSE);
  } else {
    return prmChannel;
  }
}


double getLoopTimeHz(int prmLoopTime) {
  return 1000000.0 / prmLoopTime;
}


double calcPidSetPoint(int prmChannel) {
  return prmChannel - MID_CHANNEL;
}


void PIDOutput::calc(double prmGyroAxisInput, double prmSetPoint) {
  output = 0.0;
  error = prmSetPoint - prmGyroAxisInput;

  P = pid->getP() * error;

  I = prevI + pid->getI() * (error + prevError) * pid->getLoopTime() / 2;
  if (I > pid->getMax()) I = pid->getMax();
  else if (I < pid->getMax() * -1) I = pid->getMax() * -1;

  D = pid->getD() * (error - prevError) / pid->getLoopTime();
  output = P + I + D;
  if (output > pid->getMax()) output = pid->getMax();
  else if (output < pid->getMax() * -1) output = pid->getMax() * -1;

  prevError = error;
  prevI = I;
}


extern int limitServo(int prmPulse) {
  int rval = prmPulse;
  if (prmPulse > MAX_PULSE) rval = MAX_PULSE;
  if (prmPulse < MIN_PULSE) rval = MIN_PULSE;
  return rval;
}


void checkIsArmed() {
  static bool turnLeftDone = false;
  static bool prevTurnLeftDone = false;
  static bool turnRightDone = false;
  static bool prevTurnRightDone = false;
  static int turnLeftCount = 0;
  static int turnRightCount = 0;
  bool has_been_armed = turnLeftDone && turnRightDone;

  if (signal_detected && has_been_armed) {
    is_armed = true;
  } else if (signal_detected && !has_been_armed) {
    // check for arming sequence
    if (channel[0] < (MID_CHANNEL-THRESHOLD_SIGNAL)) {
      turnLeftCount++;
      if (turnLeftCount == SIGNALS_COUNT_THRESHOLD) {
        turnLeftDone = true;
        if (!prevTurnLeftDone) {
          Serial.println("Turn left done");
          playShortBeep();
          prevTurnLeftDone = true;
        }
      } else if  (turnLeftCount > SIGNALS_COUNT_THRESHOLD)  {
        turnLeftCount = SIGNALS_COUNT_THRESHOLD;
        turnRightCount = 0;
      }
    } else if (channel[0] > (MID_CHANNEL+THRESHOLD_SIGNAL) && turnLeftDone) {
      turnRightCount++;
      if (turnRightCount == SIGNALS_COUNT_THRESHOLD) {
        turnRightDone = true;
        if (!prevTurnRightDone) {
          Serial.println("Turn right done");
          writeServoPWM(MOTOR_STEERING_PWM_CHANNEL, MID_CHANNEL + steerServoCenterOffset);
          armEsc();
          playArmed();
          prevTurnRightDone = true;
        }
      } else if (turnRightCount > SIGNALS_COUNT_THRESHOLD) {
        turnRightCount = SIGNALS_COUNT_THRESHOLD;
        turnLeftCount = 0;
      }
    }
    is_armed = turnLeftDone && turnRightDone;
  } else {
    turnLeftDone = false;
    prevTurnLeftDone = false;
    turnRightDone = false;
    prevTurnRightDone = false;
    turnLeftCount = 0;
    turnRightCount = 0;
    is_armed = false;
  }
}


bool isArmed() {
  return is_armed;
}


void initValues() {
  gyro_roll_input = 0.0;
  gyro_pitch_input = 0.0;    
  gyro_yaw_input = 0.0;

  angle_roll = mpu6050.getAngleRollAcc();
  angle_pitch = mpu6050.getAnglePitchAcc();
  angle_yaw = 0.0;
  
  yawOutputPID.reset();
}


void waitPlayReady() {
  for (;;) {
    if (rtttl::done()) {
      break;
    }
    delay(1);
  }
}


void playTune(String prmTune) {
  if ((!buzzerDisabled) && (!buzzerOff)) {
    static char buf[64];
    strcpy(buf, prmTune.c_str());
    rtttl::begin(BUZZER_PIN, buf);
  }
}


void playCalibrate() {
  playTune("Calibrate:d=16,o=5,b=140:c5,P,c6,P,a7,P");
}


void playShortBeep() {
  playTune("ShortBeep:d=32,o=5,b=140:c5,P");
}


void playLowVoltageAlarm() {
  if (isVoltageAlarmEnabled) {
    playTune("LowVoltageAlarm:d=16,o=5,b=140:c6,P,c5,P,c6,P,c5,P");
  }
}


void playArmed() {
  Serial.println("Armed");        
  playTune("Armed:d=16,o=5,b=140:c5,P,c6,P,a7,P");
}


void playSignalDetected() {
  Serial.println("Signal Detected");   
  playTune("SignalDetected:d=16,o=5,b=140:c5,P,c5,P,c5,P,c5,P,c5,P,c5,P,a7,P");
}


void playSignalLost() {
  Serial.println("Signal Lost");   
  playTune("SignalLost:d=16,o=5,b=140:a7,P,c5,P,c5,P,c5,P,c5,P,c5,P,c5,P");  
}


bool isValidSignalValue(int prmSignalValue) {
  return ((prmSignalValue > MIN_VALID_SIGNAL_VALUE) && (prmSignalValue < MAX_VALID_SIGNAL_VALUE));
}


bool isZeroSpeed(int prmSpeedSignal) {
  return (abs(prmSpeedSignal - MID_CHANNEL) < DEADBAND_HALF);
}


int getMaxSpeed() {
  switch (drivingMode) {
    case dmHalfSpeed:
      return MID_CHANNEL+HALF_SPEED_DELTA_SIGNAL;
    default:
      return MID_CHANNEL+MAX_DELTA_SIGNAL;
  }
}


int getMinSpeed() {
  switch (drivingMode) {
    case dmHalfSpeed:
      return MID_CHANNEL-HALF_SPEED_DELTA_SIGNAL;
    default:
      return MID_CHANNEL-MAX_DELTA_SIGNAL;
  }
}


void calcMotorValues(int prmYawChannel, int prmThrottleChannel) {
  if (signal_detected) {
    steer_input = map(prmYawChannel, MID_CHANNEL-MAX_DELTA_SIGNAL, MID_CHANNEL+MAX_DELTA_SIGNAL, MIN_STEER, MAX_STEER);
    speed_input = map(prmThrottleChannel, MID_CHANNEL-MAX_DELTA_SIGNAL, MID_CHANNEL+MAX_DELTA_SIGNAL, MIN_SPEED, MAX_SPEED);
    if (steer_input > 0) {
      // turn right
      leftSpeedSteer = speed_input;
      rightSpeedSteer = map(steer_input, MAX_STEER, 0, MIN_STEER_FACTOR*leftSpeedSteer, leftSpeedSteer);          
    } else {
      // turn left
      rightSpeedSteer = speed_input;        
      leftSpeedSteer = map(steer_input, MIN_STEER, 0, MIN_STEER_FACTOR*rightSpeedSteer, rightSpeedSteer);          
    }
    steerServo = limitServo(MID_CHANNEL + yawOutputPID.getOutput() + steerServoCenterOffset);
    leftEscs = map(leftSpeedSteer + speedEscCenterOffset, MIN_SPEED, MAX_SPEED, getMinSpeed(), getMaxSpeed());
    rightEscs = map(rightSpeedSteer + speedEscCenterOffset, MIN_SPEED, MAX_SPEED, getMinSpeed(), getMaxSpeed());        
  } else {
    steerServo = MID_CHANNEL + steerServoCenterOffset;
    leftEscs = MID_CHANNEL + speedEscCenterOffset;
    rightEscs = MID_CHANNEL + speedEscCenterOffset;           
  }  

  if ((isZeroSpeed(leftEscs) && isZeroSpeed(rightEscs))) {
    leftEscs = MID_CHANNEL + speedEscCenterOffset;
    rightEscs = MID_CHANNEL + speedEscCenterOffset;
  }  
}  


float readCurrentSensor() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);
  int sensorValue = adc1_get_raw(ADC1_CHANNEL_5);
  Serial.println(sensorValue);
  return sensorValue * (3.3 / 4095.0);
}


int getNrOfCells(float prmVBatTotal) {
  const float ABS_MAX_CELLVOLTAGE = FULLY_CHARGED_VOLTAGE+0.2;
  
  if (prmVBatTotal < 0.1) {
    return 0;
  } else if (prmVBatTotal < ABS_MAX_CELLVOLTAGE) {
    return 1;
  } else if (prmVBatTotal < 2*ABS_MAX_CELLVOLTAGE) {
    return 2;
  } else if (prmVBatTotal < 3*ABS_MAX_CELLVOLTAGE) {
    return 3;
  } else if (prmVBatTotal < 4*ABS_MAX_CELLVOLTAGE) {
    return 4;
  } else {
    // not supported 
    return 0; 
  }   
}


int32_t getWiFiSignalStrength() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.RSSI();
  } else {
    return MIN_SIGNAL_STRENGTH;
  }
}


float readVoltage() {
  const float R1 = 4700.0;
  const float R2 = 1000.0;

  float vBatTotal = analogRead(VOLTAGE_SENSOR_PIN) * (3.3 / 4095.0) * (R1+R2)/R2;
  nrOfCells = getNrOfCells(vBatTotal);
  float cellVoltage = 0.0;
  if (nrOfCells > 0) {
    cellVoltage = voltageCorrectionFactor * vBatTotal / nrOfCells;
  }
  /*
  Serial.print(vBatTotal); 
  Serial.print("\t");   
  Serial.print(nrOfCells); 
  Serial.print("\t");   
  Serial.println(cellVoltage); 
  */
  return cellVoltage;
}


String getVoltageStr() {
  return String(voltage, 2);
}


float readCurrent() {
  if (hasCurrentSensor) {
    return currentCorrectionFactor * analogRead(CURRENT_SENSOR_PIN);
  } else {
    return 0;
  }
}


String getCurrentStr() {
  return String(current, 2);
}


int getSpeed() {
  float speed = 0.0f;
  if (isValidSignalValue(channel[1]) && isArmed()) {
    speed = PI*getTireDiameter()*getKV()*nrOfCells*voltage*60/1000000*abs(channel[1]-MID_CHANNEL)/(MAX_PULSE-MID_CHANNEL);
    speed = speed*(getMaxSpeed()-MID_CHANNEL)/MAX_DELTA_SIGNAL;
  }
  return round(speed);
}


void incDrivingMode() {
  int dm = drivingMode;
  dm++;
  if (dm > dmFullSpeed) {
    dm = dmHalfSpeed;
  }
  drivingMode = (DrivingMode) dm;
}


void playDrivingMode() {
  static char DrivingModeTune[128];
  strcpy(DrivingModeTune, "DrivingModeTune:d=32,o=5,b=140:");
  for (int dm = 0; dm <= drivingMode; dm++) { 
    strcat(DrivingModeTune, "c5,P");   
    if (dm != drivingMode) {
      strcat(DrivingModeTune, ",");  
    }
  }
  playTune(DrivingModeTune);
}


void initLEDs() {
  if (nrOfLEDStrips == 0) return;
  // without delay the strip does not initialize correctly
  delay(1);
  strip = new NeoPixelBus<PIXEL_COLOR_FEATURE, PIXEL_T_METHOD>(nrOfLEDStrips*NR_OF_PIXELS_PER_LEDSTRIP, LED_PIN);
  strip->Begin();
  for (unsigned int i=0; i<strip->PixelCount(); i++){
      strip->SetPixelColor(i, grey);
   }  
   strip->Show();
}


void updateLEDs() {
  if (nrOfLEDStrips == 0) return;
  static bool state = true;
  unsigned int nrOfSteerIndicatorPixels = (nrOfLEDStrips == 2) ? 4 : 2;

  if (signal_detected) {
    RgbColor throttleStateColor = speed_input > 3 ? green : speed_input <-3 ? white : red;
    for (unsigned int i=0; i<strip->PixelCount(); i++){
      strip->SetPixelColor(i, throttleStateColor);
    }  

    RgbColor steeringStateColor = state ? orange : black;
    if (steer_input < -3) {
      for (unsigned int i=0; i<strip->PixelCount(); i++){
        if (i >= nrOfSteerIndicatorPixels) {
          strip->SetPixelColor(i, throttleStateColor);
        } else {
          strip->SetPixelColor(i, steeringStateColor);          
        }
      }  
    }

    if (steer_input > 3) {
      for (unsigned int i=0; i<strip->PixelCount(); i++){
        if (i < strip->PixelCount()-nrOfSteerIndicatorPixels) {
          strip->SetPixelColor(i, throttleStateColor);
        } else {
          strip->SetPixelColor(i, steeringStateColor);          
        }
      }  
    }
  } else {
    RgbColor disArmedStateColor = state ? blue : black;
    for (unsigned int i=0; i<strip->PixelCount(); i++){
      strip->SetPixelColor(i, disArmedStateColor);
    }     
  }

  state = !state;

  strip->Show();    
} 


void print_signals() {
  Serial.print("voltage:");
  Serial.print(voltage);

  Serial.print("\ttimer:");
  Serial.print(signal_micros_timer_value);
  
  Serial.print("\tSteer:");
  Serial.print(channel[0]);
  
  Serial.print("\tThrottle:");
  Serial.print(channel[1]);

  Serial.print("\tSwitch:");
  Serial.print(channel[2]);

  if (signal_detected == false) {
    Serial.print("\tno signal");
  }  
  Serial.println();
}


void print_calculated_values() {
  Serial.print("voltage:");  
  Serial.print(voltage);
  Serial.print(" ; steer_input:");
  Serial.print(steer_input);  
  Serial.print(" ; speed_input:");
  Serial.print(speed_input);
  Serial.print(" ; leftSpeedSteer:");
  Serial.print(leftSpeedSteer);
  Serial.print(" ; rightSpeedSteer:");
  Serial.print(rightSpeedSteer);    
  Serial.print(" ; steerServo:");
  Serial.print(steerServo);      
  Serial.print(" ; leftEscs:");
  Serial.print(leftEscs);
  Serial.print(" ; rightEscs:");
  Serial.println(rightEscs);        
}


void getGPSData() {
  if (!hasGPSSensor) return;

  while (hs.available() > 0) {
    bool positionIsAccurate = false;
    if (gps.encode(hs.read()))
    {
      if (gps.satellites.isValid()) {
        int nrOfSatellites = gps.satellites.value();
        positionIsAccurate = (nrOfSatellites >= MIN_GPS_SATELLITES) ? true : false;
        sat_str = String(nrOfSatellites);
      } else {
        sat_str = "";
      }

      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        lat_str = String(latitude , 6);
        longitude = gps.location.lng();
        lng_str = String(longitude , 6);
        if (!startingPointFound) {
          startingPointFound = true;
          prev_latitude = latitude;
          prev_longitude = longitude;
        } else {
          double distance = abs(gps.distanceBetween(latitude, longitude, prev_latitude, prev_longitude));
          if ((distance > MIN_GPS_DISTANCE) && (positionIsAccurate)) {
            totalDistance += distance/1000.0;
            prev_latitude = latitude;
            prev_longitude = longitude;
          }
          total_distance_str = String(totalDistance, 3);          
        }
      } else {
        latitude = 0.0f;
        lat_str = "";
        longitude = 0.0f;
        lng_str = "";
      }

      if (gps.date.isValid())
      {
        date_str = "";
        int day = gps.date.day();
        int month = gps.date.month();
        int year = gps.date.year();

        if (day < 10)
          date_str = '0';
        date_str += String(day);

        date_str += "/";

        if (month < 10)
          date_str += '0';
        date_str += String(month);

        date_str += "/";

        if (year < 10)
          date_str += '0';
        date_str += String(year);
      } else {
        date_str = "";
      }

      if (gps.time.isValid())
      {
        time_str = "";
        int hour = gps.time.hour();
        int minute = gps.time.minute();
        int second = gps.time.second();

        if (hour < 10)
          time_str = '0';
        time_str += String(hour);

        time_str += ":";

        if (minute < 10)
          time_str += '0';
        time_str += String(minute);

        time_str += ":";

        if (second < 10)
          time_str += '0';
        time_str += String(second);
      } else {
        time_str = "";
      }

      if (gps.speed.isValid()) {
          double speed = gps.speed.kmph();
          if (speed < MIN_GPS_SPEED) speed = 0.0;
          speed_str = String(speed, 0);
          if ((speed > maxSpeed) && (speed >= MIN_GPS_SPEED) && (positionIsAccurate)) {
            maxSpeed = speed;
          }
          max_speed_str = String(maxSpeed, 0);
      } else {
        speed_str = "";
      }
    }
  }

  if ((millis() > 5000) && (gps.charsProcessed() < 10)) {
//    Serial.println("** No characters received from GPS: check wiring **");
  }
}


void printString(String prmValue, String prmTitle, String prmUnit)
{
  char buf[64];
  sprintf(buf, "%s : %4s %s", prmTitle.c_str(), prmValue.c_str(), prmUnit.c_str()); 
  Serial.println(buf); 
}


void printInt(int32_t prmValue, String prmTitle, String prmUnit)
{
  char buf[64];
  sprintf(buf, "%s : %6d %s", prmTitle.c_str(), prmValue, prmUnit.c_str()); 
  Serial.println(buf); 
}


String getBSSID(uint8_t* prmStrongestBssid) {
  if (prmStrongestBssid == NULL) {
    return "";
  }
  static char BSSID_char[18];
  for (int i = 0; i < 6; ++i){
    sprintf(BSSID_char,"%s%02x:",BSSID_char,prmStrongestBssid[i]);
  } 
  BSSID_char[17] = '\0';
  return String(BSSID_char);
}


uint8_t* getChannelWithStrongestSignal(String prmSSID[], int prmNrOfSSIDs, int32_t *prmStrongestChannel, int *prmStrongestSSIDIndex) {
  Serial.println("getChannelWithStrongestSignal ...............");
  byte available_networks = WiFi.scanNetworks();

  uint8_t* strongestBssid = NULL;
  int32_t rssiMax = -2147483648;
  *prmStrongestChannel = -1;
  *prmStrongestSSIDIndex = -1;
  for (int network = 0; network < available_networks; network++) {
    for (int i = 0; i < prmNrOfSSIDs; i++) {
      if (WiFi.SSID(network).equalsIgnoreCase(prmSSID[i])) {
        if (WiFi.RSSI(network) > rssiMax) {
          rssiMax = WiFi.RSSI(network);
          strongestBssid = WiFi.BSSID(network);
          *prmStrongestSSIDIndex = i;
          *prmStrongestChannel = WiFi.channel(network);
        }
      }
    }    
  }

  printInt(rssiMax, "rssiMax", "dB");          
  printInt(*prmStrongestChannel, "StrongestChannel", "");
  printInt(*prmStrongestSSIDIndex, "StrongestSSIDIndex", "");
  if (*prmStrongestSSIDIndex != -1) {
    printString(prmSSID[*prmStrongestSSIDIndex], "StrongestSSID", "");  
  }
  printString(getBSSID(strongestBssid), "StrongestBssid", "");  
  Serial.println();

  return strongestBssid;
}


void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info)
{
  APStarted = true;
  Serial.println("AP Started");
}


bool isAPStarted() {
  return APStarted;
}


String getIdFromName(String prmName) {
  String id = prmName;
  id.replace(" ", "_");
  id.replace("[", "");
  id.replace("]", "");
  id.replace("(", "");
  id.replace(")", "");
  id.replace("/", "");
  return id;
}


String addDQuotes(String prmValue) {
  return "\"" + prmValue + "\"";
}


String addRow(String prmName, bool prmIsVisible, bool prmIsEditableCell, bool prmIsHeader, String prmValue = "") {
  String visibleStyle = prmIsVisible ? "" : INVISIBLE_STYLE;
  String editableCell = prmIsEditableCell ? "contenteditable='true'" : "";
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:20%\" ALIGN=CENTER>" + prmName + suffix;
  String col2 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(prmName)) + " style=\"width:20%\" ALIGN=CENTER>" + prmValue + suffix;
  return "<tr " + visibleStyle + ">" + col1 + col2 + "</tr>";
}


String addRow(String name, bool prmIsHeader, String prmName1, String prmName2, String prmName3, String prmName4, String prmName5, String prmName6, String prmName7, String prmName8, String prmName9, String prmName10) {
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:9%\" ALIGN=CENTER>" + name + suffix;
  String col2, col3, col4, col5, col6, col7, col8, col9, col10, col11;
  if (prmIsHeader) {
    col2 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName1 + suffix;
    col3 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName2 + suffix;
    col4 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName3 + suffix;
    col5 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName4 + suffix;
    col6 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName5 + suffix;  
    col7 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName6 + suffix;  
    col8 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName7 + suffix;  
    col9 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName8 + suffix;  
    col10 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName9 + suffix;  
    col11 = prefix + " style=\"width:9%\" ALIGN=CENTER>" + prmName10 + suffix;      
  } else {
    col2 = prefix + " id=" + addDQuotes(getIdFromName(prmName1)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col3 = prefix + " id=" + addDQuotes(getIdFromName(prmName2)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col4 = prefix + " id=" + addDQuotes(getIdFromName(prmName3)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col5 = prefix + " id=" + addDQuotes(getIdFromName(prmName4)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col6 = prefix + " id=" + addDQuotes(getIdFromName(prmName5)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col7 = prefix + " id=" + addDQuotes(getIdFromName(prmName6)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col8 = prefix + " id=" + addDQuotes(getIdFromName(prmName7)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col9 = prefix + " id=" + addDQuotes(getIdFromName(prmName8)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col10 = prefix + " id=" + addDQuotes(getIdFromName(prmName9)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
    col11 = prefix + " id=" + addDQuotes(getIdFromName(prmName10)) + " style=\"width:9%\" ALIGN=CENTER>" + suffix;
  }
  return "<tr>" + col1 + col2 + col3 + col4 + col5 + col6 + col7 + col8 + col9 + col10 + col11 + "</tr>";
}


String addRow2(String name, String value1, String value2, String value3, String value4, bool prmIsEditableCell, bool prmIsHeader) {
  String editableCell = prmIsEditableCell ? "contenteditable='true'" : "";
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:20%\" ALIGN=CENTER>" + name + suffix;
  String col2 = prefix + editableCell + " id=" + addDQuotes(name + "1") + " style=\"width:20%\" ALIGN=CENTER>" + value1 + suffix;
  String col3 = prefix + editableCell + " id=" + addDQuotes(name + "2") + " style=\"width:20%\" ALIGN=CENTER>" + value2 + suffix;
  String col4 = prefix + editableCell + " id=" + addDQuotes(name + "3") + " style=\"width:20%\" ALIGN=CENTER>" + value3 + suffix;
  String col5 = prefix + editableCell + " id=" + addDQuotes(name + "4") + " style=\"width:20%\" ALIGN=CENTER>" + value4 + suffix;
  return "<tr>" + col1 + col2 + col3 + col4 + col5 + "</tr>";
}


String toString(bool prmValue) {
  if (prmValue) return "True";
  return "False";
}


String toString(int prmValue) {
  return String(prmValue);
}


String toString(unsigned long prmValue) {
  return String(prmValue);
}


String toString(double prmValue) {
  return String(prmValue, 2);
}


String getGenericProgressStr(String prmID, String prmSpanID, int prmValueMin, int prmValueMax) {
  return "<div class=\"progress\"><div id=\"" + prmID + "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"" + String(prmValueMin) + "\"  aria-valuemax=\"" + String(prmValueMax) + "\" style=\"width:0%\"><span id=\"" + prmSpanID + "\" ></span></div></div>";
}


String getVoltageProgressStr() {
  return "<div class=\"progress\"><div id=\"" ID_PROGRESS_VOLTAGE "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">0%</div></div>";
}


String getCurrentProgressStr() {
  return "<div class=\"progress\"><div id=\"" ID_PROGRESS_CURRENT "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">0%</div></div>";
}


String getChannelProgressStr(String prmChannelDivID, String prmChannelSpanID, String prmColor) {
  return "<div class=\"progress\"><div id=\"" + prmChannelDivID + "\" class=\"progress-bar " + prmColor + "\"" + " role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"800\" aria-valuemax=\"2200\" style=\"width:0%\"><span id=\"" + prmChannelSpanID + "\" ></span></div></div>";
}


String getLoopTimeProgressStr(String prmLoopTimeID, String prmLoopTimeSpanID, int prmLoopTime) {
  return "<div class=\"progress\"><div id=\"" + prmLoopTimeID + "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"" + String(prmLoopTime) + "\" style=\"width:0%\"><span id=\"" + prmLoopTimeSpanID + "\" ></span></div></div>";
}


String getHtmlHeader() {
  String s = "";
  s += "<head>";
  s += "  <meta><title>WebService RaceCar: " + robotName + " </title>";

  s += "  <style>";
  s += "  .tab {";
  s += "    margin:auto;";
  s += "    width:50%;";
  s += "    overflow: hidden;";
  s += "    border: 1px solid #ccc;";
  s += "    background-color: #f1f1f1;";
  s += "  }";
  s += "  .tab button {";
  s += "    font-family: Helvetica;";
  s += "    font-size: 20px;";
  s += "    background-color: inherit;";
  s += "    float: left;";
  s += "    border: none;";
  s += "    outline: none;";
  s += "    cursor: pointer;";
  s += "    padding: 14px 16px;";
  s += "    transition: 0.3s;";
  s += "  }";
  s += "  .tab button:hover {";
  s += "    background-color: #ddd;";
  s += "  }";
  s += "  .tab button.active {";
  s += "    background-color: #ccc;";
  s += "  }";
  s += "  </style>";

  s += "  <style>";
  s += "    table, th, td {border: 1px solid black; border-collapse: collapse;}";
  s += "    th { height: " ROW_HEIGHT_TH "; font-size: " FONT_SIZE_TH ";}";
  s += "    td { height: " ROW_HEIGHT_TD "; font-size: " FONT_SIZE_TD ";}";
  s += "    tr:nth-child(even) { background-color: #eee }";
  s += "    tr:nth-child(odd) { background-color: #fff;}";
  s += "    td:first-child { background-color: lightgrey; color: black;}";
  s += "    th { background-color: lightgrey; color: black;}";
  s += "  </style>";  

  s += "  <style>";
  s += "    html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
  s += "    .button { border-radius: 12px; background-color: grey; border: none; color: white; padding: 16px 40px;";
  s += "    text-decoration: none; font-size:" FONT_SIZE_BUTTON "; margin: 10px; cursor: pointer;}";
  s += "    .progress-bar{float:left;width:0%;height:100%;font-size:" FONT_SIZE_TD ";line-height:" LINE_HEIGHT_TD ";color:#fff;text-align:center;background-color:#337ab7;-webkit-box-shadow:inset 0 -1px 0 rgba(0,0,0,.15);box-shadow:inset 0 -1px 0 rgba(0,0,0,.15);-webkit-transition:width .6s ease;-o-transition:width .6s ease;transition:width .6s ease}";
  s += "    .progress-bar-success{background-color:#5cb85c}";
  s += "    .progress-bar-warning{background-color:#f0ad4e}";
  s += "    .progress-bar-danger{background-color:#d9534f}";
  s += "    .progress-bar-ch1{background-color:#cc0000}";
  s += "    .progress-bar-ch2{background-color:purple}";
  s += "    .progress-bar-ch3{background-color:#0066cc}";
  s += "    .progress-bar-ch4{background-color:#2eb8b8}";
  s += "    .progress-bar-ch5{background-color:#26734d}";
  s += "    .progress-bar-ch6{background-color:#2eb82e}";
  s += "  </style>";

  s += "  <style>";
  s += "    .progress {";
  s += "        position: relative;";
  s += "        height:" ROW_HEIGHT_TD ";";
  s += "    }";
  s += "    .progress span {";
  s += "        position: absolute;";
  s += "        display: block;";
  s += "        width: 100%;";
  s += "        color: black;";
  s += "     }";
  s += "  </style>";  

  s += "  <style>";
  s += "    .btn-group button {";
  s += "        width: 140px;";
  s += "        height: 32px;";
  s += "        white-space: nowrap;";
  s += "        text-align:center;"; 
  s += "        vertical-align:middle;";
  s += "        padding: 0px;";
  s += "    }";
  s += "  </style>"; 
  
  s += getSpeedoMeterStyle(); 
  
  s += "</head>";
  return s;
}


String getScript(String prmToBeClickedTabButton) {
  String s = "";
  s += "<script>";

  s += getSpeedoMeterFunctions(); 
  s += "var theSpeedoMeter = new speedometer({maxVal:" + String(SPEEDOMETER_MAX_SPEED) + "});";
  s += "document.getElementById('" +  getIdFromName(NAME_SPEEDOMETER_GAUGE) + "').append(theSpeedoMeter.elm);";

  s += "document.getElementById(\"" + getIdFromName(prmToBeClickedTabButton) + "\").click();";
  s += "var errorCounter = 0;";
  s += "var firstTimeOutOfSync = true;";
  s += "var sendCounter = 0;";
  s += "var receiveCounter = 0;";
  s += "var requestSendTime = getTimeMS();";
  s += "var responseReceiveTime = requestSendTime;";  
  s += "requestData();";
    
  s += "var timerId = setInterval(requestData, " TELEMETRY_REFRESH_INTERVAL ");";
  s += "var isAliveTimerId = setInterval(updateResponseTimeData, " IS_ALIVE_REFRESH_INTERVAL ");";

  s += "function map(x, in_min, in_max, out_min, out_max) {";
  s += "  run = in_max - in_min;";
  s += "  if (run == 0) {";
  s += "    return -1;";
  s += "  }";
  s += "  rise = out_max - out_min;";
  s += "  delta = x - in_min;";
  s += "  return (delta * rise) / run + out_min;";
  s += "}";

  s += "function getResponseTimeColorMsg(prmResponseTime) {";  
  s += "  if (prmResponseTime > " + String(BAD_RESPONSE_TIME) + ") {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  } else if (prmResponseTime > " + String(WARNING_RESPONSE_TIME) + ") {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  }";  
  s += "}";

  s += "function getResponseTimePercentage(prmResponseTime) {";
  s += "  var percentage = map(prmResponseTime," + String(MIN_RESPONSE_TIME) + "," + String(MAX_RESPONSE_TIME) + ",0,100);";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getWiFiSignalStrengthColorMsg(prmWiFiSignalStrength) {";  
  s += "  if (prmWiFiSignalStrength < " + String(LOW_SIGNAL_STRENGTH) + ") {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  } else if (prmWiFiSignalStrength < " + String(WARNING_SIGNAL_STRENGTH) + ") {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  }";  
  s += "}";

  s += "function getWiFiSignalStrengthPercentage(prmWiFiSignalStrength) {";
  s += "  var percentage = map(prmWiFiSignalStrength," + String(MIN_SIGNAL_STRENGTH) + "," + String(MAX_SIGNAL_STRENGTH) + ",0,100);";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getVoltageColorMsg(prmVoltage) {";  
  s += "  if (prmVoltage < " + String(LOW_VOLTAGE_ALARM, 2) + ") {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  } else if (prmVoltage <" + String(WARNING_VOLTAGE, 2) + ") {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  }";  
  s += "}";

  s += "function getVoltagePercentage(prmVoltage) {";
  s += "  var percentage = 100.0*((prmVoltage - " + String(LOW_VOLTAGE_ALARM, 2) + ")/" + String(FULLY_CHARGED_VOLTAGE - LOW_VOLTAGE_ALARM, 2) + ");";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getCurrentColorMsg(prmCurrent) {";  
  s += "  if (prmCurrent < " + String(SAFE_CURRENT, 2) + ") {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  } else if (prmCurrent <" + String(WARNING_CURRENT, 2) + ") {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  }";  
  s += "}";

  s += "function getCurrentPercentage(prmCurrent) {";
  s += "  var percentage = 100.0*(prmCurrent/" + String(FULL_SCALE_CURRENT, 2) + ");";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getChannelPercentage(prmPulse) {";
  s += "  var percentage = 100.0*((prmPulse - 800)/(2200-800));";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function updateResponseTime(prmDivProgressID, prmSpanProgressID, prmValue) {";
  s += "  var divProgressElem = document.getElementById(prmDivProgressID);";
  s += "  var spanProgressElem = document.getElementById(prmSpanProgressID);";
  s += "  var responseTime = parseInt(prmValue);";
  s += "  divProgressElem.setAttribute(\"class\", getResponseTimeColorMsg(responseTime));";
  s += "  divProgressElem.setAttribute(\"aria-valuenow\", responseTime);";
  s += "  divProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getResponseTimePercentage(responseTime)).toFixed(0) + \"%\");";
  s += "  spanProgressElem.innerText = responseTime;";
  s += "}";

  s += "function updateWifiSignalStrength(prmDivProgressID, prmSpanProgressID, prmValue) {";
  s += "  var divProgressElem = document.getElementById(prmDivProgressID);";
  s += "  var spanProgressElem = document.getElementById(prmSpanProgressID);";
  s += "  var wiFiSignalStrength = parseInt(prmValue);";
  s += "  divProgressElem.setAttribute(\"class\", getWiFiSignalStrengthColorMsg(wiFiSignalStrength));";
  s += "  divProgressElem.setAttribute(\"aria-valuenow\", wiFiSignalStrength);";
  s += "  divProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getWiFiSignalStrengthPercentage(wiFiSignalStrength)).toFixed(0) + \"%\");";
  s += "  spanProgressElem.innerText = wiFiSignalStrength;";
  s += "}";

  s += "function updateChannel(prmDivProgressID, prmSpanProgressID, prmValue) {";
  s += "  var divProgressElem = document.getElementById(prmDivProgressID);";
  s += "  var spanProgressElem = document.getElementById(prmSpanProgressID);";
  s += "  var value = parseInt(prmValue);";
  s += "  divProgressElem.setAttribute(\"aria-valuenow\", value);";
  s += "  divProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getChannelPercentage(value)).toFixed(0) + \"%\");";
  s += "  spanProgressElem.innerText = value;";
  s += "}";

  s += "function updateLoopTime(prmDivProgressID, prmSpanProgressID, prmLoopTime, prmValue) {";
  s += "  var divProgressElem = document.getElementById(prmDivProgressID);";
  s += "  var spanProgressElem = document.getElementById(prmSpanProgressID);";
  s += "  var usedUpLoopTime = parseInt(prmValue);";
  s += "  divProgressElem.setAttribute(\"class\", getLoopTimeColorMsg(usedUpLoopTime, prmLoopTime));";
  s += "  divProgressElem.setAttribute(\"aria-valuenow\", usedUpLoopTime);";
  s += "  divProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getLoopTimePercentage(usedUpLoopTime, prmLoopTime)).toFixed(0) + \"%\");";
  s += "  spanProgressElem.innerText = usedUpLoopTime;";
  s += "}";

  s += "function getLoopTimePercentage(prmUsedUpLoopTime, prmLoopTime) {";
  s += "  var percentage = 100*prmUsedUpLoopTime/prmLoopTime;";
  s += "  if (percentage > 100.0) {";
  s += "    percentage = 100.0;";
  s += "  }";
  s += "  if (percentage < 0.0) {";
  s += "    percentage = 0.0;";
  s += "  }";
  s += "  return percentage;";
  s += "}";

  s += "function getLoopTimeColorMsg(prmUsedUpLoopTime, prmLoopTime) {"; 
  s += "  var percentage = getLoopTimePercentage(prmUsedUpLoopTime, prmLoopTime);";
  s += "  if (percentage > 75) {";
  s += "    return \"progress-bar progress-bar-danger\";";
  s += "  } else if (percentage > 50) {";
  s += "    return \"progress-bar progress-bar-warning\";";
  s += "  } else {";
  s += "    return \"progress-bar progress-bar-success\";";
  s += "  }";  
  s += "}";

  s += "function getTimeMS() {";
  s += "  var date = new Date();";
  s += "  return date.getTime();";
  s += "}";

  s += "function updateResponseTimeData() {";
  s += "  var responseTime =  responseReceiveTime-requestSendTime;";
  s += "  if (responseTime < 0) {";
  s += "    responseTime =  getTimeMS()-responseReceiveTime;";
  s += "  }";
  s += "  updateResponseTime(\"" + getIdFromName(ID_PROGRESS_RESP_TIME) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_RESP_TIME) + "\", responseTime);";
  s += "}";

  s += "function IsReadyToSendNextRequest() {";
  s += "  if (sendCounter == receiveCounter) {"; 
  s += "    firstTimeOutOfSync = true;";
  s += "    errorCounter = 0;";  
  s += "    return true;";
  s += "  } else {";
  s += "    if (firstTimeOutOfSync) {";
  s += "      firstTimeOutOfSync = false;";  
  s += "      return false;";
  s += "    } else {";  
  s += "      if (errorCounter > 10) {";
  s += "        firstTimeOutOfSync = true;";  
  s += "        errorCounter = 0;";
  s += "        sendCounter = 0;";
  s += "        receiveCounter = 0;";
  s += "        return false;";
  s += "      } else {";
  s += "        errorCounter++;";  
  s += "        return false;";
  s += "      }";  
  s += "    }";  
  s += "  }";
  s += "}";  

  s += "function requestData() {";  
  s += "  if (!IsReadyToSendNextRequest()) {";
  s += "    return;";
  s += "  }";  

  s += "  var xhr = new XMLHttpRequest();";
  s += "  xhr.open(\"GET\", \"/RequestLatestData\", true);";  
  s += "  xhr.timeout = (" TELEMETRY_RECEIVE_TIMEOUT ");";  
  s += "  xhr.onload = function() {";
  s += "    receiveCounter++;";
  s += "    responseReceiveTime = getTimeMS();";
  s += "    if (xhr.status == 200) {";
  s += "      if (xhr.responseText) {";
  s += "        var data = JSON.parse(xhr.responseText);";
  s += "        var parser = new DOMParser();";

  s += "        var speed = data." + getIdFromName(NAME_SPEEDOMETER_GAUGE) + ";";
  s += "        theSpeedoMeter.setPosition(speed);";

  s += "        updateWifiSignalStrength(\"" + getIdFromName(ID_PROGRESS_WIFI_SS) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_WIFI_SS) + "\"," + "data." + getIdFromName(NAME_WIFI_SIGNAL_STRENGTH) + ");";

  s += "        document.getElementById(\"" + getIdFromName(NAME_SIGNAL_DETECTED) + "\").innerText = data." + getIdFromName(NAME_SIGNAL_DETECTED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ARMED) + "\").innerText = data." + getIdFromName(NAME_ARMED) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_DRIVING_MODE) + "\").innerText = data." + getIdFromName(NAME_DRIVING_MODE) + ";";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_1) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_1) + "\"," + "data." + getIdFromName(NAME_CHANNEL_1) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_2) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_2) + "\"," + "data." + getIdFromName(NAME_CHANNEL_2) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_3) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_3) + "\"," + "data." + getIdFromName(NAME_CHANNEL_3) + ");";

  s += "        document.getElementById(\"" + getIdFromName(NAME_VOLTAGE) + "\").innerText = data." + getIdFromName(NAME_VOLTAGE) + ";";  
  s += "        var voltageProgressElem = document.getElementById(\"" ID_PROGRESS_VOLTAGE "\");";
  s += "        var voltage = parseFloat(data." + getIdFromName(NAME_VOLTAGE) + ");";
  s += "        voltageProgressElem.setAttribute(\"class\", getVoltageColorMsg(voltage));";
  s += "        voltageProgressElem.setAttribute(\"aria-valuenow\", parseFloat(getVoltagePercentage(voltage)).toFixed(0));";
  s += "        voltageProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getVoltagePercentage(voltage)).toFixed(0) + \"%\");";
  s += "        voltageProgressElem.innerText = parseFloat(getVoltagePercentage(voltage)).toFixed(0) + \"%\";";

  s += "        document.getElementById(\"" + getIdFromName(NAME_CURRENT) + "\").innerText = data." + getIdFromName(NAME_CURRENT) + ";";  
  s += "        var currentProgressElem = document.getElementById(\"" ID_PROGRESS_CURRENT "\");";
  s += "        var current = parseFloat(data." + getIdFromName(NAME_CURRENT) + ");";
  s += "        currentProgressElem.setAttribute(\"class\", getCurrentColorMsg(current));";
  s += "        currentProgressElem.setAttribute(\"aria-valuenow\", parseFloat(getCurrentPercentage(current)).toFixed(0));";
  s += "        currentProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getCurrentPercentage(current)).toFixed(0) + \"%\");";
  s += "        currentProgressElem.innerText = parseFloat(getCurrentPercentage(current)).toFixed(0) + \"%\";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_STEER_SERVO) + "\").innerText = data." + getIdFromName(NAME_STEER_SERVO) + ";";    
  s += "        document.getElementById(\"" + getIdFromName(NAME_LEFT_ESCS) + "\").innerText = data." + getIdFromName(NAME_LEFT_ESCS) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_RIGHT_ESCS) + "\").innerText = data." + getIdFromName(NAME_RIGHT_ESCS) + ";";    

  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_X) + "\").innerText = data." + getIdFromName(NAME_GYRO_X) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_Y) + "\").innerText = data." + getIdFromName(NAME_GYRO_Y) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_Z) + "\").innerText = data." + getIdFromName(NAME_GYRO_Z) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ACC_X) + "\").innerText = data." + getIdFromName(NAME_ACC_X) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ACC_Y) + "\").innerText = data." + getIdFromName(NAME_ACC_Y) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ACC_Z) + "\").innerText = data." + getIdFromName(NAME_ACC_Z) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_TEMPERATURE) + "\").innerText = data." + getIdFromName(NAME_TEMPERATURE) + ";";

  s += "        updateLoopTime(\"" + getIdFromName(ID_PROGRESS_LOOPTIME_1) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_LOOPTIME_1) + "\"," + String(LOOP_TIME_TASK1) + "," + "data." + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_1) + ");";
  s += "        updateLoopTime(\"" + getIdFromName(ID_PROGRESS_LOOPTIME_2) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_LOOPTIME_2) + "\"," + String(LOOP_TIME_TASK2) + "," + "data." + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_2) + ");";
  s += "        updateLoopTime(\"" + getIdFromName(ID_PROGRESS_LOOPTIME_3) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_LOOPTIME_3) + "\"," + String(LOOP_TIME_TASK3) + "," + "data." + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_3) + ");";
  s += "        updateLoopTime(\"" + getIdFromName(ID_PROGRESS_LOOPTIME_4) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_LOOPTIME_4) + "\"," + String(LOOP_TIME_TASK4) + "," + "data." + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_4) + ");";

  s += "        document.getElementById(\"" + getIdFromName(NAME_SATELLITES) + "\").innerText = data." + getIdFromName(NAME_SATELLITES) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_LATITUDE) + "\").innerText = data." + getIdFromName(NAME_LATITUDE) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_LONGITUDE) + "\").innerText = data." + getIdFromName(NAME_LONGITUDE) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_DATE) + "\").innerText = data." + getIdFromName(NAME_DATE) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_TIME) + "\").innerText = data." + getIdFromName(NAME_TIME) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_SPEED) + "\").innerText = data." + getIdFromName(NAME_SPEED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_MAX_SPEED) + "\").innerText = data." + getIdFromName(NAME_MAX_SPEED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_TOTAL_DISTANCE) + "\").innerText = data." + getIdFromName(NAME_TOTAL_DISTANCE) + ";";

  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_YAW_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_YAW_ACC) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_YAW) + "\").innerText = data." + getIdFromName(NAME_ANGLE_YAW) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_YAW_LEVEL_ADJUST) + "\").innerText = data." + getIdFromName(NAME_YAW_LEVEL_ADJUST) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_ROLL_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_ROLL_INPUT) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_PITCH_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_PITCH_INPUT) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_YAW_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_YAW_INPUT) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_YAW_SETPOINT) + "\").innerText = data." + getIdFromName(NAME_PID_YAW_SETPOINT) + ";"; 

  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_ERROR) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_P) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_I) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_D) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW) + ";"; 

  s += "        document.getElementById(" + addDQuotes(ID_BUZZER_BUTTON) + ").innerText = data." + ID_BUZZER_BUTTON + ";"; 

  s += "      }";  
  s += "    }";
  s += "  };";

  s += "  requestSendTime = getTimeMS();"; 
  s += "  sendCounter++;";
  s += "  xhr.send();";  
  s += "}";
  
  s += "function getNameValue(prmName) {";
  s += "  var value = document.getElementById(prmName).innerHTML;";
  s += "  return prmName + \"=\" + value;";
  s += "}";

  s += "function savePropValues() {";
  s += "  clearInterval(timerId);";
  s += "  var xhr = new XMLHttpRequest();";
  s += "  var yawValues = getNameValue(\"Yaw1\") + \"&\" + getNameValue(\"Yaw2\") + \"&\" + getNameValue(\"Yaw3\") + \"&\" + getNameValue(\"Yaw4\");";
  s += "  var steerExpoFactor =  getNameValue(\"" + getIdFromName(NAME_STEER_EXPO) + "\");";
  s += "  var steerServoCenterOffset =  getNameValue(\"" + getIdFromName(NAME_STEER_SERVO_CENTER_OFFSET) + "\");";
  s += "  var speedEscCenterOffset =  getNameValue(\"" + getIdFromName(NAME_SPEED_ESC_CENTER_OFFSET) + "\");";
  s += "  var voltageCorrectionFactor =  getNameValue(\"" + getIdFromName(NAME_VOLTAGE_CORRECTION) + "\");";
  s += "  var currentCorrectionFactor =  getNameValue(\"" + getIdFromName(NAME_CURRENT_CORRECTION) + "\");";
  s += "  var calibrated_accX = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCX) + "\");";
  s += "  var calibrated_accY = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCY) + "\");";  
  s += "  var calibrated_accZ = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCZ) + "\");";  
  s += "  xhr.open(\"GET\", \"/Save?\" + yawValues + \"&\" + steerExpoFactor + \"&\" + steerServoCenterOffset + \"&\" + speedEscCenterOffset + \"&\" + voltageCorrectionFactor + \"&\" + currentCorrectionFactor + \"&\" + calibrated_accX + \"&\" + calibrated_accY + \"&\" + calibrated_accZ, false);";  
  s += "  xhr.send();";
  s += "  location.reload();";
  s += "}";

  s += "function selectTab(evt, prmTabId) {";
  s += "  var i, tabcontent, tablinks;";
  s += "  tabcontent = document.getElementsByClassName(\"tabcontent\");";
  s += "  for (i = 0; i < tabcontent.length; i++) {";
  s += "    tabcontent[i].style.display = \"none\";";
  s += "  }";
  s += "  tablinks = document.getElementsByClassName(\"tablinks\");";
  s += "  for (i = 0; i < tablinks.length; i++) {";
  s += "    tablinks[i].className = tablinks[i].className.replace(\" active\", "");";
  s += "  }";
  s += "  document.getElementById(prmTabId).style.display = \"block\";";
  s += "  evt.currentTarget.className += \" active\";";
  s += "}";

  s += "</script>";
  
  return s;
}


String getDrivingModeSt() {
  switch (drivingMode) {
    case dmHalfSpeed:
      return "Half Speed";
    case dmFullSpeed:
      return "Full Speed";
    default:
      return "None";
  }
}


String getBuzzerCaption() {
  String st = "Buzzer ";
  st += buzzerOff ? "On" : "Off";
  return st;
}


String getWebPage(String prmToBeClickedTabButton) {
  String s = "<!DOCTYPE html><html>";
  s += getHtmlHeader();

  String gpsStyle = hasGPSSensor ? "" : INVISIBLE_STYLE;

  s += "<body>";

  s += "<div class=\"tab\">";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_TELEMETRY) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_TELEMETRY) + "\">" NAME_TAB_TELEMETRY "</button>";
  s += "<button " + gpsStyle + " class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_GPS) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_GPS) + "\">" NAME_TAB_GPS "</button>";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_SPEEDOMETER) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_SPEEDOMETER) + "\">" NAME_TAB_SPEEDOMETER "</button>";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_SETTINGS) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_SETTINGS) + "\">" NAME_TAB_SETTINGS "</button>";
  s += "</div>";


  s += "<div id=\"" NAME_TAB_TELEMETRY "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_MODEL, true, false, true, robotName);
  s += addRow(NAME_VERSION, true, false, false, RACECAR_VERSION);  
  s += addRow(NAME_RESPONSE_TIME, true, false, false, getGenericProgressStr(ID_PROGRESS_RESP_TIME, ID_SPAN_PROGRESS_RESP_TIME, MIN_RESPONSE_TIME, MAX_RESPONSE_TIME));
  s += addRow(NAME_WIFI_SIGNAL_STRENGTH, true, false, false, getGenericProgressStr(ID_PROGRESS_WIFI_SS, ID_SPAN_PROGRESS_WIFI_SS, MIN_SIGNAL_STRENGTH, MAX_SIGNAL_STRENGTH));
  s += addRow(NAME_SIGNAL_DETECTED, true, false, false);
  s += addRow(NAME_ARMED, true, false, false);
  s += addRow(NAME_DRIVING_MODE, true, false, false);
  s += addRow(NAME_CHANNEL_1, true, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_1, ID_SPAN_PROGRESS_CHANNEL_1, "progress-bar-ch1"));
  s += addRow(NAME_CHANNEL_2, true, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_2, ID_SPAN_PROGRESS_CHANNEL_2, "progress-bar-ch2"));
  s += addRow(NAME_CHANNEL_3, true, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_3, ID_SPAN_PROGRESS_CHANNEL_3, "progress-bar-ch3"));
  s += addRow(NAME_VOLTAGE, true, false, false);
  s += addRow(NAME_VOLTAGE_PROGRESS, true, false, false, getVoltageProgressStr());
  s += addRow(NAME_CURRENT, hasCurrentSensor, false, false);
  s += addRow(NAME_CURRENT_PROGRESS, hasCurrentSensor, false, false, getCurrentProgressStr());
  s += addRow(NAME_STEER_SERVO, true, false, false);
  s += addRow(NAME_LEFT_ESCS, true, false, false);
  s += addRow(NAME_RIGHT_ESCS, true, false, false);  
  s += addRow(NAME_GYRO_X, true, false, false);
  s += addRow(NAME_GYRO_Y, true, false, false);
  s += addRow(NAME_GYRO_Z, true, false, false);
  s += addRow(NAME_ACC_X, true, false, false);
  s += addRow(NAME_ACC_Y, true, false, false);
  s += addRow(NAME_ACC_Z, true, false, false);
  s += addRow(NAME_TEMPERATURE, true, false, false);
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_1, true, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_1, ID_SPAN_PROGRESS_LOOPTIME_1, LOOP_TIME_TASK1));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_2, true, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_2, ID_SPAN_PROGRESS_LOOPTIME_2, LOOP_TIME_TASK2));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_3, true, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_3, ID_SPAN_PROGRESS_LOOPTIME_3, LOOP_TIME_TASK3));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_4, true, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_4, ID_SPAN_PROGRESS_LOOPTIME_4, LOOP_TIME_TASK4));
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow("", true, "Angle Acc", "Angle", "Level Adjust", "Input", "Setpoint", "Error", "P", "I", "D", "Output");
  s += addRow(NAME_TELEMETRY_ROLL, false, NAME_ANGLE_ROLL_ACC, NAME_ANGLE_ROLL, NAME_ROLL_LEVEL_ADJUST, NAME_GYRO_ROLL_INPUT, NAME_PID_ROLL_SETPOINT, NAME_PID_OUTPUT_ROLL_ERROR, NAME_PID_OUTPUT_ROLL_P, NAME_PID_OUTPUT_ROLL_I, NAME_PID_OUTPUT_ROLL_D, NAME_PID_OUTPUT_ROLL);
  s += addRow(NAME_TELEMETRY_PITCH, false, NAME_ANGLE_PITCH_ACC, NAME_ANGLE_PITCH, NAME_PITCH_LEVEL_ADJUST, NAME_GYRO_PITCH_INPUT, NAME_PID_PITCH_SETPOINT, NAME_PID_OUTPUT_PITCH_ERROR, NAME_PID_OUTPUT_PITCH_P, NAME_PID_OUTPUT_PITCH_I, NAME_PID_OUTPUT_PITCH_D, NAME_PID_OUTPUT_PITCH);
  s += addRow(NAME_TELEMETRY_YAW, false, NAME_ANGLE_YAW_ACC, NAME_ANGLE_YAW, NAME_YAW_LEVEL_ADJUST, NAME_GYRO_YAW_INPUT, NAME_PID_YAW_SETPOINT, NAME_PID_OUTPUT_YAW_ERROR, NAME_PID_OUTPUT_YAW_P, NAME_PID_OUTPUT_YAW_I, NAME_PID_OUTPUT_YAW_D, NAME_PID_OUTPUT_YAW);
  s += "</table>";

  s += "</div>";


  s += "<div id=\"" NAME_TAB_GPS "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_GPS_INFO, true, false, true, "Information");
  s += addRow(NAME_SATELLITES, true, false, false);
  s += addRow(NAME_LATITUDE, true, false, false);
  s += addRow(NAME_LONGITUDE, true, false, false);
  s += addRow(NAME_DATE, true, false, false);
  s += addRow(NAME_TIME, true, false, false);
  s += addRow(NAME_SPEED, true, false, false);
  s += addRow(NAME_MAX_SPEED, true, false, false);
  s += addRow(NAME_TOTAL_DISTANCE, true, false, false);
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<div class=\"btn-group\" style=\"width:100%\">";
  s += "<a href=\"/Zero\"><button type=\"button\" class=\"button\">Zero</button></a>";
  s += "</div>";
  s += "</div>";


  s += "<div id=\"" NAME_TAB_SPEEDOMETER "\" class=\"tabcontent\">";
  s += "<br>";
  s += "<br>";
  s += "<br><br><br><br><br><br><br><br><br><br>";
  s += "<div id=\"" NAME_SPEEDOMETER_GAUGE "\" style=\"scale:" + String(SPEEDOMETER_SCALE, 2) + ";display:flex;justify-content:center;\">";
  s += "</div>";
  s += "</div>";


  s += "<div id=\"" NAME_TAB_SETTINGS "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_STEER_EXPO, true, true, false, String(steerExpoFactor, 2));
  s += addRow(NAME_STEER_SERVO_CENTER_OFFSET, true, true, false, String(steerServoCenterOffset));
  s += addRow(NAME_SPEED_ESC_CENTER_OFFSET, true, true, false, String(speedEscCenterOffset));
  s += addRow(NAME_VOLTAGE_CORRECTION, true, true, false, String(voltageCorrectionFactor, 2));
  s += addRow(NAME_CURRENT_CORRECTION, true, true, false, String(currentCorrectionFactor, 2));
  s += addRow(NAME_CALIBRATED_ACCX, true, true, false, String(mpu6050.getCalibrationAccX(), 2));
  s += addRow(NAME_CALIBRATED_ACCY, true, true, false, String(mpu6050.getCalibrationAccY(), 2));  
  s += addRow(NAME_CALIBRATED_ACCZ, true, true, false, String(mpu6050.getCalibrationAccZ(), 2));  
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow2("PID", "P", "I", "D", "Max", false, true);
  s += addRow2(NAME_PID_SETTINGS_YAW, toString(yawPID.getP()), toString(yawPID.getI()), toString(yawPID.getD()), toString(yawPID.getMax()), true, false);
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<div class=\"btn-group\" style=\"width:100%\">";
  s += "<a href=\"/CalibrateAcc\"><button type=\"button\" style=\"width:180px;\" class=\"button\">Calibrate Acc</button></a>";  
  s += "<a href=\"/BuzzerOnOff\"><button id=" + addDQuotes(ID_BUZZER_BUTTON) + " type=\"button\" style=\"width:160px;\" class=\"button\">" + getBuzzerCaption() + "</button></a>";
  s += "<a href=\"/WifiOff\"><button type=\"button\" class=\"button\">Wifi Off</button></a>";
  s += "</div>";

  s += "<div class=\"btn-group\" style=\"width:100%\">";
  s += "<a><button onclick=\"savePropValues()\" type=\"button\" class=\"button\">Save</button></a>";  
  s += "<a href=\"/Cancel\"><button type=\"button\" class=\"button\">Cancel</button></a>";
  s += "<a href=\"/Defaults\"><button type=\"button\" class=\"button\">Defaults</button></a>";
  s += "</div>";

  s += "</div>";  

  s += getScript(prmToBeClickedTabButton);
  
  s += "</body></html>";
  return s;
}


String getLatestData() {
  String data = "{";

  data += "\"" + getIdFromName(NAME_WIFI_SIGNAL_STRENGTH) + "\":" + addDQuotes(toString(wiFiSignalStrength)) + ",";
  data += "\"" + getIdFromName(NAME_SIGNAL_DETECTED) + "\":" + addDQuotes(toString(signal_detected)) + ",";
  data += "\"" + getIdFromName(NAME_ARMED) + "\":" + addDQuotes(toString(isArmed())) + ",";
  data += "\"" + getIdFromName(NAME_DRIVING_MODE) + "\":" + addDQuotes(getDrivingModeSt()) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_1) + "\":" + addDQuotes(toString(channel[0])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_2) + "\":" + addDQuotes(toString(channel[1])) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_3) + "\":" + addDQuotes(toString(channel[2])) + ",";
  data += "\"" + getIdFromName(NAME_VOLTAGE) + "\":" + addDQuotes(getVoltageStr()) + ",";  
  data += "\"" + getIdFromName(NAME_CURRENT) + "\":" + addDQuotes(getCurrentStr()) + ",";  
  data += "\"" + getIdFromName(NAME_STEER_SERVO) + "\":" + addDQuotes(toString(steerServo)) + ",";  
  data += "\"" + getIdFromName(NAME_LEFT_ESCS) + "\":" + addDQuotes(toString(leftEscs)) + ",";  
  data += "\"" + getIdFromName(NAME_RIGHT_ESCS) + "\":" + addDQuotes(toString(rightEscs)) + ",";  
  data += "\"" + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_1) + "\":" + String(usedUpLoopTimeTask1) + ",";
  data += "\"" + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_2) + "\":" + String(usedUpLoopTimeTask2) + ",";
  data += "\"" + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_3) + "\":" + String(usedUpLoopTimeTask3) + ",";
  data += "\"" + getIdFromName(NAME_USED_UP_LOOPTIME_PROGRESS_4) + "\":" + String(usedUpLoopTimeTask4) + ",";

  data += "\"" + getIdFromName(NAME_GYRO_X) + "\":" + addDQuotes(String(mpu6050.getCalibratedRateRoll())) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_Y) + "\":" + addDQuotes(String(mpu6050.getCalibratedRatePitch())) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_Z) + "\":" + addDQuotes(String(mpu6050.getCalibratedRateYaw())) + ",";
  data += "\"" + getIdFromName(NAME_ACC_X) + "\":" + addDQuotes(String(mpu6050.getCalibratedAccX())) + ",";
  data += "\"" + getIdFromName(NAME_ACC_Y) + "\":" + addDQuotes(String(mpu6050.getCalibratedAccY())) + ",";
  data += "\"" + getIdFromName(NAME_ACC_Z) + "\":" + addDQuotes(String(mpu6050.getCalibratedAccZ())) + ",";
  data += "\"" + getIdFromName(NAME_TEMPERATURE) + "\":" + addDQuotes(String(mpu6050.getTempCelsius(), 1)) + ",";

  data += "\"" + getIdFromName(NAME_ANGLE_ROLL_ACC) + "\":" + addDQuotes(String(mpu6050.getAngleRollAcc(), 0)) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_PITCH_ACC) + "\":" + addDQuotes(String(mpu6050.getAnglePitchAcc(), 0)) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_YAW_ACC) + "\":" + addDQuotes(String(0.0, 0)) + ",";

  data += "\"" + getIdFromName(NAME_ANGLE_ROLL) + "\":" + String(angle_roll, 0) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_PITCH) + "\":" + String(angle_pitch, 0) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_YAW) + "\":" + String(angle_yaw, 0) + ",";

  data += "\"" + getIdFromName(NAME_YAW_LEVEL_ADJUST) + "\":" + String(yaw_level_adjust, 2) + ",";

  data += "\"" + getIdFromName(NAME_GYRO_ROLL_INPUT) + "\":" + String(gyro_roll_input, 2) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_PITCH_INPUT) + "\":" + String(gyro_pitch_input, 2) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_YAW_INPUT) + "\":" + String(gyro_yaw_input, 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_YAW_SETPOINT) + "\":" + String(pid_yaw_setpoint, 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_ERROR) + "\":" + String(yawOutputPID.getError(), 2)+ ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_P) + "\":" + String(yawOutputPID.getP(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_I) + "\":" + String(yawOutputPID.getI(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_D) + "\":" + String(yawOutputPID.getD(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW) + "\":" + String(yawOutputPID.getOutput(), 2) + ",";

  data += "\"" + getIdFromName(NAME_SATELLITES) + "\":" + addDQuotes(sat_str) + ",";
  data += "\"" + getIdFromName(NAME_LATITUDE) + "\":" + addDQuotes(lat_str) + ",";
  data += "\"" + getIdFromName(NAME_LONGITUDE) + "\":" + addDQuotes(lng_str) + ",";
  data += "\"" + getIdFromName(NAME_DATE) + "\":" + addDQuotes(date_str) + ",";
  data += "\"" + getIdFromName(NAME_TIME) + "\":" + addDQuotes(time_str) + ",";
  data += "\"" + getIdFromName(NAME_SPEED) + "\":" + addDQuotes(speed_str) + ",";
  data += "\"" + getIdFromName(NAME_MAX_SPEED) + "\":" + addDQuotes(String(maxSpeed, 0)) + ",";
  data += "\"" + getIdFromName(NAME_TOTAL_DISTANCE) + "\":" + addDQuotes(String(totalDistance, 3)) + ",";

  data += "\"" + getIdFromName(NAME_SPEEDOMETER_GAUGE) + "\":" + addDQuotes(String(getSpeed())) + ",";

  data += addDQuotes(ID_BUZZER_BUTTON) + ":" + addDQuotes(getBuzzerCaption());

  data += "}";  
  //Serial.println(data);  
  return data;
}





