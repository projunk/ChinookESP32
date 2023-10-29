#include <functions.h>


volatile int32_t wiFiSignalStrength = MIN_SIGNAL_STRENGTH;
volatile bool APStarted = false;
volatile int peakCount;
volatile unsigned long previousTimerValue;
volatile int frontEsc = 0;
volatile int backEsc = 0;
volatile int frontServo = 0;
volatile int backServo = 0;
volatile float voltage;
String robotName;
volatile bool signal_detected = false;
int signal_detected_count = 0;
int signal_lost_count = SIGNALS_DETECTED_LOST_THRESHOLD;
volatile double rollExpoFactor = defaultRollExpoFactor;
volatile double pitchExpoFactor = defaultPitchExpoFactor;
volatile double yawExpoFactor = defaultYawExpoFactor;
volatile int frontServoCenterOffset = defaultFrontServoCenterOffset;
volatile int backServoCenterOffset = defaultBackServoCenterOffset;
volatile double voltageCorrectionFactor = defaultVoltageCorrectionFactor;
volatile bool buzzerDisabled = false;
volatile bool buzzerOff = false;
volatile double gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
volatile double desiredRollAngle, desiredPitchAngle;
volatile double desiredRollRate, desiredPitchRate, desiredYawRate;
bool isVoltageAlarmEnabled = true;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

TwoPosSwitch switchA(SWA_CHANNEL);
TwoPosSwitch switchB(SWB_CHANNEL);
ThreePosSwitch switchC(SWC_CHANNEL);
TwoPosSwitch switchD(SWD_CHANNEL);

FlightMode flightMode;


PID rollAnglePID(2.0, 0.0, 0.0, 400.0, (LOOP_TIME_TASK3/1000000.0), "rollAngle");
PID pitchAnglePID(2.0, 0.0, 0.0, 400.0, (LOOP_TIME_TASK3/1000000.0), "pitcAngle");

PIDOutput rollAngleOutputPID(&rollAnglePID);
PIDOutput pitchAngleOutputPID(&pitchAnglePID);


PID rollRatePID(0.6, 3.5, 0.03, 400.0, (LOOP_TIME_TASK3/1000000.0), "roll");
PID pitchRatePID(0.6, 3.5, 0.03, 400.0, (LOOP_TIME_TASK3/1000000.0), "pitch");
PID yawRatePID(2.0, 12.0, 0.0, 400.0, (LOOP_TIME_TASK3/1000000.0), "yaw");

PIDOutput rollRateOutputPID(&rollRatePID);
PIDOutput pitchRateOutputPID(&pitchRatePID);
PIDOutput yawRateOutputPID(&yawRatePID);



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
  Serial.print(rollExpoFactor);
  Serial.print("\t");
  Serial.print(pitchExpoFactor);
  Serial.print("\t");
  Serial.print(yawExpoFactor);
  Serial.print("\t");
  Serial.print(frontServoCenterOffset);
  Serial.print("\t");
  Serial.print(backServoCenterOffset);
  Serial.print("\t");
  Serial.print(voltageCorrectionFactor);  
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
      rollExpoFactor = f.readStringUntil('\n').toDouble();
      pitchExpoFactor = f.readStringUntil('\n').toDouble();
      yawExpoFactor = f.readStringUntil('\n').toDouble();
      frontServoCenterOffset = f.readStringUntil('\n').toInt();
      backServoCenterOffset = f.readStringUntil('\n').toInt();
      voltageCorrectionFactor = f.readStringUntil('\n').toDouble();
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
    f.println(String(rollExpoFactor, 2));
    f.println(String(pitchExpoFactor, 2));
    f.println(String(yawExpoFactor, 2));
    f.println(String(frontServoCenterOffset));
    f.println(String(backServoCenterOffset));
    f.println(String(voltageCorrectionFactor, 2));
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


bool isEqualID(const uint8_t prmID[UniqueIDsize]) {
  for (size_t i = 0; i < UniqueIDsize; i++) {
			if (UniqueID[i] != prmID[i]) {
        return false;
      }
  }
  return true;
}


String identifyRobot() {
  String name;
  if (isEqualID(CHINOOK)) {
    name = "Chinook";
  } else {
    name = "Unknown";
  }
  return name;
}


double toDegrees(double prmRadians) {
  return prmRadians * 180.0 / PI;
}


bool isValidSignal(int prmPulse) {
  return (prmPulse > INVALID_SIGNAL_PULSE);
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
  int nrOfCells = getNrOfCells(vBatTotal);
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


int fixChannelDirection(int prmChannel, boolean prmReversed) {
  if (prmReversed) {
    return map(prmChannel, MIN_PULSE, MAX_PULSE, MAX_PULSE, MIN_PULSE);
  } else {
    return prmChannel;
  }
}


void printSetPoints(double prmRollSetPoint, double prmPitchSetPoint, double prmYawSetPoint) {
  Serial.print("RollSetPoint:\t");
  Serial.print(prmRollSetPoint);
  Serial.print("\tPitchSetPoint:\t");
  Serial.print(prmPitchSetPoint);
  Serial.print("\tYawSetPoint:\t");
  Serial.print(prmYawSetPoint);
  Serial.println();
}


void printPIDOutputs(double prmOutputRoll, double prmOutputPitch, double prmOutputYaw) {
  Serial.print("OutputRoll:\t");
  Serial.print(prmOutputRoll);
  Serial.print("\tOutputPitch:\t");
  Serial.print(prmOutputPitch);
  Serial.print("\tOutputYaw:\t");
  Serial.print(prmOutputYaw);
  Serial.println();
}


void printMotorOutputs(int prmFrontEsc, int prmBackEsc, int prmFrontServo, int prmBackServo) {
  Serial.print(prmFrontEsc);
  Serial.print("\t");
  Serial.print(prmBackEsc);
  Serial.print("\t");
  Serial.print(prmFrontServo);
  Serial.print("\t");
  Serial.print(prmBackServo);
  Serial.println();
}


void delayEx(uint32_t prmMilisec) {
  uint32_t timer = millis();
  while(millis() - timer < prmMilisec) {
    rtttl::play();
    vTaskDelay(1);
  }
}


double getLoopTimeHz(int prmLoopTime) {
  return 1000000.0 / prmLoopTime;
}


bool isBootButtonPressed() {
  return (digitalRead(0) == 0);
}


bool isBootButtonReleased() {
  return (digitalRead(0) == 1);
}


bool isBatteryConnected() {
  return (voltage > LOW_VOLTAGE_ALARM);
}


bool isBatteryDisconnected() {
  return !isBatteryConnected();
}


void waitForBatteryConnected() {
  for (;;) {
    if (isBatteryConnected()) {
      break;
    }
    playVeryShortBeep();
    delayEx(500);
  }
}


void waitForBatteryDisconnected() {
  for (;;) {
    if (isBatteryDisconnected()) {
      break;
    }
    playLongBeep();
    delayEx(500);
  }
}


void waitForBootButtonClicked() {
  for (;;) {
    if (isBootButtonPressed()) {
      break;
    }
    delayEx(50);
  }

  delay(100);

  for (;;) {
    if (isBootButtonReleased()) {
      break;
    }
    delayEx(50);
    Serial.println(voltage);
  }
}


extern void calibrateESCs() {
  Serial.println("Disconnect battery");
  waitForBatteryDisconnected();

  Serial.println("Connect battery");
  writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, MAX_PULSE);
  writeEscPWM(MOTOR_BACK_PWM_CHANNEL, MAX_PULSE);
  waitForBatteryConnected();

  Serial.println("Calibrating...");
  writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, MIN_PULSE);
  writeEscPWM(MOTOR_BACK_PWM_CHANNEL, MIN_PULSE);
  delayEx(2000);

  playCalibrated();
}


extern int limitEsc(int prmPulse) {
  int rval = prmPulse;
  if (prmPulse > MAX_PULSE) rval = MAX_PULSE;
  if (prmPulse < MIN_THROTTLE) rval = MIN_THROTTLE;
  return rval;
}


extern int limitServo(int prmPulse) {
  int rval = prmPulse;
  if (prmPulse > MAX_PULSE) rval = MAX_PULSE;
  if (prmPulse < MIN_PULSE) rval = MIN_PULSE;
  return rval;
}


bool isArmingSwitchTriggered(){
  return (signal_detected && (switchA.readPos() == 2));
}


bool isArmed() {
  return currentIsArmed;
}


bool isArmingAllowed() {
  int throttleChannel = fixChannelDirection(ppm->getValue(THROTTLE_CHANNEL), throttleChannelReversed);
  return throttleChannel < (MIN_PULSE + DEADBAND_HALF);
}


void initValues() {
  gyro_roll_input = 0.0;
  gyro_pitch_input = 0.0;
  gyro_yaw_input = 0.0;

  kalmanRollAngle = mpu6050.getAngleRollAcc();
  kalmanPitchAngle = mpu6050.getAnglePitchAcc();
  yawAngle = 0.0;

  rollAngleOutputPID.reset();
  pitchAngleOutputPID.reset();

  rollRateOutputPID.reset();
  pitchRateOutputPID.reset();
  yawRateOutputPID.reset();
}


FlightMode getFlightMode() {
  switch (switchC.readPos()) {
    case 1:
      return fmAutoLevel;
    case 2:
      return fmAngleLimit;
    default:
      return fmNone;
  } 

}


void playTune(String prmTune) {
  if ((!buzzerDisabled) && (!buzzerOff)) {
    static char buf[64];
    strcpy(buf, prmTune.c_str());
    rtttl::begin(BUZZER_PIN, buf);
  }
}


void playVeryShortBeep() {
  playTune("VeryShortBeep:d=8,o=5,b=140:c5");
}


void playShortBeep() {
  playTune("ShortBeep:d=32,o=5,b=140:c5");
}


void playLongBeep() {
  playTune("LongBeep:d=64,o=5,b=140:c5");
}


void playLowVoltageAlarm() {
  if (isVoltageAlarmEnabled) {
    playTune("LowVoltageAlarm:d=16,o=5,b=140:c6,P,c5,P,c6,P,c5,P");
  }
}


void playArmed() {
  Serial.println("Armed");        
  playTune("Armed:d=16,o=5,b=140:c5,P,c6,P,a7");
}


void playDisarmed() {
  Serial.println("Disarmed");        
  playTune("DisArmed:d=16,o=5,b=140:a7,P,c6,P,c5");  
}


void playCalibrated() {
  Serial.println("Calibrated");        
  playTune("Calibrated:d=16,o=5,b=140:c5,P,c5,P,c5,P,c5,P,a7,P");
}


void playSignalDetected() {
  Serial.println("Signal Detected");   
  playTune("SignalDetected:d=16,o=5,b=140:c5,P,c5,P,c5,P,c5,P,c5,P,c5,P,a7,P");
}


void playSignalLost() {
  Serial.println("Signal Lost");   
  playTune("SignalLost:d=16,o=5,b=140:a7,P,c5,P,c5,P,c5,P,c5,P,c5,P,c5,P");  
}


void writeServoPWM(uint8_t prmChannel, uint32_t prmMicroSeconds) {
  uint32_t valueMax = 1000000/PWM_FREQUENCY_SERVO;
  uint32_t m = pow(2, PWM_RESOLUTION_SERVO) - 1;
  uint32_t duty = (m * min(prmMicroSeconds, valueMax)/valueMax);
  /*
  Serial.print(valueMax);
  Serial.print("\t");
  Serial.print(prmMicroSeconds);
  Serial.print("\t");
  Serial.print(m);
  Serial.print("\t");
  Serial.println(duty);
  */
  ledcWrite(prmChannel, duty);
}


void writeEscPWM(uint8_t prmChannel, uint32_t prmMicroSeconds) {
  uint32_t valueMax = 1000000/PWM_FREQUENCY_ESC;
  uint32_t m = pow(2, PWM_RESOLUTION_ESC) - 1;
  uint32_t duty = (m * min(prmMicroSeconds, valueMax)/valueMax);
  ledcWrite(prmChannel, duty);
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


String getSSID() {
  String ssid = SSID_BASE + robotName;
  ssid.toUpperCase();
  return ssid;
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


String getIdFromName(String prmName) {
  String id = prmName;
  id.replace(" ", "_");
  id.replace("[", "");
  id.replace("]", "");
  id.replace("(", "");
  id.replace(")", "");
  return id;
}


bool isAPStarted() {
  return APStarted;
}


void WiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info) {
  APStarted = true;
  Serial.println("AP Started");
}


String addDQuotes(String prmValue) {
  return "\"" + prmValue + "\"";
}


String addRow(String prmName, bool prmIsEditableCell, bool prmIsHeader, String prmValue = "") {
  String editableCell = prmIsEditableCell ? "contenteditable='true'" : "";
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:20%\" ALIGN=CENTER>" + prmName + suffix;
  String col2 = prefix + editableCell + " id=" + addDQuotes(getIdFromName(prmName)) + " style=\"width:20%\" ALIGN=CENTER>" + prmValue + suffix;
  return "<tr>" + col1 + col2 + "</tr>";
}


String addRow(String name, bool prmIsHeader, String prmName1, String prmName2, String prmName3, String prmName4, String prmName5, String prmName6, String prmName7, String prmName8, String prmName9) {
  String prefix = prmIsHeader ? "<th " : "<td ";
  String suffix = prmIsHeader ? "</th>" : "</td>";
  String col1 = prefix + "style=\"width:9%\" ALIGN=CENTER>" + name + suffix;
  String col2, col3, col4, col5, col6, col7, col8, col9, col10;
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
  }
  return "<tr>" + col1 + col2 + col3 + col4 + col5 + col6 + col7 + col8 + col9 + col10 + "</tr>";
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


String toString(double prmValue) {
  return String(prmValue, 2);
}


String getGenericProgressStr(String prmID, String prmSpanID, int prmValueMin, int prmValueMax) {
  return "<div class=\"progress\"><div id=\"" + prmID + "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"" + String(prmValueMin) + "\"  aria-valuemax=\"" + String(prmValueMax) + "\" style=\"width:0%\"><span id=\"" + prmSpanID + "\" ></span></div></div>";
}


String getVoltageProgressStr() {
  return "<div class=\"progress\"><div id=\"" ID_PROGRESS_VOLTAGE "\" class=\"progress-bar progress-bar-danger\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">0%</div></div>";
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
  s += "  <meta><title>WebService: " + robotName + "</title>";

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
  s += "    .progress-bar-ch7{background-color:#ccff99}";  
  s += "    .progress-bar-ch8{background-color:#ffcc00}";
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

  s += "</head>";
  return s;
}


String getScript(String prmToBeClickedTabButton) {
  String s = "";
  s += "<script>";

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

  s += "        updateWifiSignalStrength(\"" + getIdFromName(ID_PROGRESS_WIFI_SS) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_WIFI_SS) + "\"," + "data." + getIdFromName(NAME_WIFI_SIGNAL_STRENGTH) + ");";

  s += "        document.getElementById(\"" + getIdFromName(NAME_SIGNAL_DETECTED) + "\").innerText = data." + getIdFromName(NAME_SIGNAL_DETECTED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ARMED) + "\").innerText = data." + getIdFromName(NAME_ARMED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_FLIGHT_MODE) + "\").innerText = data." + getIdFromName(NAME_FLIGHT_MODE) + ";";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_1) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_1) + "\"," + "data." + getIdFromName(NAME_CHANNEL_1) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_2) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_2) + "\"," + "data." + getIdFromName(NAME_CHANNEL_2) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_3) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_3) + "\"," + "data." + getIdFromName(NAME_CHANNEL_3) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_4) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_4) + "\"," + "data." + getIdFromName(NAME_CHANNEL_4) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_5) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_5) + "\"," + "data." + getIdFromName(NAME_CHANNEL_5) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_6) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_6) + "\"," + "data." + getIdFromName(NAME_CHANNEL_6) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_7) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_7) + "\"," + "data." + getIdFromName(NAME_CHANNEL_7) + ");";
  s += "        updateChannel(\"" + getIdFromName(ID_PROGRESS_CHANNEL_8) + "\"," + "\"" + getIdFromName(ID_SPAN_PROGRESS_CHANNEL_8) + "\"," + "data." + getIdFromName(NAME_CHANNEL_8) + ");";

  s += "        document.getElementById(\"" + getIdFromName(NAME_VOLTAGE) + "\").innerText = data." + getIdFromName(NAME_VOLTAGE) + ";";  
  s += "        var voltageProgressElem = document.getElementById(\"" ID_PROGRESS_VOLTAGE "\");";
  s += "        var voltage = parseFloat(data." + getIdFromName(NAME_VOLTAGE) + ");";
  s += "        voltageProgressElem.setAttribute(\"class\", getVoltageColorMsg(voltage));";
  s += "        voltageProgressElem.setAttribute(\"aria-valuenow\", parseFloat(getVoltagePercentage(voltage)).toFixed(0));";
  s += "        voltageProgressElem.setAttribute(\"style\", \"width:\" + parseFloat(getVoltagePercentage(voltage)).toFixed(0) + \"%\");";
  s += "        voltageProgressElem.innerText = parseFloat(getVoltagePercentage(voltage)).toFixed(0) + \"%\";";

  s += "        document.getElementById(\"" + getIdFromName(NAME_FRONT_ESC) + "\").innerText = data." + getIdFromName(NAME_FRONT_ESC) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_BACK_ESC) + "\").innerText = data." + getIdFromName(NAME_BACK_ESC) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_FRONT_SERVO) + "\").innerText = data." + getIdFromName(NAME_FRONT_SERVO) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_BACK_SERVO) + "\").innerText = data." + getIdFromName(NAME_BACK_SERVO) + ";";  

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

  // roll angle  
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_KALMAN_ROLL_ANGLE_INPUT) + "\").innerText = data." + getIdFromName(NAME_KALMAN_ROLL_ANGLE_INPUT) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_ROLL_ANGLE_DESIRED) + "\").innerText = data." + getIdFromName(NAME_PID_ROLL_ANGLE_DESIRED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_ERROR) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_P) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_I) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_D) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE) + ";";
  
  // pitch angle
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_KALMAN_PITCH_ANGLE_INPUT) + "\").innerText = data." + getIdFromName(NAME_KALMAN_PITCH_ANGLE_INPUT) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_PITCH_ANGLE_DESIRED) + "\").innerText = data." + getIdFromName(NAME_PID_PITCH_ANGLE_DESIRED) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_ERROR) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_P) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_I) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_D) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE) + ";";  
   
  // roll rate
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL_ACC_COPY) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_ROLL_COPY) + "\").innerText = data." + getIdFromName(NAME_ANGLE_ROLL) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_ROLL_RATE_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_ROLL_RATE_INPUT) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_ROLL_RATE_DESIRED) + "\").innerText = data." + getIdFromName(NAME_PID_ROLL_RATE_DESIRED) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_ERROR) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_P) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_I) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_D) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE) + ";";

  // pitch rate
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH_ACC_COPY) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH_ACC) + ";";
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_PITCH_COPY) + "\").innerText = data." + getIdFromName(NAME_ANGLE_PITCH) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_PITCH_RATE_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_PITCH_RATE_INPUT) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_PITCH_RATE_DESIRED) + "\").innerText = data." + getIdFromName(NAME_PID_PITCH_RATE_DESIRED) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_ERROR) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_P) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_I) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_D) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE) + ";";  

  // yaw rate
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_YAW_ACC) + "\").innerText = data." + getIdFromName(NAME_ANGLE_YAW_ACC) + ";";  
  s += "        document.getElementById(\"" + getIdFromName(NAME_ANGLE_YAW) + "\").innerText = data." + getIdFromName(NAME_ANGLE_YAW) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_GYRO_YAW_RATE_INPUT) + "\").innerText = data." + getIdFromName(NAME_GYRO_YAW_RATE_INPUT) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_YAW_RATE_DESIRED) + "\").innerText = data." + getIdFromName(NAME_PID_YAW_RATE_DESIRED) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_ERROR) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_ERROR) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_P) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_P) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_I) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_I) + ";"; 
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_D) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_D) + ";";   
  s += "        document.getElementById(\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE) + "\").innerText = data." + getIdFromName(NAME_PID_OUTPUT_YAW_RATE) + ";"; 
    
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
  s += "  var rollAngleValues = getNameValue(\"RollAngleP\") + \"&\" + getNameValue(\"RollAngleI\") + \"&\" + getNameValue(\"RollAngleD\") + \"&\" + getNameValue(\"RollAngleMax\");";
  s += "  var pitchAngleValues = getNameValue(\"PitchAngleP\") + \"&\" + getNameValue(\"PitchAngleI\") + \"&\" + getNameValue(\"PitchAngleD\") + \"&\" + getNameValue(\"PitchAngleMax\");";
  s += "  var rollRateValues = getNameValue(\"RollRateP\") + \"&\" + getNameValue(\"RollRateI\") + \"&\" + getNameValue(\"RollRateD\") + \"&\" + getNameValue(\"RollRateMax\");";
  s += "  var pitchRateValues = getNameValue(\"PitchRateP\") + \"&\" + getNameValue(\"PitchRateI\") + \"&\" + getNameValue(\"PitchRateD\") + \"&\" + getNameValue(\"PitchRateMax\");";
  s += "  var yawRateValues = getNameValue(\"YawRateP\") + \"&\" + getNameValue(\"YawRateI\") + \"&\" + getNameValue(\"YawRateD\") + \"&\" + getNameValue(\"YawRateMax\");";
  s += "  var rollExpo = getNameValue(\"" + getIdFromName(NAME_ROLL_EXPO) + "\");";
  s += "  var pitchExpo = getNameValue(\"" + getIdFromName(NAME_PITCH_EXPO) + "\");";
  s += "  var yawExpo = getNameValue(\"" + getIdFromName(NAME_YAW_EXPO) + "\");";
  s += "  var frontServoCenterOffset = getNameValue(\"" + getIdFromName(NAME_FRONT_SERVO_CENTER_OFFSET) + "\");";
  s += "  var backServoCenterOffset = getNameValue(\"" + getIdFromName(NAME_BACK_SERVO_CENTER_OFFSET) + "\");";
  s += "  var voltageCorrectionFactor = getNameValue(\"" + getIdFromName(NAME_VOLTAGE_CORRECTION) + "\");";
  s += "  var calibrated_accX = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCX) + "\");";
  s += "  var calibrated_accY = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCY) + "\");";  
  s += "  var calibrated_accZ = getNameValue(\"" + getIdFromName(NAME_CALIBRATED_ACCZ) + "\");";  
  s += "  xhr.open(\"GET\", \"/Save?\" + rollAngleValues + \"&\" + pitchAngleValues + \"&\" + rollRateValues + \"&\" + pitchRateValues + \"&\" + yawRateValues + \"&\" + rollExpo + \"&\" + pitchExpo + \"&\" + yawExpo + \"&\" + frontServoCenterOffset + \"&\" + backServoCenterOffset + \"&\" + voltageCorrectionFactor + \"&\" + calibrated_accX + \"&\" + calibrated_accY + \"&\" + calibrated_accZ, false);";
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


String getFlightModeSt() {
  switch (getFlightMode()) {
    case fmAutoLevel:
      return "AutoLevel";
    case fmAngleLimit:
      return "AngleLimit";
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

  s += "<body>";

  s += "<div class=\"tab\">";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_TELEMETRY) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_TELEMETRY) + "\">" NAME_TAB_TELEMETRY "</button>";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_PID) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_PID) + "\">" NAME_TAB_PID "</button>";
  s += "<button class=\"tablinks\" onclick=\"selectTab(event, '" + getIdFromName(NAME_TAB_SETTINGS) + "')\" id=\"" + getIdFromName(NAME_TAB_BUTTON_SETTINGS) + "\">" NAME_TAB_SETTINGS "</button>";
  s += "</div>";


  s += "<div id=\"" NAME_TAB_TELEMETRY "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_MODEL, false, true, robotName);
  s += addRow(NAME_VERSION, false, false, CHINOOK_VERSION);
  s += addRow(NAME_RESPONSE_TIME, false, false, getGenericProgressStr(ID_PROGRESS_RESP_TIME, ID_SPAN_PROGRESS_RESP_TIME, MIN_RESPONSE_TIME, MAX_RESPONSE_TIME));
  s += addRow(NAME_WIFI_SIGNAL_STRENGTH, false, false, getGenericProgressStr(ID_PROGRESS_WIFI_SS, ID_SPAN_PROGRESS_WIFI_SS, MIN_SIGNAL_STRENGTH, MAX_SIGNAL_STRENGTH));
  s += addRow(NAME_SIGNAL_DETECTED, false, false);
  s += addRow(NAME_ARMED, false, false);
  s += addRow(NAME_FLIGHT_MODE, false, false);
  s += addRow(NAME_CHANNEL_1, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_1, ID_SPAN_PROGRESS_CHANNEL_1, "progress-bar-ch1"));
  s += addRow(NAME_CHANNEL_2, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_2, ID_SPAN_PROGRESS_CHANNEL_2, "progress-bar-ch2"));
  s += addRow(NAME_CHANNEL_3, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_3, ID_SPAN_PROGRESS_CHANNEL_3, "progress-bar-ch3"));
  s += addRow(NAME_CHANNEL_4, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_4, ID_SPAN_PROGRESS_CHANNEL_4, "progress-bar-ch4"));
  s += addRow(NAME_CHANNEL_5, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_5, ID_SPAN_PROGRESS_CHANNEL_5, "progress-bar-ch5"));
  s += addRow(NAME_CHANNEL_6, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_6, ID_SPAN_PROGRESS_CHANNEL_6, "progress-bar-ch6"));  
  s += addRow(NAME_CHANNEL_7, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_7, ID_SPAN_PROGRESS_CHANNEL_7, "progress-bar-ch7"));  
  s += addRow(NAME_CHANNEL_8, false, false, getChannelProgressStr(ID_PROGRESS_CHANNEL_8, ID_SPAN_PROGRESS_CHANNEL_8, "progress-bar-ch8"));  
  s += addRow(NAME_VOLTAGE, false, false);
  s += addRow(NAME_VOLTAGE_PROGRESS, false, false, getVoltageProgressStr());
  s += addRow(NAME_FRONT_ESC, false, false);
  s += addRow(NAME_BACK_ESC, false, false);
  s += addRow(NAME_FRONT_SERVO, false, false);
  s += addRow(NAME_BACK_SERVO, false, false);
  s += addRow(NAME_GYRO_X, false, false);
  s += addRow(NAME_GYRO_Y, false, false);
  s += addRow(NAME_GYRO_Z, false, false);
  s += addRow(NAME_ACC_X, false, false);
  s += addRow(NAME_ACC_Y, false, false);
  s += addRow(NAME_ACC_Z, false, false);
  s += addRow(NAME_TEMPERATURE, false, false);
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_1, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_1, ID_SPAN_PROGRESS_LOOPTIME_1, LOOP_TIME_TASK1));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_2, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_2, ID_SPAN_PROGRESS_LOOPTIME_2, LOOP_TIME_TASK2));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_3, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_3, ID_SPAN_PROGRESS_LOOPTIME_3, LOOP_TIME_TASK3));
  s += addRow(NAME_USED_UP_LOOPTIME_PROGRESS_4, false, false, getLoopTimeProgressStr(ID_PROGRESS_LOOPTIME_4, ID_SPAN_PROGRESS_LOOPTIME_4, LOOP_TIME_TASK4));  
  s += "</table>";

  s += "</div>";


  s += "<div id=\"" NAME_TAB_PID "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow("", true, "Angle Acc", "Angle", "Input", "Desired", "Error", "P", "I", "D", "Output");
  s += addRow(NAME_TELEMETRY_ROLL_ANGLE, false, NAME_ANGLE_ROLL_ACC, NAME_ANGLE_ROLL, NAME_KALMAN_ROLL_ANGLE_INPUT, NAME_PID_ROLL_ANGLE_DESIRED, NAME_PID_OUTPUT_ROLL_ANGLE_ERROR, NAME_PID_OUTPUT_ROLL_ANGLE_P, NAME_PID_OUTPUT_ROLL_ANGLE_I, NAME_PID_OUTPUT_ROLL_ANGLE_D, NAME_PID_OUTPUT_ROLL_ANGLE);
  s += addRow(NAME_TELEMETRY_PITCH_ANGLE, false, NAME_ANGLE_PITCH_ACC, NAME_ANGLE_PITCH, NAME_KALMAN_PITCH_ANGLE_INPUT, NAME_PID_PITCH_ANGLE_DESIRED, NAME_PID_OUTPUT_PITCH_ANGLE_ERROR, NAME_PID_OUTPUT_PITCH_ANGLE_P, NAME_PID_OUTPUT_PITCH_ANGLE_I, NAME_PID_OUTPUT_PITCH_ANGLE_D, NAME_PID_OUTPUT_PITCH_ANGLE);
  s += addRow(NAME_TELEMETRY_ROLL_RATE, false, NAME_ANGLE_ROLL_ACC_COPY, NAME_ANGLE_ROLL_COPY, NAME_GYRO_ROLL_RATE_INPUT, NAME_PID_ROLL_RATE_DESIRED, NAME_PID_OUTPUT_ROLL_RATE_ERROR, NAME_PID_OUTPUT_ROLL_RATE_P, NAME_PID_OUTPUT_ROLL_RATE_I, NAME_PID_OUTPUT_ROLL_RATE_D, NAME_PID_OUTPUT_ROLL_RATE);
  s += addRow(NAME_TELEMETRY_PITCH_RATE, false, NAME_ANGLE_PITCH_ACC_COPY, NAME_ANGLE_PITCH_COPY, NAME_GYRO_PITCH_RATE_INPUT, NAME_PID_PITCH_RATE_DESIRED, NAME_PID_OUTPUT_PITCH_RATE_ERROR, NAME_PID_OUTPUT_PITCH_RATE_P, NAME_PID_OUTPUT_PITCH_RATE_I, NAME_PID_OUTPUT_PITCH_RATE_D, NAME_PID_OUTPUT_PITCH_RATE);
  s += addRow(NAME_TELEMETRY_YAW_RATE, false, NAME_ANGLE_YAW_ACC, NAME_ANGLE_YAW, NAME_GYRO_YAW_RATE_INPUT, NAME_PID_YAW_RATE_DESIRED, NAME_PID_OUTPUT_YAW_RATE_ERROR, NAME_PID_OUTPUT_YAW_RATE_P, NAME_PID_OUTPUT_YAW_RATE_I, NAME_PID_OUTPUT_YAW_RATE_D, NAME_PID_OUTPUT_YAW_RATE);
  s += "</table>";

  s += "</div>";


  s += "<div id=\"" NAME_TAB_SETTINGS "\" class=\"tabcontent\">";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow(NAME_SETTINGS, false, true, "");  
  s += addRow(NAME_ROLL_EXPO, true, false, String(rollExpoFactor, 2));
  s += addRow(NAME_PITCH_EXPO, true, false, String(pitchExpoFactor, 2));
  s += addRow(NAME_YAW_EXPO, true, false, String(yawExpoFactor, 2));
  s += addRow(NAME_FRONT_SERVO_CENTER_OFFSET, true, false, String(frontServoCenterOffset));
  s += addRow(NAME_BACK_SERVO_CENTER_OFFSET, true, false, String(backServoCenterOffset));
  s += addRow(NAME_VOLTAGE_CORRECTION, true, false, String(voltageCorrectionFactor, 2));
  s += addRow(NAME_CALIBRATED_ACCX, true, false, String(mpu6050.getCalibrationAccX(), 2));
  s += addRow(NAME_CALIBRATED_ACCY, true, false, String(mpu6050.getCalibrationAccY(), 2));  
  s += addRow(NAME_CALIBRATED_ACCZ, true, false, String(mpu6050.getCalibrationAccZ(), 2));  
  s += "</table>";

  s += "<br>";
  s += "<br>";

  s += "<table ALIGN=CENTER style=width:50%>";
  s += addRow2("PID", "P", "I", "D", "Max", false, true);
  s += addRow2(NAME_PID_SETTINGS_ROLL_ANGLE, toString(rollAnglePID.getP()), toString(rollAnglePID.getI()), toString(rollAnglePID.getD()), toString(rollAnglePID.getMax()), true, false);
  s += addRow2(NAME_PID_SETTINGS_PITCH_ANGLE, toString(pitchAnglePID.getP()), toString(pitchAnglePID.getI()), toString(pitchAnglePID.getD()), toString(pitchAnglePID.getMax()), true, false);
  s += addRow2(NAME_PID_SETTINGS_ROLL_RATE, toString(rollRatePID.getP()), toString(rollRatePID.getI()), toString(rollRatePID.getD()), toString(rollRatePID.getMax()), true, false);
  s += addRow2(NAME_PID_SETTINGS_PITCH_RATE, toString(pitchRatePID.getP()), toString(pitchRatePID.getI()), toString(pitchRatePID.getD()), toString(pitchRatePID.getMax()), true, false);
  s += addRow2(NAME_PID_SETTINGS_YAW_RATE, toString(yawRatePID.getP()), toString(yawRatePID.getI()), toString(yawRatePID.getD()), toString(yawRatePID.getMax()), true, false);
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
  data += "\"" + getIdFromName(NAME_FLIGHT_MODE) + "\":" + addDQuotes(getFlightModeSt()) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_1) + "\":" + addDQuotes(toString((int)ppm->getValue(1))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_2) + "\":" + addDQuotes(toString((int)ppm->getValue(2))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_3) + "\":" + addDQuotes(toString((int)ppm->getValue(3))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_4) + "\":" + addDQuotes(toString((int)ppm->getValue(4))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_5) + "\":" + addDQuotes(toString((int)ppm->getValue(5))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_6) + "\":" + addDQuotes(toString((int)ppm->getValue(6))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_7) + "\":" + addDQuotes(toString((int)ppm->getValue(7))) + ",";
  data += "\"" + getIdFromName(NAME_CHANNEL_8) + "\":" + addDQuotes(toString((int)ppm->getValue(8))) + ",";
  data += "\"" + getIdFromName(NAME_VOLTAGE) + "\":" + addDQuotes(getVoltageStr()) + ",";  
  data += "\"" + getIdFromName(NAME_FRONT_ESC) + "\":" + addDQuotes(toString(frontEsc)) + ",";  
  data += "\"" + getIdFromName(NAME_BACK_ESC) + "\":" + addDQuotes(toString(backEsc)) + ",";  
  data += "\"" + getIdFromName(NAME_FRONT_SERVO) + "\":" + addDQuotes(toString(frontServo)) + ",";  
  data += "\"" + getIdFromName(NAME_BACK_SERVO) + "\":" + addDQuotes(toString(backServo)) + ",";  
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

  data += "\"" + getIdFromName(NAME_ANGLE_ROLL) + "\":" + String(kalmanRollAngle, 0) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_PITCH) + "\":" + String(kalmanPitchAngle, 0) + ",";
  data += "\"" + getIdFromName(NAME_ANGLE_YAW) + "\":" + String(yawAngle, 0) + ",";

  data += "\"" + getIdFromName(NAME_KALMAN_ROLL_ANGLE_INPUT) + "\":" + String(kalmanRollAngle, 2) + ",";
  data += "\"" + getIdFromName(NAME_KALMAN_PITCH_ANGLE_INPUT) + "\":" + String(kalmanPitchAngle, 2) + ",";

  data += "\"" + getIdFromName(NAME_GYRO_ROLL_RATE_INPUT) + "\":" + String(gyro_roll_input, 2) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_PITCH_RATE_INPUT) + "\":" + String(gyro_pitch_input, 2) + ",";
  data += "\"" + getIdFromName(NAME_GYRO_YAW_RATE_INPUT) + "\":" + String(gyro_yaw_input, 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_ROLL_ANGLE_DESIRED) + "\":" + String(desiredRollAngle, 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_PITCH_ANGLE_DESIRED) + "\":" + String(desiredPitchAngle, 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_ROLL_RATE_DESIRED) + "\":" + String(desiredRollRate, 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_PITCH_RATE_DESIRED) + "\":" + String(desiredPitchRate, 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_YAW_RATE_DESIRED) + "\":" + String(desiredYawRate, 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_ERROR) + "\":" + String(rollAngleOutputPID.getError(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_ERROR) + "\":" + String(pitchAngleOutputPID.getError(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_P) + "\":" + String(rollAngleOutputPID.getP(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_P) + "\":" + String(pitchAngleOutputPID.getP(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_I) + "\":" + String(rollAngleOutputPID.getI(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_I) + "\":" + String(pitchAngleOutputPID.getI(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE_D) + "\":" + String(rollAngleOutputPID.getD(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE_D) + "\":" + String(pitchAngleOutputPID.getD(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_ANGLE) + "\":" + String(rollAngleOutputPID.getOutput(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_ANGLE) + "\":" + String(pitchAngleOutputPID.getOutput(), 2) + ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_ERROR) + "\":" + String(rollRateOutputPID.getError(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_ERROR) + "\":" + String(pitchRateOutputPID.getError(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_ERROR) + "\":" + String(yawRateOutputPID.getError(), 2)+ ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_P) + "\":" + String(rollRateOutputPID.getP(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_P) + "\":" + String(pitchRateOutputPID.getP(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_P) + "\":" + String(yawRateOutputPID.getP(), 2)+ ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_I) + "\":" + String(rollRateOutputPID.getI(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_I) + "\":" + String(pitchRateOutputPID.getI(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_I) + "\":" + String(yawRateOutputPID.getI(), 2)+ ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE_D) + "\":" + String(rollRateOutputPID.getD(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE_D) + "\":" + String(pitchRateOutputPID.getD(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE_D) + "\":" + String(yawRateOutputPID.getD(), 2)+ ",";

  data += "\"" + getIdFromName(NAME_PID_OUTPUT_ROLL_RATE) + "\":" + String(rollRateOutputPID.getOutput(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_PITCH_RATE) + "\":" + String(pitchRateOutputPID.getOutput(), 2) + ",";
  data += "\"" + getIdFromName(NAME_PID_OUTPUT_YAW_RATE) + "\":" + String(yawRateOutputPID.getOutput(), 2) + ",";

  data += addDQuotes(ID_BUZZER_BUTTON) + ":" + addDQuotes(getBuzzerCaption());

  data += "}";
  //Serial.println(data);
  return data;
}
