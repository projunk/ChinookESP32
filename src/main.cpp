#include <functions.h>
#include <WebServer.h>


#define NR_OF_ARRAY_ELEMS(array) ((sizeof(array))/(sizeof(array[0])))


// configure failsafe for PPM on IA6B receiver: https://www.youtube.com/watch?v=T0DcXxpBS78
// procedure to program receiver:
// 1) set ch3 min endpoint to 120% (Setup/Endpoints)
// 2) set failsafe on ch 3 to -118%  (Function/RX Setup/Failsafe)
// 3) set ch3 min endpoint back to 100% (Setup/Endpoints)
// throttle signal smaller then certain value => failsafe triggered
// apperently this method makes a change on the receiver, since after disconnecting transmitter, 
// the transmitter itself cannot be sending these shorter pulses anymore


TaskHandle_t handle_task1, handle_task2, handle_task3, handle_task4;

IPAddress ip(192, 168, 1, 170);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
String ssid[] = WIFI_SSID;
String password[] = WIFI_PASSWORD;

IPAddress AP_ip(192,168,8,1);
IPAddress AP_gateway(192,168,8,1);
IPAddress AP_subnet(255,255,255,0);
const char* AP_password = "123gps456";
const uint8_t AP_channel = 13;

WebServer server(80);

#define REDIRECT_TO_ROOT server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");

String activeTab = NAME_TAB_BUTTON_TELEMETRY;


void getWebPageHandler() {
  server.send(200, "text/html", getWebPage(activeTab)); 
}

void saveHandler() {
  Serial.println("/Save");

  rollAnglePID.set(server.arg("RollAngleP"), server.arg("RollAngleI"), server.arg("RollAngleD"), server.arg("RollAngleMax"));
  rollAnglePID.print();
  rollAnglePID.save();

  pitchAnglePID.set(server.arg("PitchAngleP"), server.arg("PitchAngleI"), server.arg("PitchAngleD"), server.arg("PitchAngleMax"));
  pitchAnglePID.print();
  pitchAnglePID.save();

  rollRatePID.set(server.arg("RollRateP"), server.arg("RollRateI"), server.arg("RollRateD"), server.arg("RollRateMax"));
  rollRatePID.print();
  rollRatePID.save();

  pitchRatePID.set(server.arg("PitchRateP"), server.arg("PitchRateI"), server.arg("PitchRateD"), server.arg("PitchRateMax"));
  pitchRatePID.print();
  pitchRatePID.save();

  yawRatePID.set(server.arg("YawRateP"), server.arg("YawRateI"), server.arg("YawRateD"), server.arg("YawRateMax"));
  yawRatePID.print();    
  yawRatePID.save();

  rollExpoFactor = checkExpo(server.arg(getIdFromName(NAME_ROLL_EXPO)).toDouble());
  pitchExpoFactor = checkExpo(server.arg(getIdFromName(NAME_PITCH_EXPO)).toDouble());
  yawExpoFactor = checkExpo(server.arg(getIdFromName(NAME_YAW_EXPO)).toDouble());
  frontServoCenterOffset = checkCenterOffset(server.arg(getIdFromName(NAME_FRONT_SERVO_CENTER_OFFSET)).toInt());
  backServoCenterOffset = checkCenterOffset(server.arg(getIdFromName(NAME_BACK_SERVO_CENTER_OFFSET)).toInt());
  voltageCorrectionFactor = server.arg(getIdFromName(NAME_VOLTAGE_CORRECTION)).toDouble();
  mpu6050.setCalibrationAccX(server.arg(getIdFromName(NAME_CALIBRATED_ACCX)).toDouble());
  mpu6050.setCalibrationAccY(server.arg(getIdFromName(NAME_CALIBRATED_ACCY)).toDouble());
  mpu6050.setCalibrationAccZ(server.arg(getIdFromName(NAME_CALIBRATED_ACCZ)).toDouble());
  printProps();
  saveProps();

  rollRateOutputPID.reset();
  pitchRateOutputPID.reset();
  yawRateOutputPID.reset();

  REDIRECT_TO_ROOT;
}


void cancelHandler() {
  Serial.println("/Cancel");
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void calibrateAccHandler() {
  Serial.println("/CalibrateAcc");
  vTaskSuspend(handle_task3);
  mpu6050.calibrateAcc();
  vTaskResume(handle_task3);
  saveProps();
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void wifiOffHandler() {
  Serial.println("/WifiOff");
  WiFi.mode(WIFI_OFF);
}  


void buzzerOnOffHandler() {
  Serial.println("/BuzzerOnOff");
  buzzerOff = ! buzzerOff;
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void defaultsHandler() {
  Serial.println("/Defaults");

  rollAnglePID.resetToDefault();
  rollAnglePID.print();
  rollAnglePID.save();

  pitchAnglePID.resetToDefault();
  pitchAnglePID.print();
  pitchAnglePID.save();

  rollRatePID.resetToDefault();
  rollRatePID.print();
  rollRatePID.save();

  pitchRatePID.resetToDefault();
  pitchRatePID.print();
  pitchRatePID.save();

  yawRatePID.resetToDefault();
  yawRatePID.print();    
  yawRatePID.save();

  rollExpoFactor = defaultRollExpoFactor;
  pitchExpoFactor = defaultPitchExpoFactor;
  yawExpoFactor = defaultYawExpoFactor;
  frontServoCenterOffset = defaultFrontServoCenterOffset;
  backServoCenterOffset = defaultBackServoCenterOffset;
  voltageCorrectionFactor = defaultVoltageCorrectionFactor;
  mpu6050.setCalibrationAccX(0.0);
  mpu6050.setCalibrationAccY(0.0);
  mpu6050.setCalibrationAccZ(0.0);
  printProps();
  saveProps();

  rollRateOutputPID.reset();
  pitchRateOutputPID.reset();
  yawRateOutputPID.reset();

  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;  
}  


void getLatestDataHandler() {
  //Serial.println("getLatestDataHandler");
  server.send(200, "text/html", getLatestData()); 
}


void notFoundHandler() {
  server.send(404, "text/plain", "Not found");
}


void setup() {
  Serial.begin(115200);

  robotName = identifyRobot(); 
  Serial.println();
  UniqueIDdump(Serial);
  Serial.println();
  Serial.print("Robot: ");
  Serial.println(robotName);

  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    rollAnglePID.load();
    pitchAnglePID.load();
    rollRatePID.load();
    pitchRatePID.load();
    yawRatePID.load();
    loadProps();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }
 
  xTaskCreatePinnedToCore(
    task1,
    "task1",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task1,
    CORE1);

  xTaskCreatePinnedToCore(
    task2,
    "task2",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task2,
    CORE1);

  xTaskCreatePinnedToCore(
    task3,
    "task3",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task3,
    CORE0);

  xTaskCreatePinnedToCore(
    task4,
    "task4",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task4,
    CORE1);


  int32_t strongestChannel;
  int strongestSSIDIndex;
  uint8_t* strongestBssid;

  strongestBssid = getChannelWithStrongestSignal(ssid, NR_OF_ARRAY_ELEMS(ssid), &strongestChannel, &strongestSSIDIndex);
  if (strongestBssid == NULL) {
    // standalone accesspoint
    WiFi.onEvent(WiFiAPStarted, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_START);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(getSSID().c_str(), AP_password, AP_channel); 
    for (;;) {
      if (isAPStarted()) {
        break;
      }
      delay(1);
    }  
    WiFi.softAPConfig(AP_ip, AP_gateway, AP_subnet);
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(getSSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    // connect to local wifi network
    Serial.print("Connecting WiFi"); 
    WiFi.config(ip, gateway, subnet);

    // disable power safe for performance (low latency)
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.begin(ssid[strongestSSIDIndex].c_str(), password[strongestSSIDIndex].c_str(), strongestChannel, strongestBssid, true);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(ip);

    // disable voltage alarm
    isVoltageAlarmEnabled = false;
  }

  voltage = readVoltage();
  Serial.print("Voltage [volt]: ");
  Serial.println(getVoltageStr());
  Serial.println();

  Serial.println("WebServer startup");

  Serial.println("WebServer startup");

  server.on("/", HTTPMethod::HTTP_GET, getWebPageHandler);   

  server.on("/Save", HTTPMethod::HTTP_GET, saveHandler);

  server.on("/Cancel", HTTPMethod::HTTP_GET, cancelHandler);

  server.on("/CalibrateAcc", HTTPMethod::HTTP_GET, calibrateAccHandler);

  server.on("/WifiOff", HTTPMethod::HTTP_GET, wifiOffHandler);

  server.on("/BuzzerOnOff", HTTPMethod::HTTP_GET, buzzerOnOffHandler);

  server.on("/Defaults", HTTPMethod::HTTP_GET, defaultsHandler);

  server.on("/RequestLatestData", HTTPMethod::HTTP_GET, getLatestDataHandler);

  server.onNotFound(notFoundHandler);

  server.begin();
}


void loop() {
  server.handleClient();
}
