#include <functions.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// configure failsafe for PPM on IA6B receiver: https://www.youtube.com/watch?v=T0DcXxpBS78
// procedure to program receiver:
// 1) set ch3 min endpoint to 120% (Setup/Endpoints)
// 2) set failsafe on ch 3 to -118%  (Function/RX Setup/Failsafe)
// 3) set ch3 min endpoint back to 100% (Setup/Endpoints)
// throttle signal smaller then certain value => failsafe triggered
// apperently this method makes a change on the receiver, since after disconnecting transmitter, 
// the transmitter itself cannot be sending these shorter pulses anymore


TaskHandle_t core2;

IPAddress ip(192, 168, 1, 170);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
String ssid = WIFI_SSID;
String password = WIFI_PASSWORD;

IPAddress AP_ip(192,168,8,1);
IPAddress AP_gateway(192,168,8,1);
IPAddress AP_subnet(255,255,255,0);
const char* AP_password = "123gps456";
const uint8_t AP_channel = 13;

AsyncWebServer server(80);



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
    rollPID.load();
    pitchPID.load();
    yawPID.load();
    loadProps();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }

  // pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RECEIVER_PPM_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_SENSOR_PIN, ANALOG);

  // receiver
  initReceiver();
  attachInterrupt(RECEIVER_PPM_PIN, ppmInterruptHandler, FALLING);
  delay(100);

  // start second core
  xTaskCreatePinnedToCore(
    runOnCore2,
    "Core2",
    STACK_SIZE_CORE2,
    NULL,
    1,
    &core2,
    0);

  int32_t strongestChannel;
  uint8_t* strongestBssid;

  strongestBssid = getChannelWithStrongestSignal(ssid, &strongestChannel);
  if (strongestBssid == NULL) {
    // standalone accesspoint
    WiFi.onEvent(WiFiAPStarted, WiFiEvent_t::SYSTEM_EVENT_AP_START);

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

    WiFi.begin(ssid.c_str(), password.c_str(), strongestChannel, strongestBssid, true);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(ip);
  }

  voltage = readVoltage();
  Serial.print("Volatge [volt]: ");
  Serial.println(getVoltageStr());
  Serial.println();

  Serial.println("WebServer startup");

  server.on("/", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {   
    request->send(200, "text/html", getWebPage());
  }); 

  server.on("/Save", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Save");
    
    rollPID.set(request->getParam("Roll1")->value(), request->getParam("Roll2")->value(), request->getParam("Roll3")->value(), request->getParam("Roll4")->value());
    rollPID.print();
    rollPID.save();

    pitchPID.set(request->getParam("Pitch1")->value(), request->getParam("Pitch2")->value(), request->getParam("Pitch3")->value(), request->getParam("Pitch4")->value());
    pitchPID.print();
    pitchPID.save();

    yawPID.set(request->getParam("Yaw1")->value(), request->getParam("Yaw2")->value(), request->getParam("Yaw3")->value(), request->getParam("Yaw4")->value());
    yawPID.print();    
    yawPID.save();

    rollExpoFactor = checkExpo(request->getParam(getIdFromName(NAME_ROLL_EXPO))->value().toDouble());
    pitchExpoFactor = checkExpo(request->getParam(getIdFromName(NAME_PITCH_EXPO))->value().toDouble());
    yawExpoFactor = checkExpo(request->getParam(getIdFromName(NAME_YAW_EXPO))->value().toDouble());
    frontServoCenterOffset = checkCenterOffset(request->getParam(getIdFromName(NAME_FRONT_SERVO_CENTER_OFFSET))->value().toInt());
    backServoCenterOffset = checkCenterOffset(request->getParam(getIdFromName(NAME_BACK_SERVO_CENTER_OFFSET))->value().toInt());
    voltageCorrectionFactor = request->getParam(getIdFromName(NAME_VOLTAGE_CORRECTION))->value().toDouble();
    calibrated_angle_roll_acc = request->getParam(getIdFromName(NAME_CALIBRATED_ROLL_ANGLE))->value().toDouble();
    calibrated_angle_pitch_acc = request->getParam(getIdFromName(NAME_CALIBRATED_PITCH_ANGLE))->value().toDouble();
    printProps();
    saveProps();

    rollOutputPID.reset();
    pitchOutputPID.reset();
    yawOutputPID.reset();

    request->redirect("/");
  });  

  server.on("/Cancel", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Cancel");
    request->redirect("/");
  });  

  server.on("/CalibrateAcc", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/CalibrateAcc");
    calibrateAcc();
    saveProps();
    request->redirect("/");
  });  

  server.on("/WifiOff", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/WifiOff");
    WiFi.mode(WIFI_OFF);
  });  

  server.on("/Defaults", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Defaults");
    
    rollPID.resetToDefault();
    rollPID.print();
    rollPID.save();

    pitchPID.resetToDefault();
    pitchPID.print();
    pitchPID.save();

    yawPID.resetToDefault();
    yawPID.print();    
    yawPID.save();

    rollExpoFactor = defaultRollExpoFactor;
    pitchExpoFactor = defaultPitchExpoFactor;
    yawExpoFactor = defaultYawExpoFactor;
    frontServoCenterOffset = defaultFrontServoCenterOffset;
    backServoCenterOffset = defaultBackServoCenterOffset;
    voltageCorrectionFactor = defaultVoltageCorrectionFactor;
    calibrated_angle_roll_acc = defaultCalibratedRollAngleAcc;
    calibrated_angle_pitch_acc = defaultCalibratedPitchAngleAcc;    
    printProps();
    saveProps();

    rollOutputPID.reset();
    pitchOutputPID.reset();
    yawOutputPID.reset();

    request->redirect("/");
  });    

  server.on("/RequestLatestData", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    //Serial.println("/RequestLatestData");
    request->send(200, "application/json", getLatestData());
  });  

  server.begin();
}


void loop() {
  voltage = LowPassFilter(BATTERY_NOICE_FILTER, readVoltage(), voltage);
  vTaskDelay(1);
}
