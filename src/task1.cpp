#include <functions.h>

volatile unsigned long usedUpLoopTimeTask1;
unsigned long voltageTimer, ledTimer;
int lowVoltageAlarmCount;


void task1_setup() {
  Serial.begin(115200);  

  pinMode(VOLTAGE_SENSOR_PIN, ANALOG);
  pinMode(CURRENT_SENSOR_PIN, ANALOG);
  pinMode(LED_PIN, OUTPUT);

  nrOfLEDStrips = getNrOfLEDStrips();
  hasGPSSensor = getHasGPSSensor();
  hasCurrentSensor = getHasCurrentSensor();

  initLEDs();  

  voltage = readVoltage();
  lowVoltageAlarmCount = 0;

  Serial.print("Voltage [V]: ");
  Serial.println(getVoltageStr());
  Serial.println();

  voltageTimer = millis();
  ledTimer = millis();

  usedUpLoopTimeTask1 = 0;

  hs.begin(9600);
}


void task1_loop() {
  unsigned long millisTimer = millis();

  wiFiSignalStrength = getWiFiSignalStrength();
  voltage = LowPassFilter(VOLTAGE_NOICE_FILTER, readVoltage(), voltage);
  current = LowPassFilter(CURRENT_NOICE_FILTER, readCurrent(), current);  
  getGPSData();

  // check battery voltage once per second
  if ((millisTimer - voltageTimer) > 1000) {
    voltageTimer = millisTimer;
//    Serial.println(batteryVoltage);         
    if (voltage < LOW_VOLTAGE_ALARM) {
//      Serial.println("lowVoltageAlarmCount++");       
      lowVoltageAlarmCount++;
      if (lowVoltageAlarmCount >= 10) {
//        Serial.println("lowVoltageAlarm!!"); 
        playLowVoltageAlarm(); 
      }
    } else {
      lowVoltageAlarmCount = 0;
    }
  }    

  // update LEDs
  if ((millisTimer - ledTimer) > 250) {
    ledTimer = millisTimer;
    updateLEDs();
  }
}


void task1(void *parameter) {
  task1_setup();
  unsigned long loopTimer;  
  loopTimer = micros();
  for (;;) {
    task1_loop();
    usedUpLoopTimeTask1 = micros() - loopTimer;
    //Serial.println(usedUpLoopTimeTask1);
    while(micros() - loopTimer < LOOP_TIME_TASK1) {
      vTaskDelay(1);
    };    
    loopTimer = micros();
  }
}
