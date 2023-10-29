#include <functions.h>


volatile unsigned long usedUpLoopTimeTask2;
volatile bool currentIsArmed = false;
bool previousIsArmed = false;
PPMReceiver *ppm;


void task2_setup() {
  ppm = new PPMReceiver(RECEIVER_PPM_PIN, NR_OF_RECEIVER_CHANNELS);
  usedUpLoopTimeTask2 = 0;
}


void task2_loop() {
  int throttleSignal = ppm->getValue(THROTTLE_CHANNEL); 
  if (isValidSignal(throttleSignal)) {
    bool is_signal_detected = (throttleSignal > SIGNAL_LOST_PULSE);
    if (is_signal_detected) {
      signal_detected_count++;      
      if (signal_detected_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = true;
        playSignalDetected();
      } else if (signal_detected_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_lost_count = 0;        
      }
    } else {
      signal_lost_count++;      
      if (signal_lost_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = false;
        playSignalLost();
      } else if (signal_lost_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_lost_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_detected_count = 0;
      }
    }
  } 

  bool actualIsArmingSwitchTriggered = isArmingSwitchTriggered();
  if (actualIsArmingSwitchTriggered && !previousIsArmed) {
    if (isArmingAllowed()) {
      initValues();
      playArmed();
      currentIsArmed = true;
    }
  } else if (!actualIsArmingSwitchTriggered && previousIsArmed) {
    initValues();
    playDisarmed();
    currentIsArmed = false;
  }
  previousIsArmed = isArmed();
}


void task2(void *parameter) {
  task2_setup();
  unsigned long loopTimer;  
  loopTimer = micros();
  for (;;) {
    task2_loop();
    usedUpLoopTimeTask2 = micros() - loopTimer;
    //Serial.println(usedUpLoopTimeTask2);
    while(micros() - loopTimer < LOOP_TIME_TASK2) {
      vTaskDelay(1);
    };    
    loopTimer = micros();
  }
}
