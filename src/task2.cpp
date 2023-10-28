#include <functions.h>

volatile unsigned long usedUpLoopTimeTask2;
bool previousIsArmed = false;


void task2_setup() {
  pinMode(RECEIVER_PPM_PIN, INPUT_PULLUP);

  // receiver
  initReceiver();
  attachInterrupt(RECEIVER_PPM_PIN, ppmInterruptHandler, FALLING);
  //delay(100);
  
  usedUpLoopTimeTask2 = 0;
}


void task2_loop() {
  int throttleSignal = channel[2]; 
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

  bool actualIsArmed = isArmed();
  if (actualIsArmed && !previousIsArmed) {
    if (isArmingAllowed()) {
      initValues();
      playArmed();
    } else {
      actualIsArmed = false;
    }
  } else if (!actualIsArmed && previousIsArmed) {
    initValues();
    playDisarmed();
  }
  previousIsArmed = actualIsArmed;
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
