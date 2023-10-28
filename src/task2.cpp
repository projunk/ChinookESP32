#include <functions.h>

volatile unsigned long usedUpLoopTimeTask2;



void task2_setup() {
  signal_micros_timer_value = 0;

  pinMode(RECEIVER_STEER_PIN, INPUT);
  pinMode(RECEIVER_THROTTLE_PIN, INPUT);
  pinMode(RECEIVER_SPEED_MODE_PIN, INPUT);  

  attachInterrupt(RECEIVER_STEER_PIN, handler_channel_1, CHANGE);
  attachInterrupt(RECEIVER_THROTTLE_PIN, handler_channel_2, CHANGE);
  attachInterrupt(RECEIVER_SPEED_MODE_PIN, handler_channel_3, CHANGE);
  
  prev_signal_detected = signal_detected;
  usedUpLoopTimeTask2 = 0;
}


void task2_loop() {
  if (signal_detected) {
    if (!prev_signal_detected) {
      playSignalDetected();
    }
  } else {
    if (prev_signal_detected) {
      playSignalLost();
    }
  }
  checkIsArmed();
  prev_signal_detected = signal_detected;  
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
