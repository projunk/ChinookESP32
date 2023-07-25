#include <functions.h>

volatile unsigned long usedUpLoopTimeTask4;


void task4_setup() {  
  pinMode(BUZZER_PIN, OUTPUT);

  usedUpLoopTimeTask4 = 0;
}


void task4_loop() {
  rtttl::play();
}


void task4(void *parameter) {
  task4_setup();
  unsigned long loopTimer;  
  loopTimer = micros();
  for (;;) {
    task4_loop();
    usedUpLoopTimeTask4 = micros() - loopTimer;
    //Serial.println(usedUpLoopTimeTask4);
    while(micros() - loopTimer < LOOP_TIME_TASK4) {
      vTaskDelay(1);
    };    
    loopTimer = micros();
  }
}
