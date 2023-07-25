#include <functions.h>

volatile unsigned long usedUpLoopTimeTask3;



void task3_setup() {
  Wire.begin();
  Wire.setClock(1000000);

  // pwm
  ledcSetup(SERVO_FRONT_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(SERVO_BACK_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(MOTOR_FRONT_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_BACK_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);

  ledcAttachPin(SERVO_FRONT_PIN, SERVO_FRONT_PWM_CHANNEL);
  ledcAttachPin(SERVO_BACK_PIN, SERVO_BACK_PWM_CHANNEL);
  ledcAttachPin(MOTOR_FRONT_PIN, MOTOR_FRONT_PWM_CHANNEL);
  ledcAttachPin(MOTOR_BACK_PIN, MOTOR_BACK_PWM_CHANNEL);

  writeServoPWM(SERVO_FRONT_PWM_CHANNEL, 0);
  writeServoPWM(SERVO_BACK_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_BACK_PWM_CHANNEL, 0);

  initValues();

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

  usedUpLoopTimeTask3 = 0;
}


void task3_loop() {
  flightMode = getFlightMode();

  int rollChannel = fixChannelDirection(getExpo(channel[0], rollExpoFactor), rollChannelReversed);
  int pitchChannel = fixChannelDirection(getExpo(channel[1], pitchExpoFactor), pitchChannelReversed);
  int throttleChannel = fixChannelDirection(channel[2], throttleChannelReversed);
  int yawChannel = fixChannelDirection(getExpo(channel[3], yawExpoFactor), yawChannelReversed);

  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  calcAngles();

  gyro_roll_input = calcDegreesPerSecond(gyro_roll_input, gyro_roll.get());
  gyro_pitch_input = calcDegreesPerSecond(gyro_pitch_input, gyro_pitch.get());
  gyro_yaw_input = calcDegreesPerSecond(gyro_yaw_input, gyro_yaw.get());

  calcLevelAdjust(flightMode);
  
  pid_roll_setpoint = calcPidSetPoint(rollChannel, roll_level_adjust);
  pid_pitch_setpoint = calcPidSetPoint(pitchChannel, pitch_level_adjust);
  pid_yaw_setpoint = 0.0;
  if (throttleChannel > 1050) pid_yaw_setpoint = calcPidSetPoint(yawChannel, 0.0);

  rollOutputPID.calc(gyro_roll_input, pid_roll_setpoint);
  pitchOutputPID.calc(gyro_pitch_input, pid_pitch_setpoint);
  yawOutputPID.calc(gyro_yaw_input, pid_yaw_setpoint);

  if (throttleChannel > MAX_THROTTLE) throttleChannel = MAX_THROTTLE;
  frontEsc = limitEsc(throttleChannel - pitchOutputPID.getOutput());
  backEsc = limitEsc(throttleChannel + pitchOutputPID.getOutput());
  frontServo = limitServo(MID_CHANNEL - rollOutputPID.getOutput() - yawOutputPID.getOutput() + frontServoCenterOffset);
  backServo = limitServo(MID_CHANNEL + rollOutputPID.getOutput() - yawOutputPID.getOutput() + backServoCenterOffset);
  

  if (signal_detected && isArmed()) {    
    if (switchB.readPos() == 2) {
      frontEsc = MIN_PULSE; 
    }
    if (switchD.readPos() == 2) {
      backEsc = MIN_PULSE;
    }
  } else {
    frontEsc = MIN_PULSE;
    backEsc = MIN_PULSE;    
    frontServo = MID_CHANNEL + frontServoCenterOffset;
    backServo = MID_CHANNEL + backServoCenterOffset;
  }

  writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, frontEsc);
  writeEscPWM(MOTOR_BACK_PWM_CHANNEL, backEsc);
  writeServoPWM(SERVO_FRONT_PWM_CHANNEL, frontServo);
  writeServoPWM(SERVO_BACK_PWM_CHANNEL, backServo);    
}


void task3(void *parameter) {
  task3_setup();
  unsigned long loopTimer;  
  loopTimer = micros();
  for (;;) {
    task3_loop();
    usedUpLoopTimeTask3 = micros() - loopTimer;
    //Serial.println(usedUpLoopTimeTask3);
    while(micros() - loopTimer < LOOP_TIME_TASK3) {
      vTaskDelay(1);
    };    
    loopTimer = micros();
  }
}
