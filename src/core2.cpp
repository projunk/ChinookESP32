#include <functions.h>


unsigned long loopTimer, batteryVoltageTimer;
int lowVoltageAlarmCount;
bool previousIsArmed = false;


void setup2() {
  Wire.begin();
  Wire.setClock(1000000);

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

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

  lowVoltageAlarmCount = 0;

  initValues();

  usedUpLoopTime = 0;
  batteryVoltageTimer = millis();
  loopTimer = micros();    
}


void loop2() {
  unsigned long millisTimer = millis();

   //printChannels();

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

  if (signal_detected && actualIsArmed) {    
    writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, frontEsc);
    writeEscPWM(MOTOR_BACK_PWM_CHANNEL, backEsc);
    writeServoPWM(SERVO_FRONT_PWM_CHANNEL, frontServo);
    writeServoPWM(SERVO_BACK_PWM_CHANNEL, backServo);    
  } else {
    writeEscPWM(MOTOR_FRONT_PWM_CHANNEL, MIN_PULSE);
    writeEscPWM(MOTOR_BACK_PWM_CHANNEL, MIN_PULSE);    
    writeServoPWM(SERVO_FRONT_PWM_CHANNEL, MID_CHANNEL + frontServoCenterOffset);
    writeServoPWM(SERVO_BACK_PWM_CHANNEL, MID_CHANNEL + backServoCenterOffset);
  }

  // check battery voltage once per second
  if ((millisTimer - batteryVoltageTimer) > 1000) {
    if (isBootButtonPressed()) {
      waitForBootButtonClicked();
      calibrateESCs();
    }

    batteryVoltageTimer = millisTimer;
//    Serial.println(voltage);     
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

  rtttl::play();

  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  usedUpLoopTime = micros() - loopTimer;
  /*
  Serial.print(usedUpLoopTime);
  Serial.println();
  */
  while(micros() - loopTimer < LOOP_TIME) {
    vTaskDelay(1);
  };
  loopTimer = micros();           
}


void runOnCore2(void *parameter) {
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());
  setup2();
  for (;;) {
    loop2();
  }
}
