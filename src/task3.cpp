#include <functions.h>

volatile unsigned long usedUpLoopTimeTask3;


// fix orientation of axis
MPU6050 mpu6050(angle_180);

double factor = 1.0 / (getLoopTimeHz(LOOP_TIME_TASK3));
volatile double angle_roll = 0.0;
volatile double angle_pitch = 0.0;
volatile double angle_yaw = 0.0;


// kalman filter: https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter
double kalmanAngleRoll = 0.0;
double kalmanUncertaintyAngleRoll = 2.0 * 2.0;
double kalmanAnglePitch = 0.0;
double kalmanUncertaintyAnglePitch = 2.0 * 2.0;
double kalman1DOutput[] = {0.0, 0.0};



void kalmanOneDimFilter(double prmKalmanState, double prmKalmanUncertainty, double prmKalmanInput, double prmKalmanMeasurement) {
  prmKalmanState = prmKalmanState + factor*prmKalmanInput;
  prmKalmanUncertainty = prmKalmanUncertainty + factor * factor * 4.0 * 4.0;
  double prmKalmanGain = prmKalmanUncertainty * 1/(1*prmKalmanUncertainty + 3.0 * 3.0);
  prmKalmanState = prmKalmanState + prmKalmanGain * (prmKalmanMeasurement - prmKalmanState);
  prmKalmanUncertainty = (1 - prmKalmanGain) * prmKalmanUncertainty;
  kalman1DOutput[0] = prmKalmanState; 
  kalman1DOutput[1] = prmKalmanUncertainty;
}


void task3_setup() {
  Wire.begin();
  Wire.setClock(4000000);

  // pwm
  ledcSetup(MOTOR_STEERING_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(MOTOR_LB_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_RB_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_LF_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_RF_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);

  ledcAttachPin(MOTOR_STEERING_PIN,MOTOR_STEERING_PWM_CHANNEL);
  ledcAttachPin(MOTOR_LB_PIN, MOTOR_LB_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RB_PIN, MOTOR_RB_PWM_CHANNEL);
  ledcAttachPin(MOTOR_LF_PIN, MOTOR_LF_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RF_PIN, MOTOR_RF_PWM_CHANNEL);

  writeServoPWM(MOTOR_STEERING_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_LB_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_RB_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_LF_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_RF_PWM_CHANNEL, 0);

  Serial.print("mpu6050.isFound=");
  Serial.println(mpu6050.isFound());

  if (mpu6050.isFound()) {
    mpu6050.setupRegisters();
    Serial.println("calibrating gyro");
    mpu6050.calibrateGyros();
    Serial.println("calibrating gyro done");
  }

  prevAuxChannel = channel[2];
  usedUpLoopTimeTask3 = 0;
}


double complementaryFilter(double prmOldValue, double prmNewValue) {
  return (prmOldValue * 0.8) + (prmNewValue * 0.2);  
}


void task3_loop() {
  int yawChannel = fixChannelDirection(getExpo(channel[0], steerExpoFactor), yawChannelReversed);
  int throttleChannel = fixChannelDirection(channel[1], throttleChannelReversed);
  int auxChannel = fixChannelDirection(channel[2], auxChannelReversed);

  if (((auxChannel > 1750) && (prevAuxChannel < 1250)) || ((auxChannel < 1250) && (prevAuxChannel > 1750))) {
    if (isValidSignalValue(auxChannel)) {
      Serial.println(auxChannel);
      prevAuxChannel = auxChannel;
      incDrivingMode();
      playDrivingMode();
      Serial.print("DrivingMode: ");
      Serial.println(drivingMode);
    }
  }

  mpu6050.readData();

  // calc roll angle
  kalmanOneDimFilter(kalmanAngleRoll, kalmanUncertaintyAngleRoll, mpu6050.getCalibratedRateRoll(), mpu6050.getAngleRollAcc());
  kalmanAngleRoll = kalman1DOutput[0]; 
  kalmanUncertaintyAngleRoll = kalman1DOutput[1];
  angle_roll = kalmanAngleRoll;

  // calc pitch angle
  kalmanOneDimFilter(kalmanAnglePitch, kalmanUncertaintyAnglePitch, mpu6050.getCalibratedRatePitch(), mpu6050.getAnglePitchAcc());
  kalmanAnglePitch = kalman1DOutput[0]; 
  kalmanUncertaintyAnglePitch = kalman1DOutput[1];
  angle_pitch = kalmanAnglePitch;

  // calc yaw angle based on integration; no option to compensate for drift (unless compass sensor is added)
  angle_yaw += mpu6050.getCalibratedRateYaw() * factor;

/*
  Serial.print(mpu6050.getAngleRollAcc());  
  Serial.print("\t");
  Serial.print(mpu6050.getAnglePitchAcc());  
  Serial.print("\t");
  Serial.print(angle_roll);  
  Serial.print("\t");
  Serial.print(angle_pitch);  
  Serial.print("\t");
  Serial.print(angle_yaw);  
  Serial.println();
*/

  gyro_roll_input = complementaryFilter(gyro_roll_input, mpu6050.getCalibratedRateRoll());
  gyro_pitch_input = complementaryFilter(gyro_pitch_input, mpu6050.getCalibratedRatePitch());
  gyro_yaw_input = complementaryFilter(gyro_yaw_input, mpu6050.getCalibratedRateYaw());

  pid_yaw_setpoint = -calcPidSetPoint(yawChannel);

  yawOutputPID.calc(gyro_yaw_input, pid_yaw_setpoint);

  calcMotorValues(yawChannel, throttleChannel);

/*
  Serial.print(gyro_yaw_input);  
  Serial.print("\t");
  Serial.print(pid_yaw_setpoint);  
  Serial.print("\t");
  Serial.print(yawOutputPID.getOutput());  
  Serial.println();
*/

  writeServoPWM(MOTOR_STEERING_PWM_CHANNEL, steerServo);

  if (signal_detected && isArmed()) {  
    writeEscPWM(MOTOR_LB_PWM_CHANNEL, leftEscs);
    writeEscPWM(MOTOR_RB_PWM_CHANNEL, rightEscs);
    writeEscPWM(MOTOR_LF_PWM_CHANNEL, leftEscs);
    writeEscPWM(MOTOR_RF_PWM_CHANNEL, rightEscs);
  } else {
    writeEscPWM(MOTOR_LB_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_RB_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_LF_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_RF_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
  }
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
