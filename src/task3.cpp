#include <functions.h>

volatile unsigned long usedUpLoopTimeTask3;


// fix orientation of axis
MPU6050 mpu6050(angle_90);


const double factor = 1.0 / (getLoopTimeHz(LOOP_TIME_TASK3));
volatile double angle_roll = 0.0;
volatile double angle_pitch = 0.0;
volatile double angle_yaw = 0.0;


// kalman filter: https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter
double kalmanAngleRoll = 0.0;
double kalmanUncertaintyAngleRoll = 2.0*2.0;
double kalmanAnglePitch = 0.0;
double kalmanUncertaintyAnglePitch = 2.0*2.0;
double kalman1DOutput[] = {0.0,0.0};



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

  Serial.print("mpu6050.isFound=");
  Serial.println(mpu6050.isFound());

  if (mpu6050.isFound()) {
    mpu6050.setupRegisters();
    Serial.println("calibrating gyro");
    mpu6050.calibrateGyros();
    Serial.println("calibrating gyro done");
  }

  usedUpLoopTimeTask3 = 0;
}


double complementaryFilter(double prmOldValue, double prmNewValue) {
  return (prmOldValue * 0.8) + (prmNewValue * 0.2);  
}


void task3_loop() {
  flightMode = getFlightMode();

  int rollChannel =  fixChannelDirection(getExpo(channel[0], rollExpoFactor), rollChannelReversed);
  int pitchChannel = fixChannelDirection(getExpo(channel[1], pitchExpoFactor), pitchChannelReversed);
  int throttleChannel = fixChannelDirection(channel[2], throttleChannelReversed);
  int yawChannel = fixChannelDirection(getExpo(channel[3], yawExpoFactor), yawChannelReversed);

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

  gyro_roll_input = complementaryFilter(gyro_roll_input, mpu6050.getCalibratedRateRoll());
  gyro_pitch_input = complementaryFilter(gyro_pitch_input, mpu6050.getCalibratedRatePitch());
  gyro_yaw_input = complementaryFilter(gyro_yaw_input, mpu6050.getCalibratedRateYaw());

  calcLevelAdjust(flightMode);
  
  pid_roll_setpoint = calcPidSetPoint(rollChannel, roll_level_adjust);
  pid_pitch_setpoint = calcPidSetPoint(pitchChannel, pitch_level_adjust);
  pid_yaw_setpoint = 0.0;
  if (throttleChannel > 1050) pid_yaw_setpoint = calcPidSetPoint(yawChannel, 0.0);

  rollOutputPID.calc(gyro_roll_input, pid_roll_setpoint);
  pitchOutputPID.calc(gyro_pitch_input, pid_pitch_setpoint);
  yawOutputPID.calc(gyro_yaw_input, pid_yaw_setpoint);

//  Serial.print(angle_roll);
//  Serial.print("\t");
//  Serial.print(gyro_roll_input);
//  Serial.print("\t");
//  Serial.print(rollChannel);
//  Serial.print("\t");
//  Serial.print(roll_level_adjust);
//  Serial.print("\t");
//  Serial.print(pid_roll_setpoint);
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getP());
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getI());
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getD());
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getPrevError());
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getError());
//  Serial.print("\t");
//  Serial.print(rollOutputPID.getOutput());
//  Serial.println();

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
