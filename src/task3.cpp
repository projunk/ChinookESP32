#include <functions.h>

volatile unsigned long usedUpLoopTimeTask3;


// fix orientation of axis
MPU6050 mpu6050(angle_90);


const double factor = 1.0 / (getLoopTimeHz(LOOP_TIME_TASK3));
volatile double kalmanRollAngle = 0.0;
volatile double kalmanPitchAngle = 0.0;
volatile double yawAngle = 0.0;

const double desiredRollAngleFactor = 2*FLIGHT_MODE_MAX_ROLL_ANGLE/(MAX_PULSE-MIN_PULSE);
const double desiredPitchAngleFactor = 2*FLIGHT_MODE_MAX_PITCH_ANGLE/(MAX_PULSE-MIN_PULSE);

const double desiredRateFactor = 0.15;  // sensitivity


// kalman filter: https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter
double kalmanUncertaintyAngleRoll = 2.0*2.0;
double kalmanUncertaintyAnglePitch = 2.0*2.0;
double kalman1DOutput[] = {0.0,0.0};


int rollChannel = 0;
int pitchChannel = 0;
int throttleChannel = 0;
int yawChannel = 0;


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


int complementaryFilter(int prmOldValue, int prmNewValue) {
  return int(round(((double)prmOldValue * 0.9) + ((double)prmNewValue * 0.1)));  
}


void task3_loop() {
  flightMode = getFlightMode();

  rollChannel = complementaryFilter(rollChannel, fixChannelDirection(getExpo(ppm->getValue(ROLL_CHANNEL), rollExpoFactor), rollChannelReversed));
  pitchChannel = complementaryFilter(pitchChannel, fixChannelDirection(getExpo(ppm->getValue(PITCH_CHANNEL), pitchExpoFactor), pitchChannelReversed));
  throttleChannel = complementaryFilter(throttleChannel, fixChannelDirection(ppm->getValue(THROTTLE_CHANNEL), throttleChannelReversed));
  yawChannel = complementaryFilter(yawChannel, fixChannelDirection(getExpo(ppm->getValue(YAW_CHANNEL), yawExpoFactor), yawChannelReversed));

  mpu6050.readData();

  // calc roll angle
  kalmanOneDimFilter(kalmanRollAngle, kalmanUncertaintyAngleRoll, mpu6050.getCalibratedRateRoll(), mpu6050.getAngleRollAcc());
  kalmanRollAngle = kalman1DOutput[0]; 
  kalmanUncertaintyAngleRoll = kalman1DOutput[1];

  // calc pitch angle
  kalmanOneDimFilter(kalmanPitchAngle, kalmanUncertaintyAnglePitch, mpu6050.getCalibratedRatePitch(), mpu6050.getAnglePitchAcc());
  kalmanPitchAngle = kalman1DOutput[0]; 
  kalmanUncertaintyAnglePitch = kalman1DOutput[1];

  // calc yaw angle based on integration; no option to compensate for drift (unless compass sensor is added)
  yawAngle += mpu6050.getCalibratedRateYaw() * factor;

  gyro_roll_input = complementaryFilter(gyro_roll_input, mpu6050.getCalibratedRateRoll());
  gyro_pitch_input = complementaryFilter(gyro_pitch_input, mpu6050.getCalibratedRatePitch());
  gyro_yaw_input = complementaryFilter(gyro_yaw_input, mpu6050.getCalibratedRateYaw());

  desiredRollAngle = desiredRollAngleFactor * (rollChannel - MID_CHANNEL);
  desiredPitchAngle = desiredPitchAngleFactor * (pitchChannel - MID_CHANNEL);

  rollAngleOutputPID.calc(kalmanRollAngle, desiredRollAngle);
  pitchAngleOutputPID.calc(kalmanPitchAngle, desiredPitchAngle);

  switch (flightMode) {
    case fmAutoLevel: {
        desiredRollRate = rollAngleOutputPID.getOutput();
        desiredPitchRate = pitchAngleOutputPID.getOutput();      
      } 
      break;   
    default: {
        desiredRollRate = desiredRateFactor * (rollChannel - MID_CHANNEL);    
        desiredPitchRate = desiredRateFactor * (pitchChannel - MID_CHANNEL);
      }
    break;      
  }
  desiredYawRate = desiredRateFactor * (yawChannel - MID_CHANNEL);
  
  rollRateOutputPID.calc(gyro_roll_input, desiredRollRate);
  pitchRateOutputPID.calc(gyro_pitch_input, desiredPitchRate);
  yawRateOutputPID.calc(gyro_yaw_input, desiredYawRate);

  if (throttleChannel > MAX_THROTTLE) throttleChannel = MAX_THROTTLE;
  frontEsc = limitEsc(throttleChannel + pitchRateOutputPID.getOutput());
  backEsc = limitEsc(throttleChannel - pitchRateOutputPID.getOutput());
  frontServo = limitServo(MID_CHANNEL + rollRateOutputPID.getOutput() - yawRateOutputPID.getOutput() + frontServoCenterOffset);
  backServo = limitServo(MID_CHANNEL - rollRateOutputPID.getOutput() - yawRateOutputPID.getOutput() + backServoCenterOffset);
  

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

//  Serial.print(kalmanRollAngle);
//  Serial.print("\t");
//  Serial.print(gyro_roll_input);
//  Serial.print("\t");
//  Serial.print(desiredRollRate);
//  Serial.print("\t");
//  Serial.print(rollChannel);
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getP());
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getI());
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getD());
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getPrevError());
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getError());
//  Serial.print("\t");
//  Serial.print(rollRateOutputPID.getOutput());
//  Serial.print("\t");
//  Serial.print(frontEsc);
//  Serial.print("\t");
//  Serial.print(backEsc);
//  Serial.print("\t");
//  Serial.print(frontServo);
//  Serial.print("\t");
//  Serial.print(backServo);
//  Serial.print("\t");  
//  Serial.println();

//ppm->printChannels();

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
