#include <mpu6050.h>


#define GYRO_CALIBRATION_COUNT      250



int16_t MPU6050::getRawGyroX() {
    switch (orientation) {
      case angle_0:
        return raw_gyroX;
      case angle_90:
        return raw_gyroY;
      case angle_180:
        return -raw_gyroX;
      case angle_270:
        return -raw_gyroY;
      default:
        return 0;
    }
}


int16_t MPU6050::getRawGyroY() {
    switch (orientation) {
      case angle_0:
        return raw_gyroY;
      case angle_90:
        return -raw_gyroX;
      case angle_180:
        return -raw_gyroY;
      case angle_270:
        return raw_gyroX;
      default:
        return 0;
    }
}


int16_t MPU6050::getRawAccX() {
    switch (orientation) {
      case angle_0:
        return raw_accX;
      case angle_90:
        return raw_accY;
      case angle_180:
        return -raw_accX;
      case angle_270:
        return -raw_accY;
      default:
        return 0;
    }
}


int16_t MPU6050::getRawAccY() {
    switch (orientation) {
      case angle_0:
        return raw_accY;
      case angle_90:
        return -raw_accX;
      case angle_180:
        return -raw_accY;
      case angle_270:
        return raw_accX;
      default:
        return 0;
    }
}


bool MPU6050::isFound() {
  static bool _isFound;
  static bool _isChecked = false;
  if (!_isChecked) {
    Wire.beginTransmission(0x68);
    _isFound = (Wire.endTransmission() == 0);
    _isChecked = true;
  }
  return _isFound;
} 


void MPU6050::setupRegisters() {
  if (!isFound()) return;
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


void MPU6050::readData() {   
  if (isFound()) {
    Wire.beginTransmission(0x68);                                       //Start communicating with the MPU-6050
    Wire.write(0x3B);                                                   //Send the requested starting register
    Wire.endTransmission();                                             //End the transmission
    Wire.requestFrom(0x68,14);                                          //Request 14 bytes from the MPU-6050
    while(Wire.available() < 14);                                       //Wait until all the bytes are received
    raw_accX = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_x variable
    raw_accY = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_y variable
    raw_accZ = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the acc_z variable
    raw_temp = Wire.read()<<8 | Wire.read();                                //Add the low and high byte to the temperature variable
    raw_gyroX = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_x variable
    raw_gyroY = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_y variable
    raw_gyroZ = Wire.read()<<8 | Wire.read();                               //Add the low and high byte to the gyro_z variable
  } 
}


void MPU6050::calibrateGyros() {
  rateCalibrationRoll = 0;
  rateCalibrationPitch = 0;
  rateCalibrationYaw = 0;

  if (isFound()) {
    for (int i = 0; i < GYRO_CALIBRATION_COUNT; i++) {
      readData();
      rateCalibrationRoll += getRateRoll();
      rateCalibrationPitch += getRatePitch();
      rateCalibrationYaw += getRateYaw();
      delay(3);
      vTaskDelay(1);
    }
    rateCalibrationRoll /= GYRO_CALIBRATION_COUNT;
    rateCalibrationPitch /= GYRO_CALIBRATION_COUNT;
    rateCalibrationYaw /= GYRO_CALIBRATION_COUNT;
  }

  Serial.print(rateCalibrationRoll);
  Serial.print("\t");
  Serial.print(rateCalibrationPitch);
  Serial.print("\t");
  Serial.print(rateCalibrationYaw);
  Serial.println();
}


void MPU6050::calibrateAcc() {
  // calibrate angle_pitch_acc and angle_roll_acc; model needs to stand horizontal
  double totalAccX = 0.0;
  double totalAccY = 0.0;
  double totalAccZ = 0.0;
  double avgAccX = 0.0;
  double avgAccY = 0.0;
  double avgAccZ = 0.0;

  if (isFound()) {
    for (int i = 0; i < GYRO_CALIBRATION_COUNT; i++) {
      readData();
      totalAccX += getAccX();
      totalAccY += getAccY();
      totalAccZ += getAccZ();
      delay(3);
      vTaskDelay(1);
    }
    avgAccX = totalAccX / GYRO_CALIBRATION_COUNT;
    avgAccY = totalAccY / GYRO_CALIBRATION_COUNT;
    avgAccZ = totalAccZ / GYRO_CALIBRATION_COUNT;
  }

/*
  Serial.print(avgAccX);
  Serial.print("\t");
  Serial.print(avgAccY);
  Serial.print("\t");
  Serial.print(avgAccZ);
  Serial.println();
*/

  calibrationAccX = avgAccX;
  calibrationAccY = avgAccY;
  calibrationAccZ = avgAccZ - 1.0;

  Serial.print(calibrationAccX);
  Serial.print("\t");
  Serial.print(calibrationAccY);
  Serial.print("\t");
  Serial.print(calibrationAccZ);
  Serial.println();
}


void MPU6050::printGyros() {
  Serial.print(getRawGyroX());
  Serial.print("\t");
  Serial.print(getRawGyroY());
  Serial.print("\t");
  Serial.print(getRawGyroZ());  
  Serial.println();
}


void MPU6050::printAccs() {
  Serial.print(getAccX());
  Serial.print("\t");
  Serial.print(getAccY());
  Serial.print("\t");
  Serial.print(getAccZ());  
  Serial.println();
}


void MPU6050::printCalibratedRates() {
  Serial.print(getCalibratedRateRoll());
  Serial.print("\t");
  Serial.print(getCalibratedRatePitch());
  Serial.print("\t");
  Serial.print(getCalibratedRateYaw());  
  Serial.println();
}


void MPU6050::printCalibratedAccs() {
  Serial.print(getCalibratedAccX());
  Serial.print("\t");
  Serial.print(getCalibratedAccY());
  Serial.print("\t");
  Serial.print(getCalibratedAccZ());  
  Serial.println();
}


void MPU6050::printAnglesAcc() {
  Serial.print(getAngleRollAcc());
  Serial.print("\t");
  Serial.print(getAnglePitchAcc());  
  Serial.println();
}
