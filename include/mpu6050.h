#ifndef MPU6050_H
#define MPU6050_H


#include <Arduino.h>
#include <Wire.h>


enum Orientation { angle_0, angle_90, angle_180, angle_270 };

class MPU6050 {
  private:
    Orientation orientation;
    volatile int16_t raw_gyroX;
    volatile int16_t raw_gyroY;
    volatile int16_t raw_gyroZ;
    volatile int16_t raw_accX;
    volatile int16_t raw_accY;
    volatile int16_t raw_accZ;
    volatile int16_t raw_temp;
    double rateCalibrationRoll = 0.0;
    double rateCalibrationPitch = 0.0;
    double rateCalibrationYaw = 0.0;  
    double calibrationAccX = 0.0;
    double calibrationAccY = 0.0;
    double calibrationAccZ = 0.0;
    double toDegrees(double prmRadians) { return prmRadians * 180.0 / PI; }
    double sqr(double x) { return x*x; }
  public:
    MPU6050(Orientation prmOrientation) : orientation(prmOrientation), raw_gyroX(0), raw_gyroY(0), raw_gyroZ(0), raw_accX(0), raw_accY(0), raw_accZ(0), raw_temp(0) { ; }    
    void setupRegisters();
    bool isFound();
    int16_t getRawGyroX();
    int16_t getRawGyroY();
    int16_t getRawGyroZ() { return raw_gyroZ; }
    int16_t getRawAccX();
    int16_t getRawAccY();
    int16_t getRawAccZ() { return raw_accZ; }
    int16_t getRawTemp() { return raw_temp; }    
    double getRateRoll() { return (double) getRawGyroY()/65.5; }   // [°/s]
    double getRatePitch() { return (double) getRawGyroX()/65.5; }  // [°/s]
    double getRateYaw() { return (double) getRawGyroZ()/65.5; }    // [°/s]
    double getCalibratedRateRoll() { return getRateRoll()-rateCalibrationRoll; }
    double getCalibratedRatePitch() { return getRatePitch()-rateCalibrationPitch; }
    double getCalibratedRateYaw() { return getRateYaw()-rateCalibrationYaw; }
    double getAccX() { return (double) getRawAccX()/4096; } 
    double getAccY() { return (double) getRawAccY()/4096; } 
    double getAccZ() { return (double) getRawAccZ()/4096; }   
    void  setCalibrationAccX(double prmCalibrationAccX) { calibrationAccX = prmCalibrationAccX; }
    void  setCalibrationAccY(double prmCalibrationAccY) { calibrationAccY = prmCalibrationAccY; }
    void  setCalibrationAccZ(double prmCalibrationAccZ) { calibrationAccZ = prmCalibrationAccZ; }
    double getCalibrationAccX() { return calibrationAccX; } 
    double getCalibrationAccY() { return calibrationAccY; } 
    double getCalibrationAccZ() { return calibrationAccZ; }   
    double getCalibratedAccX() { return getAccX()-calibrationAccX; } 
    double getCalibratedAccY() { return getAccY()-calibrationAccY; } 
    double getCalibratedAccZ() { return getAccZ()-calibrationAccZ; }   
    double getAngleRollAcc() { return toDegrees(-atan(getCalibratedAccX()/sqrt(sqr(getCalibratedAccX())+sqr(getCalibratedAccZ())))); }
    double getAnglePitchAcc() { return toDegrees(atan(getCalibratedAccY()/sqrt(sqr(getCalibratedAccY())+sqr(getCalibratedAccZ())))); }
    void readData();
    void calibrateGyros();
    void calibrateAcc();
    double getTempCelsius() { return 36.53 + (double) getRawTemp()/340.0; }
    void printGyros();
    void printAccs();
    void printCalibratedRates();
    void printCalibratedAccs();
    void printAnglesAcc();
};



#endif
