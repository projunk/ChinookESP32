#include <pid.h>


double PIDOutput::limitValue(double prmValue) {
  if (prmValue > pid->getMax()) {
    return  pid->getMax();
  } else if (prmValue < -pid->getMax()) {
    return -pid->getMax();
  } else {
    return prmValue;
  }
}


void PIDOutput::calc(double prmGyroAxisInput, double prmSetPoint) {
  output = 0.0;
  error = prmSetPoint - prmGyroAxisInput;

  P = pid->getP() * error;

  I = limitValue(prevI + pid->getI() * (error + prevError) * pid->getLoopTime() / 2);

  D = pid->getD() * (error - prevError) / pid->getLoopTime();
  output = limitValue(P + I + D);

  prevError = error;
  prevI = I;
}
