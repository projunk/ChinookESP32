#ifndef PID_H
#define PID_H

#include <arduino.h>


class PID {
    private:
        double defaultP;
        double defaultI;
        double defaultD;
        double defaultMax;
        double P;
        double I;
        double D;
        double max;
        double loopTime;
        String fname;
    public:
        PID() {}
        PID(double prmP, double prmI, double prmD, double prmMax, double prmLoopTime, String prmFName) : defaultP(prmP), defaultI(prmI), defaultD(prmD), defaultMax(prmMax), P(prmP), I(prmI), D(prmD), max(prmMax), loopTime(prmLoopTime), fname(prmFName) {}
        void set(String prmP, String prmI, String prmD, String prmMax) { P = prmP.toDouble(); I = prmI.toDouble() ; D = prmD.toDouble(); max = prmMax.toDouble(); }
        double getP() { return P; }
        double getI() { return I; }
        double getD() { return D; }
        double getMax() { return max; }
        double getLoopTime() { return loopTime; }
        void resetToDefault() { P = defaultP; I = defaultI, D = defaultD, max = defaultMax; }
        void load();
        void save();
        void print() { Serial.print(P); Serial.print("\t"); Serial.print(I); Serial.print("\t"); Serial.print(D); Serial.print("\t"); Serial.print(max); Serial.println(); }
};

class PIDOutput {
    private:
        PID *pid;
        double error;
        double prevError;
        double prevI;
        double P;
        double I;
        double D;
        double output;
        double limitValue(double prmValue);
    public:
        PIDOutput(PID *prmPid) : pid(prmPid), prevError(0.0), prevI(0.0) {}
        double getError() { return error; }
        double getPrevError() { return prevError; }
        double getP() { return P; }
        double getI() { return I; }
        double getD() { return D; }
        double getOutput() { return output; }
        void calc(double prmGyroAxisInput, double prmSetPoint);
        void reset() { prevError = 0.0; prevI = 0.0; }
};


#endif
