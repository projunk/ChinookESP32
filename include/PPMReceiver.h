#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H


#include <Arduino.h>


class PPMReceiver {
  private:
   byte interruptPinNr;
   byte nrOfChannels;
   byte maxPeakCount;
   volatile int *values = NULL;
   static PPMReceiver *ppmReceiver;
   volatile int peakCount;
   volatile unsigned long previousTimerValue;
   void interruptHandler();
   static void IRAM_ATTR PPM_ISR(void);

  public:
    PPMReceiver(byte prmInterruptPinNr, byte prmNrOfChannels);
    ~PPMReceiver();
    byte getNrOfChannels() { return nrOfChannels; } 
    int getValue(byte prmChannelNr);  
    void printChannels();
};



#endif
