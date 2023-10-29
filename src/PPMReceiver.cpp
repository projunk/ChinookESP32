#include <PPMReceiver.h>


PPMReceiver *PPMReceiver::ppmReceiver;


#define PPM_PULSE_TRAIN_PERIOD  5000   // micro seconds


void PPMReceiver::interruptHandler() {
  unsigned long actualTimerValue = micros();
  unsigned long actualTimeBetweenPeaks = actualTimerValue - previousTimerValue;
  previousTimerValue = actualTimerValue;
  if (actualTimeBetweenPeaks > PPM_PULSE_TRAIN_PERIOD) {
    peakCount = 0;
  } else {
    if (peakCount < maxPeakCount) {
      values[peakCount] = actualTimeBetweenPeaks;
    }
    peakCount++;
  }
}


void IRAM_ATTR PPMReceiver::PPM_ISR(void) {
  ppmReceiver->interruptHandler(); 
}


PPMReceiver::PPMReceiver(byte prmInterruptPinNr, byte prmNrOfChannels) {
  peakCount = 0;
  previousTimerValue = 0;
  interruptPinNr = prmInterruptPinNr;
  nrOfChannels = prmNrOfChannels;
  maxPeakCount = nrOfChannels+1;
  values = new int[prmNrOfChannels];
  for (int i = 0; i < nrOfChannels; i++) {
    values[i] = 0;
  }

  pinMode(interruptPinNr, INPUT_PULLUP);
  if (ppmReceiver == NULL) {
    ppmReceiver = this;
    attachInterrupt(digitalPinToInterrupt(interruptPinNr), PPM_ISR, FALLING);
  }
}


PPMReceiver::~PPMReceiver() {
  detachInterrupt(digitalPinToInterrupt(interruptPinNr));
  if(ppmReceiver == this) ppmReceiver = NULL;
  delete [] values;
}


int PPMReceiver::getValue(byte prmChannelNr) {
  if (prmChannelNr >= 1 && prmChannelNr <= nrOfChannels) {
    noInterrupts();
    int value = values[prmChannelNr-1];
    interrupts();
    return value;
  } else {
      return 0;
  }
}


void PPMReceiver::printChannels() {
  for (byte i = 1; i <= nrOfChannels; i++) {    
    Serial.print(getValue(i));
    if (i < nrOfChannels) Serial.print("\t");
  }
  Serial.println();
}   
