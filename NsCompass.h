
#ifndef NSCOMPASS_H
#define NSCOMPASS_H

#include "Arduino.h"


class NsCompassClass
{
private:
  int pin0_;
  int pin1_;
  int pin2_;
  int intervalMsec_;
  int dirId_;
  float dir_;
  unsigned long timePrev_;
  
public:
  NsCompassClass(void);
  
  void init(int, int, int, int);
  int update();
  int getDirId();
  float getDir();
  unsigned long getTime();
};

extern NsCompassClass NsCompass;

#endif NSCOMPASS_H
