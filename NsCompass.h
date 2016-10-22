
#ifndef NSCOMPASS_H
#define NSCOMPASS_H

#include "Arduino.h"


class NsCompassClass
{
private:
  byte pin0_;
  byte pin1_;
  byte pin2_;
  int intervalMsec_;
  byte dirId_;
  float dir_;
  unsigned long timePrev_;

  byte* dirIdHist_;
  int   dirIdHistLen_;
  byte  dirIdHistPos_;
  byte  dirIdCnts_[8];
  byte  dirIdCntsMax_;
  byte  dirIdCntsMaxId_;

public:
  NsCompassClass(void);
  
  void init(byte, byte, byte, int, int);
  int update();
  byte getDirId();
  byte getDirIdFiltered();
  float getDir();
  unsigned long getTime();
};

extern NsCompassClass NsCompass;

#endif

