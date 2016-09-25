
#ifndef NSGYRO_H
#define NSGYRO_H

#include "Arduino.h"

#define NSGYRO_VREF   5.0
#define NSGYRO_SENS   0.00067
#define NSGYRO_OPAMP  10.1
#define NSGYRO_SIGN   -1


class NsGyroClass
{
private:
  int pin_;
  int intervalMsec_;
  float filter_;
  float yawAD0_;
  float dirAV_;
  float dirAVFiltered_;
  float dir_;
  unsigned long timePrev_;
	
public:
  NsGyroClass(void);
  
  void init(int, int, float);
  void initOrigin(int, int);
  int update();
  float getYawAD0();
  float getDirAV();
  float getDirAVFiltered();
  float getDir();
  unsigned long getTime();
  void setDir(float);
};

extern NsGyroClass NsGyro;

#endif NSGYRO_H
