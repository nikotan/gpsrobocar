
#include "NsGyro.h"


NsGyroClass::NsGyroClass(void)
{
}

void NsGyroClass::init(int pin, int intervalMsec, float filter)
{
  pin_ = pin;
  intervalMsec_ = intervalMsec;
  filter_ = filter;
  yawAD0_ = 0.0f;
  dirAV_ = 0.0f;
  dir_ = 0.0f;
  dirAVFiltered_ = 0.0f;
  timePrev_ = 0;
}

void NsGyroClass::initOrigin(int count, int intervalMsec)
{
  dir_ = 0.0f;
  yawAD0_ = 0.0f;
  int cnt = 0;
  while(cnt < count){
    yawAD0_ += analogRead(pin_);
    cnt++;
    delay(intervalMsec);
  }
  yawAD0_ /= cnt;
}

int NsGyroClass::update()
{
  unsigned long timeNow = millis();
  if( timeNow > timePrev_ + intervalMsec_ ){
    int ad = analogRead(pin_);
    dirAV_ = (ad - yawAD0_) * NSGYRO_VREF / 1024 / NSGYRO_SENS / NSGYRO_OPAMP * NSGYRO_SIGN;
    int msec = 0;
    if(timePrev_ > 0) {
      msec = timeNow - timePrev_;
      dir_ += dirAV_ * msec / 1000.0f;
      if(dir_ >= 360.0f){
        dir_ -= 360.0f;
      } else if(dir_ < 0.0f){
        dir_ += 360.0f;
      }
      dirAVFiltered_ = dirAV_ * filter_ + dirAVFiltered_ * (1.0 - filter_);
    } else {
      dirAVFiltered_ = dirAV_;
    }
    timePrev_ = timeNow;
    return msec;
  }
  return 0;
}

float NsGyroClass::getYawAD0()
{
  return yawAD0_;
}

float NsGyroClass::getDirAV()
{
  return dirAV_;
}

float NsGyroClass::getDirAVFiltered()
{
  return dirAVFiltered_;
}

float NsGyroClass::getDir()
{
  return dir_;
}

unsigned long NsGyroClass::getTime()
{
  return timePrev_;
}

void NsGyroClass::setDir(float dir)
{
  dir_ = dir;
}

NsGyroClass NsGyro;
