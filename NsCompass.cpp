
#include "NsCompass.h"


NsCompassClass::NsCompassClass(void)
{
}

void NsCompassClass::init(int pin0, int pin1, int pin2, int intervalMsec)
{
  pin0_ = pin0;
  pin1_ = pin1;
  pin2_ = pin2;
  intervalMsec_ = intervalMsec;
  dirId_ = 0;
  dir_ = 0.0f;
  timePrev_ = 0;

  pinMode(pin0_, INPUT);
  pinMode(pin1_, INPUT);
  pinMode(pin2_, INPUT);
}

int NsCompassClass::update()
{
  unsigned long timeNow = millis();
  if( timeNow > timePrev_ + intervalMsec_ ){
    byte val0 = digitalRead(pin0_);
    byte val1 = digitalRead(pin1_);
    byte val2 = digitalRead(pin2_);
    if( val0==HIGH ){
      if( val1==HIGH ){
        if( val2==HIGH){
          dirId_ = 7;
        }else{
          dirId_ = 0;
        }
      }else{
        if( val2==HIGH){
          dirId_ = 6;
        }else{
          dirId_ = 5;
        }
      }
    }else{
      if( val1==HIGH ){
        if( val2==HIGH){
          dirId_ = 2;
        }else{
          dirId_ = 1;
        }
      }else{
        if( val2==HIGH){
          dirId_ = 3;
        }else{
          dirId_ = 4;
        }
      }
    }

    // correction
    dirId_ += 3;
    if( dirId_ >= 8 ){
      dirId_ -= 8;
    }
    dir_ = dirId_ * 45.0f + 22.5f;

    int msec = timeNow - timePrev_;
    timePrev_ = timeNow;
    return msec;
  }
  return 0;
}

int NsCompassClass::getDirId()
{
  return dirId_;
}

float NsCompassClass::getDir()
{
  return dir_;
}

unsigned long NsCompassClass::getTime()
{
  return timePrev_;
}

NsCompassClass NsCompass;
