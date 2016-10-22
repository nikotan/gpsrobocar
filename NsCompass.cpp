
#include "NsCompass.h"


NsCompassClass::NsCompassClass(void)
{
}

void NsCompassClass::init(byte pin0, byte pin1, byte pin2, int intervalMsec, int histLen)
{
  pin0_ = pin0;
  pin1_ = pin1;
  pin2_ = pin2;
  intervalMsec_ = intervalMsec;
  dirIdHistLen_ = histLen;

  dirId_ = 0;
  dir_ = 0.0f;
  timePrev_ = 0;

  dirIdHist_ = (byte*)malloc(sizeof(byte) * dirIdHistLen_);
  for(int i=0; i<dirIdHistLen_; i++){
    dirIdHist_[i] = 0;
  }
  dirIdHistPos_ = 0;
  for(int i=0; i<8; i++){
    dirIdCnts_[i] = 0;
  }
  dirIdCnts_[0] = dirIdHistLen_;

  pinMode(pin0_, INPUT);
  pinMode(pin1_, INPUT);
  pinMode(pin2_, INPUT);
}

int NsCompassClass::update()
{
  unsigned long timeNow = millis();
  if( timeNow >= timePrev_ + intervalMsec_ ){
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

    // update history
    dirIdHistPos_++;
    if(dirIdHistPos_ >= dirIdHistLen_){
      dirIdHistPos_ = 0;
    }
    dirIdCnts_[dirIdHist_[dirIdHistPos_]]--;
    dirIdHist_[dirIdHistPos_] = dirId_;
    dirIdCnts_[dirId_]++;
    dirIdCntsMax_ = 0;
    dirIdCntsMaxId_ = -1;
    for(int i=0; i<8; i++){
      if(dirIdCnts_[i] > dirIdCntsMax_){
        dirIdCntsMax_ = dirIdCnts_[i];
        dirIdCntsMaxId_ = i;
      }
    }

    // update time
    int msec = timeNow - timePrev_;
    timePrev_ = timeNow;
    return msec;
  } else if(timeNow < timePrev_) {
    timePrev_ = timeNow;
  }
  return 0;
}

byte NsCompassClass::getDirId()
{
  return dirId_;
}

byte NsCompassClass::getDirIdFiltered()
{
  return dirIdCntsMaxId_;
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
