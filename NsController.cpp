#include "NsController.h"


NsControllerClass::NsControllerClass(void)
{
}

void NsControllerClass::initPWM(int pwmServo0, int pwmSpeed0, byte *pwmServo, byte *pwmSpeed)
{
  pwmServo0_ = pwmServo0;
  pwmSpeed0_ = pwmSpeed0;

  for(int i=0; i<6; i++){
    pwmServo_[i] = pwmServo[i];
  }
  for(int i=0; i<4; i++){
    pwmSpeed_[i] = pwmSpeed[i];
  }
}

void NsControllerClass::initRadius(int idx, float *ctrRadius)
{
  for(int i=0; i<6; i++){
    ctrRadius_[idx][i] = ctrRadius[i];
  }
}

void NsControllerClass::initVelocity(float *ctrVelocity)
{
  for(int i=0; i<4; i++){
    ctrVelocity_[i] = ctrVelocity[i];
  }
}

void NsControllerClass::initCtrC(float ctrRadiusC, float ctrVelocityC)
{
  ctrRadiusC_ = ctrRadiusC;
  ctrVelocityC_ = ctrVelocityC;
}

void NsControllerClass::initDistTh(float *ctrDistTh)
{
  for(int i=0; i<4; i++){
    ctrDistTh_[i] = ctrDistTh[i];
  }
}


void NsControllerClass::reset(byte *bPwmServo, byte *bPwmSpeed)
{
  *bPwmServo = pwmServo0_;
  *bPwmSpeed = pwmSpeed0_;
}

void NsControllerClass::update(
  byte *bPwmServo, byte *bPwmSpeed, float *velocity,
  float radius, float distGPS, float error
)
{
  int speedIdx = -1;
  if(distGPS > ctrDistTh_[0]) {
    speedIdx = -1;
  } else if(distGPS > ctrDistTh_[1]) {
    speedIdx = 0;
  } else if(distGPS > ctrDistTh_[2]) {
    speedIdx = 1;
  } else if(distGPS > ctrDistTh_[3]) {
    speedIdx = 2;
  } else {
    speedIdx = 3;
  }

  if(speedIdx < 0){
    *bPwmSpeed = pwmSpeed0_;
    *velocity = 0.0f;
  } else {
    *bPwmSpeed = pwmSpeed0_ + pwmSpeed_[speedIdx];
    *velocity = ctrVelocity_[speedIdx] * ctrVelocityC_;
  }

  if(speedIdx >= 0){
    float rAbs = abs(radius);
    int servoIdx;
    for(servoIdx=0; servoIdx<6; servoIdx++){
      if(rAbs > ctrRadius_[speedIdx][servoIdx] * ctrRadiusC_){
        break;
      }
    }
    servoIdx--;
    if(servoIdx < 0){
      *bPwmServo = pwmServo0_;
    } else {
      if(radius > 0){
        *bPwmServo = pwmServo0_ + pwmServo_[servoIdx];
      } else {
        *bPwmServo = pwmServo0_ - pwmServo_[servoIdx];
      }
    }
  }
}


NsControllerClass NsController;

