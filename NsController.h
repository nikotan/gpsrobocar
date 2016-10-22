#ifndef NSCONTROLLER_H
#define NSCONTROLLER_H

#ifdef ARDUINO
  #include "Arduino.h"
#endif


class NsControllerClass
{
  private:
    int   pwmServo0_, pwmSpeed0_;
    byte  pwmServo_[6];
    byte  pwmSpeed_[4];
    float ctrRadius_[4][6];
    float ctrVelocity_[4];
    float ctrRadiusC_, ctrVelocityC_;
    float ctrDistTh_[4];

  public:
    NsControllerClass(void);

    void initPWM(int, int, byte*, byte*);
    void initRadius(int, float*);
    void initVelocity(float*);
    void initCtrC(float, float);
    void initDistTh(float*);

    void reset(byte*, byte*);
    void update(byte*, byte*, float*, float, float, float);
};

extern NsControllerClass NsController;

#endif

