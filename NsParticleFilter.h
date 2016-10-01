
#ifndef NSPARTICLEFILTER_H
#define NSPARTICLEFILTER_H

#include "Arduino.h"


class NsParticle
{
public:
  float x;
  float y;
  float v;
  float w;

  NsParticle(){
    x = 0.0f;
    y = 0.0f;
    v = 0.0f;
    w = 0.0f;
  }

  void set(const NsParticle& obj){
    x = obj.x;
    y = obj.y;
    v = obj.v;
    w = obj.w;
  }
};


class NsParticleFilterClass
{
private:
  int numParticles_;
  float sigmaV_;
  float sigmaP_;
  float initR_;

  NsParticle* particles_;
  NsParticle* particles2_;
  float* weights_;
  unsigned long timePrev_;
  NsParticle p_;

  float x_;
  float y_;
  float sigma_;

public:
  NsParticleFilterClass(void);
  
  void init(int, float, float, float);
  void initParticle();
  void predict(unsigned long, float);
  bool estimate(float, float);
  void resample();
  float getX();
  float getY();
  float getSigma();
};

extern NsParticleFilterClass NsParticleFilter;

#endif

