
#include "NsParticleFilter.h"
#include "NsRandom.h"

NsParticleFilterClass::NsParticleFilterClass(void)
{
}

void NsParticleFilterClass::init(int numParticles, float sigmaV, float sigmaP, float initR)
{
  numParticles_ = numParticles;
  sigmaV_ = sigmaV;
  sigmaP_ = sigmaP;
  initR_  = initR;

  particles_  = (NsParticle*)malloc(sizeof(NsParticle) * numParticles_);
  particles2_ = (NsParticle*)malloc(sizeof(NsParticle) * numParticles_);
  weights_ = (float*)malloc(sizeof(float) * numParticles_);
  timePrev_ = 0;

  initParticle();
}

void NsParticleFilterClass::initParticle()
{
  x_ = 0.0f;
  y_ = 0.0f;
  sigma_ = 0.0f;

  for(int i=0; i<numParticles_; i++){
    float r = NsRandom.nextFloat() * initR_;
    float t = NsRandom.nextFloat() * 2.0f * M_PI;
    particles_[i].x = r * cos(t);
    particles_[i].y = r * sin(t);
    particles_[i].v = sigmaV_ * NsRandom.nextNormalDist();
    particles_[i].w = 1.0f / numParticles_;

    x_ += particles_[i].x * particles_[i].w;
    y_ += particles_[i].y * particles_[i].w;
  }
}

void NsParticleFilterClass::predict(unsigned long timeNow, float dir)
{
  if(timePrev_ > 0 && timeNow > timePrev_){
    float fx = sin(dir * M_PI / 180.0f) * (timeNow - timePrev_) / 1000.0f;
    float fy = cos(dir * M_PI / 180.0f) * (timeNow - timePrev_) / 1000.0f;
    float ve = 0.0f;
    x_ = 0.0f;
    y_ = 0.0f;
    for(int i=0; i<numParticles_; i++){
      ve = NsRandom.nextNormalDist() * sigmaV_;
      particles_[i].x += (particles_[i].v + ve / 2.0) * fx;
      particles_[i].y += (particles_[i].v + ve / 2.0) * fy;
      particles_[i].v += ve;

      x_ += particles_[i].x * particles_[i].w;
      y_ += particles_[i].y * particles_[i].w;
    }
  }
  timePrev_ = timeNow;
}

bool NsParticleFilterClass::estimate(float x, float y)
{
  float wsum = 0.0f;
  float dx, dy;
  for(int i=0; i<numParticles_; i++){
    dx = particles_[i].x - x;
    dy = particles_[i].y - y;
    particles_[i].w = exp(-1.0f * (dx * dx + dy * dy) / 2.0f / sigmaP_ / sigmaP_);
    wsum += particles_[i].w;
  }
  x_ = 0.0f;
  y_ = 0.0f;
  if(wsum > 0.0f){
    for(int i=0; i<numParticles_; i++){
      particles_[i].w /= wsum;
      x_ += particles_[i].x * particles_[i].w;
      y_ += particles_[i].y * particles_[i].w;
      particles2_[i].set(particles_[i]);
    }
    return true;
  } else {
    initParticle();
    return false;
  }
}


// systematic resampling
void NsParticleFilterClass::resample()
{
  weights_[0] = particles_[0].w;
  for(int i=1; i<numParticles_; i++){
    weights_[i] = weights_[i-1] + particles_[i].w;

    // shuffle
    int idx = NsRandom.next() % (numParticles_ - i + 1);
    p_.set(particles_[numParticles_ - i]);
    particles_[numParticles_ - i].set(particles_[idx]);
    particles_[idx].set(p_);
  }
  float w = NsRandom.nextFloat() / numParticles_;
  int j = 0;
  for(int i=0; i<numParticles_; i++){
    j--;
    while(weights_[++j] < w);
    particles_[i].x = particles2_[j].x;
    particles_[i].y = particles2_[j].y;
    particles_[i].v = particles2_[j].v;
    particles_[i].w = 1.0f / numParticles_;
    w += 1.0 / numParticles_;
  }
}

/*
// multinomial resampling
void NsParticleFilterClass::resample()
{
  float w;
  int j;
  weights_[0] = particles_[0].w;
  for(int i=1; i<numParticles_; i++){
    weights_[i] = weights_[i-1] + particles_[i].w;
  }
  for(int i=0; i<numParticles_; i++){
    w = NsRandom.nextFloat();
    j = -1;
    while(weights_[++j] < w);
    particles_[i].x = particles2_[j].x;
    particles_[i].y = particles2_[j].y;
    particles_[i].v = particles2_[j].v;
    particles_[i].w = 1.0f / numParticles_;
  }
}
*/

float NsParticleFilterClass::getX(){ return x_; }
float NsParticleFilterClass::getY(){ return y_; }
float NsParticleFilterClass::getSigma(){ return sigma_; }

NsParticleFilterClass NsParticleFilter;

