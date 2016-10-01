//#include <cstdlib>
//#include <cmath>
#include "NsKarmanFilter.h"

NsKarmanFilterClass::NsKarmanFilterClass(void)
{
}

void NsKarmanFilterClass::init(float sigmaV, float sigmaD, float sigmaP)
{
	sigmaV_ = sigmaV;
	sigmaD_ = sigmaD * M_PI / 180.0f;
	sigmaP_ = sigmaP;
	
	x_ = 0.0f;
	y_ = 0.0f;
	timePrev_ = 0;
	for(int i=0; i<4; i++){
		p_[i]   = 0.0f;
		s_[i]   = 0.0f;
		si_[i]  = 0.0f;
		k_[i]   = 0.0f;
	}
	sd_ = 0.0f;
	cd_ = 0.0f;
}

void NsKarmanFilterClass::predict(unsigned long timeNow, float v, float dir)
{
	if(timePrev_ > 0){
		sd_ = sin(dir * M_PI / 180.0f);
		cd_ = cos(dir * M_PI / 180.0f);
		t2_ = (timeNow - timePrev_) * (timeNow - timePrev_) / 1000.0f / 1000.0f;
		
		x_ += v * (timeNow - timePrev_) / 1000.0f * sd_;
		y_ += v * (timeNow - timePrev_) / 1000.0f * cd_;
		
		p_[0] += (sigmaV_ * sigmaV_ * sd_ * sd_ + sigmaD_ * sigmaD_ * v * v * cd_ * cd_) * t2_;
		p_[1] += (sigmaV_ * sigmaV_ - sigmaD_ * sigmaD_ * v * v) * sd_ * cd_ * t2_;
		p_[2] += (sigmaV_ * sigmaV_ - sigmaD_ * sigmaD_ * v * v) * sd_ * cd_ * t2_;
		p_[3] += (sigmaV_ * sigmaV_ * cd_ * cd_ + sigmaD_ * sigmaD_ * v * v * sd_ * sd_) * t2_;
	}
	timePrev_ = timeNow;
}

bool NsKarmanFilterClass::estimate(float x, float y)
{
	s_[0] = sigmaP_ * sigmaP_ + p_[0];
	s_[1] = p_[1];
	s_[2] = p_[2];
	s_[3] = sigmaP_ * sigmaP_ + p_[3];
	
	float det = s_[0] * s_[3] - s_[1] * s_[2];
	if(det > 0.0f){
		si_[0] = s_[3] / det;
		si_[1] = -1.0f * s_[1] / det;
		si_[2] = -1.0f * s_[2] / det;
		si_[3] = s_[0] / det;
		
		k_[0] = p_[0] * si_[0] + p_[1] * si_[2];
		k_[1] = p_[0] * si_[1] + p_[1] * si_[3];
		k_[2] = p_[2] * si_[0] + p_[3] * si_[2];
		k_[3] = p_[2] * si_[1] + p_[3] * si_[3];
		
		x_ += k_[0] * (x - x_) + k_[1] * (y - y_);
		y_ += k_[2] * (x - x_) + k_[3] * (y - y_);
		
		p_[0] = (1.0f - k_[0]) * p_[0] - k_[1] * p_[2];
		p_[1] = (1.0f - k_[0]) * p_[1] - k_[1] * p_[3];
		p_[2] = -1.0f * k_[2] * p_[0] + (1.0f - k_[3]) * p_[2];
		p_[3] = -1.0f * k_[2] * p_[1] + (1.0f - k_[3]) * p_[3];
		
		return true;
	} else {
		return false;
	}
}

float NsKarmanFilterClass::getX(){ return x_; }
float NsKarmanFilterClass::getY(){ return y_; }
float NsKarmanFilterClass::getP(int idx){ return p_[idx]; }
float NsKarmanFilterClass::getSigmaX(){ return sqrt(p_[0]); }
float NsKarmanFilterClass::getSigmaY(){ return sqrt(p_[3]); }


NsKarmanFilterClass NsKarmanFilter;

