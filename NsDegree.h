
#ifndef NSDEGREE_H
#define NSDEGREE_H

#include "Arduino.h"


class NsDegree
{
private:
  int   degI_;
  float degF_;

public:

  NsDegree()
  {
    degI_ = 0;
    degF_ = 0.0f;
  }

  void set(int degI, float degF)
  {
    degI_ = degI;
    degF_ = degF;
  }

  void set(const NsDegree &obj)
  {
    degI_ = obj.getDegI();
    degF_ = obj.getDegF();
  }

  // str::ddd.ddddd
  bool parseD(String str)
  {
    int idx = str.indexOf('.');
    if(idx <= 0){
      return false;
    } else {
      degI_ = str.substring(0, idx).toInt();
      str = "0." + str.substring(idx + 1);
      degF_ = atof(str.c_str());
      return true;
    }
  }
  
  // str::dddmm.mmm
  bool parseDM(String str)
  {
    int idx = str.indexOf('.');
    if(idx <= 3){
      return false;
    } else {
      degI_ = str.substring(0, idx - 2).toInt();
      degF_ = atof(str.substring(idx - 2).c_str()) / 60.0f;
      return true;
    }
  }

  int getDegI()
  {
    return degI_;
  }
  
  float getDegF()
  {
    return degF_;
  }

  float getDeg()
  {
    return (float)degI_ + degF_;
  }

  void add(const NsDegree &obj)
  {
    degI_ += obj.getDegI();
    degF_ += obj.getDegF();
    if(degF_ >= 1.0f){
      degF_ -= 1.0f;
      degI_ += 1;
    } else if(degF_ < 0.0f){
      degF_ += 1.0f;
      degI_ -= 1;
    }
  }
  
  void sub(const NsDegree &obj)
  {
    degI_ -= obj.getDegI();
    degF_ -= obj.getDegF();
    if(degF_ >= 1.0f){
      degF_ -= 1.0f;
      degI_ += 1;
    } else if(degF_ < 0.0f){
      degF_ += 1.0f;
      degI_ -= 1;
    }
  }

  void multiply(float a)
  {
    float d = degI_ * a;
    degI_ = (int)floor(d);
    degF_ = degF_ * abs(a) + (d - (float)degI_);
    if(degF_ >= 1.0f){
      degF_ -= 1.0f;
      degI_ += 1;
    } else if(degF_ < 0.0f){
      degF_ += 1.0f;
      degI_ -= 1;
    }
  }

  void formatD(char* buf)
  {
    sprintf(buf, "%d.%0ld", degI_, (long)round(degF_ * 1000000l));
  }

  void formatDF(char* buf)
  {
    sprintf(buf, "%3d.%0ld", degI_, (long)round(degF_ * 1000000l));
  }

};


#endif NSDEGREE_H
