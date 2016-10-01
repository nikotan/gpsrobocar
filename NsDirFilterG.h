
#ifndef NSDIRFG_H
#define NSDIRFG_H

#include "Arduino.h"


class NsDirFilterGClass
{
private:
  int   numGrid_;
  float errorDir_;
  float sigmaDir_;
  float sigmaDirAV_;
  float cpsDec_;

  float *pr_;
  float *pr_tmp_;
  float *tr_;
  unsigned long timePrev_;
  float dirAVPrev_;
  float dir_;
  
public:
  NsDirFilterGClass(void);

  void init(int, float, float, float, float);
  void predict(unsigned long, float);
  float estimate(int);
  float getDir();
};

extern NsDirFilterGClass NsDirFilterG;

#endif

