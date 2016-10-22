#ifndef NSFIELD_H
#define NSFIELD_H

#include "NsDegree.h"

#define NSFIELD_A  6378137.0
#define NSFIELD_E2 0.006694380
#define NSFIELD_AE 6335439.3


class NsFieldClass
{
  private:
    NsDegree lat0_;
    NsDegree lon0_;
    NsDegree lat1_;
    NsDegree lon1_;
    float targetA_;
    float targetD_;
    float controlL_;

    NsDegree latOr_;
    NsDegree lonOr_;
    NsDegree dtmp_;
    float fX_;
    float fY_;

    float x0_;
    float y0_;
    float x1_;
    float y1_;

    float targetX_[6];
    float targetY_[6];
    float controlX_[6];
    float controlY_[6];

    float f_, x_, y_, xc_, yc_, fb_, d_;
    int   i_;

  public:
    NsFieldClass(void);

    void init(const NsDegree&, const NsDegree&, const NsDegree&, const NsDegree&, float, float, float);

    void getLat0(NsDegree* d){ d->set(lat0_); }
    void getLon0(NsDegree* d){ d->set(lon0_); }
    void getLat1(NsDegree* d){ d->set(lat1_); }
    void getLon1(NsDegree* d){ d->set(lon1_); }
    void getOriginLat(NsDegree* d){ d->set(latOr_); }
    void getOriginLon(NsDegree* d){ d->set(lonOr_); }
    float getFX(){ return fX_; }
    float getFY(){ return fY_; }

    void getRadius(float*, NsDegree*, NsDegree*, const NsDegree&, const NsDegree&, float, int);
    void getDistance(float*, int, const NsDegree&, const NsDegree&);
    void getDistance(float*, const NsDegree&, const NsDegree&, const NsDegree&, const NsDegree&);
    bool isNear(float d){ return (d < targetA_); }

    void getLatFromY(NsDegree*, float);
    void getLonFromX(NsDegree*, float);
    void getYFromLat(float*, const NsDegree&);
    void getXFromLon(float*, const NsDegree&);

    void getDLat(NsDegree*, const NsDegree&);
    void getDLon(NsDegree*, const NsDegree&);

    void getTargetLat(int i, NsDegree* d)
    {
      getLatFromY(&dtmp_, targetY_[i]);
      d->set(dtmp_);
    }
    void getTargetLon(int i, NsDegree* d)
    {
      getLonFromX(&dtmp_, targetX_[i]);
      d->set(dtmp_);
    }
    void getControlLat(int i, NsDegree* d)
    {
      getLatFromY(&dtmp_, targetY_[i] + controlY_[i]);
      d->set(dtmp_);
    }
    void getControlLon(int i, NsDegree* d)
    {
      getLonFromX(&dtmp_, targetX_[i] + controlX_[i]);
      d->set(dtmp_);
    }
};

extern NsFieldClass NsField;

#endif

