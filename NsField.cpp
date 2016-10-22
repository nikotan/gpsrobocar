#include "NsField.h"
#include "NsDegree.h"


NsFieldClass::NsFieldClass(void)
{
}

void NsFieldClass::init(
  const NsDegree &lat0, const NsDegree &lon0, const NsDegree &lat1, const NsDegree &lon1,
  float targetArea, float targetDistance, float controlLength
)
{
  lat0_.set(lat0);
  lon0_.set(lon0);
  lat1_.set(lat1);
  lon1_.set(lon1);
  targetA_ = targetArea;
  targetD_ = targetDistance;
  controlL_ = controlLength;

  // calc coordinate parameter (Hubeny : http://yamadarake.jp/trdi/report000001.html)
  latOr_.set(lat0_);
  latOr_.add(lat1_);
  latOr_.multiply(0.5f);
  lonOr_.set(lon0_);
  lonOr_.add(lon1_);
  lonOr_.multiply(0.5f);
  float sinlat = sin(latOr_.getDeg() * M_PI / 180.0f);
  float es = sqrt(1.0f - NSFIELD_E2 * sinlat * sinlat);
  fX_ = NSFIELD_A * abs(cos(latOr_.getDeg() * M_PI / 180.0f)) / es * M_PI / 180.0f;
  fY_ = NSFIELD_AE / es / es / es * M_PI / 180.0f;

  getXFromLon(&x0_, lon0_);
  getYFromLat(&y0_, lat0_);
  getXFromLon(&x1_, lon1_);
  getYFromLat(&y1_, lat1_);

  // calc target and control points
  float vx = x1_ - x0_;
  float vy = y1_ - y0_;
  float d2 = vx * vx + vy * vy;
  vx /= sqrt(d2);
  vy /= sqrt(d2);

  targetX_[0] = x1_ - vy * targetD_;
  targetY_[0] = y1_ + vx * targetD_;
  targetX_[1] = x1_ + vx * targetD_;
  targetY_[1] = y1_ + vy * targetD_;
  targetX_[2] = x1_ + vy * targetD_;
  targetY_[2] = y1_ - vx * targetD_;
  targetX_[3] = x0_ - vy * targetD_;
  targetY_[3] = y0_ + vx * targetD_;
  targetX_[4] = x0_ - vx * targetD_;
  targetY_[4] = y0_ - vy * targetD_;
  targetX_[5] = x0_ + vy * targetD_;
  targetY_[5] = y0_ - vx * targetD_;

  controlX_[0] = -vx;
  controlY_[0] = -vy;
  controlX_[1] = -vy;
  controlY_[1] =  vx;
  controlX_[2] =  vx;
  controlY_[2] =  vy;
  controlX_[3] =  vx;
  controlY_[3] =  vy;
  controlX_[4] = -vy;
  controlY_[4] =  vx;
  controlX_[5] = -vx;
  controlY_[5] = -vy;
}

// Bezier Curve
//   http://d.hatena.ne.jp/shspage/20140625/1403702735
//   http://geom.web.fc2.com/geometry/bezier/curvature.html
void NsFieldClass::getRadius(float* p, NsDegree* plat, NsDegree* plon, const NsDegree &lat, const NsDegree &lon, float dir, int targetId)
{
  getDistance(&d_, targetId, lat, lon);
  getXFromLon(&x_, lon);
  getYFromLat(&y_, lat);
  xc_ = x_ + sin(dir * M_PI / 180.0f) * d_ * controlL_;
  yc_ = y_ + cos(dir * M_PI / 180.0f) * d_ * controlL_;

  f_  = sqrt((xc_ - x_) * (xc_ - x_) + (yc_ - y_) * (yc_ - y_));
  fb_ = (
    (xc_ - x_) * (targetY_[targetId] + controlY_[targetId] * d_ * controlL_ - 2.0f * yc_ + y_) -
    (targetX_[targetId] + controlX_[targetId] * d_ * controlL_ - 2.0f * xc_ + x_) * (yc_ - y_)
    );
  if(abs(fb_) < 0.01){
    if(fb_ < 0.0f) {
      *p = -100.0f;
    }else if(fb_ > 0.0f) {
      *p = 100.0f;
    }
  } else {
    *p =  3.0f * f_ * f_ * f_ / 2.0f / fb_;
    if(*p > 100.0f) {
      *p = 100.0f;
    } else if(*p < -100.0f) {
      *p = -100.0f;
    }
  }

  x_ -= *p * cos(dir * M_PI / 180.0f);
  y_ += *p * sin(dir * M_PI / 180.0f);
  getLatFromY(plat, y_);
  getLonFromX(plon, x_);
}

void NsFieldClass::getDistance(float* p, int targetId, const NsDegree &lat, const NsDegree &lon)
{
  getXFromLon(&x_, lon);
  getYFromLat(&y_, lat);
  *p = sqrt((x_ - targetX_[targetId]) * (x_ - targetX_[targetId]) + (y_ - targetY_[targetId]) * (y_ - targetY_[targetId]));
}

void NsFieldClass::getDistance(float* p, const NsDegree &lat0, const NsDegree &lon0, const NsDegree &lat1, const NsDegree &lon1)
{
  getXFromLon(&x_, lon0);
  getYFromLat(&y_, lat0);
  getXFromLon(&xc_, lon1);
  getYFromLat(&yc_, lat1);
  *p = sqrt((x_ - xc_) * (x_ - xc_) + (y_ - yc_) * (y_ - yc_));
}

void NsFieldClass::getLatFromY(NsDegree* p, float y)
{
  f_ = y / fY_;
  i_ = (int)floor(f_);
  p->set(latOr_);
  p->add(i_, f_ - i_);
}
void NsFieldClass::getLonFromX(NsDegree* p, float x)
{
  f_ = x / fX_;
  i_ = (int)floor(f_);
  p->set(lonOr_);
  p->add(i_, f_ - i_);
}

void NsFieldClass::getYFromLat(float* p, const NsDegree &lat)
{
  dtmp_.set(lat);
  dtmp_.sub(latOr_);
  *p = dtmp_.getDeg() * fY_;
}
void NsFieldClass::getXFromLon(float* p, const NsDegree &lon)
{
  dtmp_.set(lon);
  dtmp_.sub(lonOr_);
  *p = dtmp_.getDeg() * fX_;
}

void NsFieldClass::getDLat(NsDegree* p, const NsDegree &lat)
{
  p->set(lat);
  p->sub(latOr_);
}
void NsFieldClass::getDLon(NsDegree* p, const NsDegree &lon)
{
  p->set(lon);
  p->sub(lonOr_);
}

NsFieldClass NsField;

