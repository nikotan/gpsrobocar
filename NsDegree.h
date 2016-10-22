#ifndef NSDEGREE_H
#define NSDEGREE_H

#ifdef ARDUINO
  #include "Arduino.h"
#endif


class NsDegree
{
  private:
    int   degI_;
    float degF_;
    int   i_;
    float f_;

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
    bool parseD(const String &str)
    {
      i_ = str.indexOf('.');
      if (i_ <= 0) {
        return false;
      } else {
        degI_ = str.substring(0, i_).toInt();
        degF_ = atof(("0." + str.substring(i_ + 1)).c_str());
        return true;
      }
    }

    // str::dddmm.mmm
    bool parseDM(const String &str)
    {
      i_ = str.indexOf('.');
      if (i_ <= 3) {
        return false;
      } else {
        degI_ = str.substring(0, i_ - 2).toInt();
        degF_ = atof(str.substring(i_ - 2).c_str()) / 60.0f;
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

    void getDeg6(long *p)
    {
      *p = degI_ * 1000000l + (long)round(degF_ * 1000000l);
    }

    void add(int degI, float degF)
    {
      degI_ += degI;
      degF_ += degF;
      i_ = (int)floor(degF_);
      degI_ += i_;
      degF_ -= i_;
    }

    void add(const NsDegree &obj)
    {
      add(obj.getDegI(), obj.getDegF());
    }

    void sub(int degI, float degF)
    {
      degI_ -= degI;
      degF_ -= degF;
      i_ = (int)floor(degF_);
      degI_ += i_;
      degF_ -= i_;
    }

    void sub(const NsDegree &obj)
    {
      sub(obj.getDegI(), obj.getDegF());
    }

    void multiply(float a)
    {
      f_ = degI_ * a;
      degI_ = (int)floor(f_);
      degF_ = degF_ * a + f_ - (float)degI_;
      i_ = (int)floor(degF_);
      degI_ += i_;
      degF_ -= i_;
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


#endif

