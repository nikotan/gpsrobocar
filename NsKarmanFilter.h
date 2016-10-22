#ifndef NSKARMANFILTER_H
#define NSKARMANFILTER_H


class NsKarmanFilterClass
{
  private:
    float sigmaV_;
    float sigmaD_;
    float sigmaA_;
    float sigmaP_;

    float x_, y_, xtmp_, ytmp_;
    float p_[4];
    unsigned long timePrev_;

    float s_[4];
    float si_[4];
    float k_[4];
    float sd_, cd_, t2_;

  public:
    NsKarmanFilterClass(void);

    void init(float, float, float);
    void init(float, float, float, float);
    void predict(unsigned long, float, float);
    bool estimate(float, float);
    bool estimate(float, float, float);
    float getX();
    float getY();
    float getP(int);
    float getSigmaX();
    float getSigmaY();
    float getSigmaP();
};

extern NsKarmanFilterClass NsKarmanFilter;

#endif
