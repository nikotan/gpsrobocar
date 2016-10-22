#ifndef GYSFDMAXB_H
#define GYSFDMAXB_H


class GYSFDMAXB
{
  private:
    char   buf_[128];
    char   xor_;
    String str_;

  public:

    GYSFDMAXB()
    {
    }

    void getCMD(char *buf, char *cmd)
    {
      sprintf(buf_, "PMTK%s", cmd);
      str_ = String(buf_);
      xor_ = 0x00;
      for(int i=0; i<str_.length(); i++){
        xor_ = xor_ ^ buf_[i];
      }
      if(xor_ > 0x0f){
        sprintf(buf, "$%s*%X\r\n", buf_, xor_);
      } else {
        sprintf(buf, "$%s*0%X\r\n",buf_, xor_);
      }
    }
};


#endif

