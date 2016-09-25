// parse setup.txt and setup parameters
void parseSetup()
{
  File file = SD.open(FILENAME_SETUP, FILE_READ);
  if(!file){
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("[ERR]SD.open()");
    sprintf(buff_, "(%s)", FILENAME_SETUP);
    NsLCD.moveCursor(0,1);
    NsLCD.print(buff_);
    waitSW1();
  } else {
    buffString_ = "";
    while( file.available() ){
      char c = (char)file.read();
      if(c == '\n'){
        buffString_.trim();
        if(buffString_.length() > 0 && !buffString_.startsWith("//")){
          int cnt = mySplit(buffString_, '=', buffStringArr_, 2);
          if(cnt == 2){
            buffStringArr_[0].trim();
            buffStringArr_[1].trim();
            // setup parameters
            if(buffStringArr_[0].equals("USE_USB")){
              if(buffStringArr_[1].equals("1")){
                useUSB_ = true;
              } else {
                useUSB_ = false;
              }
            } else if(buffStringArr_[0].equals("USE_XBEE")){
              if(buffStringArr_[1].equals("1")){
                useXBEE_ = true;
              } else {
                useXBEE_ = false;
              }
            } else if(buffStringArr_[0].equals("REF_LAT0")){
              refLat0_.parseD(buffStringArr_[1]);
            } else if(buffStringArr_[0].equals("REF_LON0")){
              refLon0_.parseD(buffStringArr_[1]);
            } else if(buffStringArr_[0].equals("REF_LAT1")){
              refLat1_.parseD(buffStringArr_[1]);
            } else if(buffStringArr_[0].equals("REF_LON1")){
              refLon1_.parseD(buffStringArr_[1]);
            } else if(buffStringArr_[0].equals("CPS_DECLINATION")){
              cps_dec_ = buffStringArr_[1].toFloat();
            }
          }
        }
        buffString_ = "";
      } else {
        buffString_ += c;
      }
    }
    file.close();

    // display message
    NsLCD.clear();
    sprintf(buff_, "[%s]", FILENAME_SETUP);
    NsLCD.moveCursor(0,0);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      NsLCD.moveCursor(0,1);
      NsLCD.print("push SW1");
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif

    // display parameters (serial port)
    NsLCD.displayOutputIcon(useUSB_); // Output Icon indicates useUSB
    NsLCD.displaySoundIcon(useXBEE_); // Sound Icon indicates useXBEE
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("USB  = ");
    NsLCD.moveCursor(7,0);
    if(useUSB_){
      NsLCD.print("on");
    } else {
      NsLCD.print("off");
    }
    NsLCD.moveCursor(0,1);
    NsLCD.print("XBEE = ");
    NsLCD.moveCursor(7,1);
    if(useXBEE_){
      NsLCD.print("on");
    } else {
      NsLCD.print("off");
    }
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif

    // display parameters (reference point0)
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("lat0: ");
    refLat0_.formatDF(buff_);
    NsLCD.moveCursor(6,0);
    NsLCD.print(buff_);
    NsLCD.moveCursor(0,1);
    NsLCD.print("lon0: ");
    refLon0_.formatDF(buff_);
    NsLCD.moveCursor(6,1);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif

    // display parameters (reference point1)
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("lat1: ");
    refLat1_.formatDF(buff_);
    NsLCD.moveCursor(6,0);
    NsLCD.print(buff_);
    NsLCD.moveCursor(0,1);
    NsLCD.print("lon1: ");
    refLon1_.formatDF(buff_);
    NsLCD.moveCursor(6,1);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif

    // display parameters (other)
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("declination:");
    dtostrf(cps_dec_, 4, 1, buff_);
    NsLCD.moveCursor(12,0);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif


    // calc coordinate parameter (Hubeny : http://yamadarake.jp/trdi/report000001.html)
    latOr_.set(refLat0_);
    latOr_.add(refLat1_);
    latOr_.multiply(0.5f);
    lonOr_.set(refLon0_);
    lonOr_.add(refLon1_);
    lonOr_.multiply(0.5f);
    float a  = 6378137.0f;
    float e2 = 0.006694380f;
    float ae = 6335439.0f;
    float sinlat = sin(latOr_.getDeg() * M_PI / 180.0f);
    float es = sqrt(1.0f - e2 * sinlat * sinlat);
    fCoordX_ = a * cos(latOr_.getDeg() * M_PI / 180.0f) / es * M_PI / 180.0f;
    fCoordY_ = ae / es / es / es * M_PI / 180.0f;
  
    // display coordinate parameters
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("latOr:");
    latOr_.formatDF(buff_);
    NsLCD.moveCursor(6,0);
    NsLCD.print(buff_);
    NsLCD.moveCursor(0,1);
    NsLCD.print("lonOr:");
    lonOr_.formatDF(buff_);
    NsLCD.moveCursor(6,1);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("fCoordX:");
    dtostrf(fCoordX_, 8, 1, buff_);
    NsLCD.moveCursor(8,0);
    NsLCD.print(buff_);
    NsLCD.moveCursor(0,1);
    NsLCD.print("fCoordY:");
    dtostrf(fCoordY_, 8, 1, buff_);
    NsLCD.moveCursor(8,1);
    NsLCD.print(buff_);
    #ifdef INTERACTIVE
      waitSW1();
    #else
      delay(DELAY_LCD_INFO);
    #endif
  }
}

