// GPS robot car control sketch
// NIKO[2016/09/11]
// NIKO[2009/10/01]
// NIKO[2009/09/29]

#include <math.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "NsLCD.h"
#include "NsGH81.h"
#include "NsSerialBuffer.h"
#include "NsGyro.h"
#include "NsCompass.h"
#include "NsDirFilterG.h"
#include "NsStopWatch.h"
#include "NsDegree.h"

// mode
//#define DEBUG
//#define INFO
//#define TIMER
#define LOGGER
//#define INTERACTIVE

// serial baud rate
#define BAUD_RATE_USB  9600  // Serial
#define BAUD_RATE_NMEA 9600  // Serial1 (SiRF star III)
#define BAUD_RATE_XBEE 9600  // Serial2 (XBee)
#define BAUD_RATE_GH81 9600  // Serial3 (gps robot car shield)

// GPS module's LED and SW pins
#define GPS_LED1 9
#define GPS_LED2 8
#define GPS_SW1  5
#define GPS_SW1_MSEC 500

// PWM control pins
#define PWM_SERVO 6
#define PWM_SPEED 7

// PWM parameters
#define PWM_SERVO_ZER  90
#define PWM_SERVO_MIN  35
#define PWM_SERVO_MAX 145
#define PWM_SPEED_ZER  60
#define PWM_SPEED_MIN  30
#define PWM_SPEED_MAX  90

// sensor pins
#define SEN_ACC_X   13
#define SEN_ACC_Y   14
#define SEN_ACC_Z   15
#define SEN_GYR_PIT 11
#define SEN_GYR_YAW 12
#define SEN_CPS_0   43
#define SEN_CPS_1   45
#define SEN_CPS_2   47

// sensor interval
#define SEN_INTERVAL_MSEC 100
#define SEN_CPS_INTMSEC 100
#define SEN_GYR_INTMSEC 20
#define SEN_GYR_FILTER 0.8
#define SEN_GYR_DRIFTMSEC 10000

// grid filter to estimate direction
#define DIRF_NUMG        128
#define DIRF_ERROR_DIR   0.0
#define DIRF_SIGMA_DIR   5.0
#define DIRF_SIGMA_DIRAV 3.0

// SD(SPI bus) pins
#define PIN_SD_CS 10
// SD file name
#define FILENAME_SETUP "setup.txt"
#define FILENAME_LOG   "log.csv"

// length of buffers
#define LEN_BUFF      128
#define LEN_BUFF_SER  128
#define LEN_BUFF_SER1 128
#define LEN_BUFF_SER2 128
#define LEN_BUFF_SARR 20

// others
#define DELAY_LCD_INFO 1000


//////////////////////////////////////

// setup parameter: serial port
boolean useUSB_;
boolean useXBEE_;

// setup parameter: reference points
NsDegree refLat0_;
NsDegree refLon0_;
NsDegree refLat1_;
NsDegree refLon1_;
NsDegree latOr_;
NsDegree lonOr_;
float fCoordX_;
float fCoordY_;

// setup parameter: magnetic declination
float cps_dec_ = 0.0f;

char buff_[LEN_BUFF];
String buffString_;
String buffStringArr_[LEN_BUFF_SARR];
NsSerialBuffer serBuffer_;
NsSerialBuffer serBuffer1_;
NsSerialBuffer serBuffer2_;

Servo pwmServo_;
Servo pwmSpeed_;
byte  bPwmServo_;
byte  bPwmSpeed_;

unsigned long timeNow_;
unsigned long timePre1s_;
unsigned long timePreDrift_;

// sensing data
NsDegree nmeaLat_;
NsDegree nmeaLon_;
float    nmeaAlt_;
int      nmeaNsat_   = 0;
int      nmeaStatus_ = 0;
NsDegree gh81Lat_;
NsDegree gh81Lon_;
float    gh81Alt_;
int      gh81Nsat_   = 0;
int      gh81Status_ = 0;
NsDegree gpsLat_;
NsDegree gpsLon_;
float    gpsAlt_;
int      gpsNsat_   = 0;
int      gpsStatus_ = 0;

int state_ = 0;
int interval_ = 0;
int num_      = 0;

// for logging
File fileLog_;


void setup()
{
  // initialize SW1
  pinMode(GPS_SW1, INPUT);

  // initialize I2C LCD
  Wire.begin( 0 ); // I2C master mode
  NsLCD.init();
  NsLCD.clear();
  NsLCD.displayAntennaIcon(false);
  NsLCD.displayTelIcon(false);
  NsLCD.displaySoundIcon(false);
  NsLCD.displayOutputIcon(false);
  NsLCD.displayUpDownIcon(false, false);
  NsLCD.displayKeyIcon(false);
  NsLCD.displayMuteIcon(false);
  NsLCD.displayBatteryIcon(true, 0);
  NsLCD.displayNumIcon(false);

  // initialize PWM
  pwmServo_.attach(PWM_SERVO,1000,2000);
  pwmSpeed_.attach(PWM_SPEED,1000,2000);
  pwmServo_.write(PWM_SERVO_ZER);
  pwmSpeed_.write(PWM_SPEED_ZER);
  bPwmServo_ = PWM_SERVO_ZER;
  bPwmSpeed_ = PWM_SPEED_ZER;

  // initialize serial ports
  Serial.begin(BAUD_RATE_USB);
  Serial1.begin(BAUD_RATE_NMEA);
  Serial2.begin(BAUD_RATE_XBEE);
  Serial3.begin(BAUD_RATE_GH81);
  serBuffer_.init(recieveSerialCMD, LEN_BUFF_SER, 1);
  serBuffer1_.init(recieveSerialNMEA, LEN_BUFF_SER1, 2);
  serBuffer2_.init(recieveSerialCMD, LEN_BUFF_SER2, 1);

  // initialize SD and read setup.txt
  if(!SD.begin(PIN_SD_CS)){
    NsLCD.clear();
    NsLCD.moveCursor(0,0);
    NsLCD.print("[ERR]SD.begin()");
    waitSW1();
  } else {
    parseSetup();
  }

  // initialize sensor pins
  NsGyro.init(SEN_GYR_YAW, SEN_GYR_INTMSEC, SEN_GYR_FILTER);
  NsCompass.init(SEN_CPS_0, SEN_CPS_1, SEN_CPS_2, SEN_CPS_INTMSEC);
  NsDirFilterG.init(DIRF_NUMG, DIRF_ERROR_DIR, DIRF_SIGMA_DIR, DIRF_SIGMA_DIRAV, cps_dec_);

  // initialize GH81
  NsGH81.init(recieveLocation,recieveTime,GPS_LED1,GPS_LED2);
  NsGH81.setStartCmd(buff_);
  Serial3.print(buff_);

  // initialize gyro origin
  initGyro();

  // initialize LCD
  NsLCD.clear();
  NsLCD.moveCursor(0,0);
  NsLCD.print("[00/00 00:00:00]");
  sprintf(buff_, "000.0,sv%03dsp%03d", bPwmServo_, bPwmSpeed_);
  NsLCD.moveCursor(0,1);
  NsLCD.print(buff_);

  // initialize timer
  timeNow_      = millis();
  timePre1s_    = timeNow_;
  timePreDrift_ = timeNow_;

  state_ = 0;
}

void loop()
{
  // check SW1
  if(digitalRead(GPS_SW1) == LOW){
    if(state_ == 0){
      startAutoDrive();
    } else if(state_ == 1){
      stopAutoDrive();
    }
    delay(GPS_SW1_MSEC);
  }
  
  // check timer
  timeNow_ = millis();

  // every 1s
  if(timeNow_ - timePre1s_ > 1000){
    #ifdef DEBUG
      check_mem(); // to check memory leak
    #endif
    timePre1s_ = timeNow_;
  }

  // gyro drift update
  if(timeNow_ - timePreDrift_ > SEN_GYR_DRIFTMSEC){
    NsGyro.setDir(NsDirFilterG.getDir());
    timePreDrift_ = timeNow_;
  }

  // ジャイロ計測
  interval_ = NsGyro.update();
  #ifdef INFO
    if(interval_ > 0){
      sprintf(buff_, "[INFO] gyro, %d[msec],", interval_);
      myPrint(buff_);
      float dirAV = NsGyro.getDirAV();
      float dir   = NsGyro.getDir();
      dtostrf(dirAV, 6, 1, buff_);
      myPrint(buff_);
      myPrint(",");
      dtostrf(dir, 6, 1, buff_);
      myPrintln(buff_);
    }
  #endif

  // コンパス計測
  interval_ = NsCompass.update();
  #ifdef INFO
    if(interval_ > 0){
      sprintf(buff_, "[INFO] compass, %d[msec], %d, ", interval_, NsCompass.getDirId());
      myPrint(buff_);
      float dir   = NsCompass.getDir();
      dtostrf(dir, 6, 1, buff_);
      myPrintln(buff_);
    }
  #endif

  if(interval_ > 0){
    // 方位推定：予測
    #ifdef TIMER
      NsStopWatch.start();
    #endif
    NsDirFilterG.predict(NsCompass.getTime(), NsGyro.getDirAVFiltered());
    #ifdef TIMER
      NsStopWatch.stop();
      sprintf(buff_, "predict: %ld[msec], %ld[microsec]", NsStopWatch.timeMillis(), NsStopWatch.timeMicros());
      myPrintln(buff_);
    #endif

    // 方位推定：推定
    #ifdef TIMER
      NsStopWatch.start();
    #endif
    float dir = NsDirFilterG.estimate(NsCompass.getDirId());
    #ifdef TIMER
      NsStopWatch.stop();
      sprintf(buff_, "estimate: %ld[msec], %ld[microsec]", NsStopWatch.timeMillis(), NsStopWatch.timeMicros());
      myPrintln(buff_);
    #endif
    if(!(dir < 0.0)){
      NsLCD.moveCursor(0,1);
      dtostrf(dir, 5, 1, buff_);
      NsLCD.print(buff_);
    }

    // send pos
    myPrint("pos,");
    gpsLat_.formatD(buff_);
    myPrint(buff_);
    myPrint(",");
    gpsLon_.formatD(buff_);
    myPrint(buff_);
    sprintf(buff_, ",%d,%d,%d,", gpsNsat_, gpsStatus_, NsCompass.getDirId());
    myPrint(buff_);
    dtostrf(NsGyro.getDir(), -1, 1, buff_);
    myPrint(buff_);
    myPrint(",");
    dtostrf(NsDirFilterG.getDir(), -1, 1, buff_);
    myPrintln(buff_);

    // logging
    #ifdef LOGGER
      if(state_ == 1 && fileLog_){
        #ifdef TIMER
          NsStopWatch.start();
        #endif
        sprintf(buff_, "%ld,%d,", NsCompass.getTime(), NsCompass.getDirId());
        fileLog_.print(buff_);
        dtostrf(NsCompass.getDir(), -1, 1, buff_);
        fileLog_.print(buff_);
        fileLog_.print(",");
        dtostrf(NsGyro.getDir(), -1, 1, buff_);
        fileLog_.print(buff_);
        fileLog_.print(",");
        dtostrf(NsDirFilterG.getDir(), -1, 1, buff_);
        fileLog_.println(buff_);
        #ifdef TIMER
          NsStopWatch.stop();
          sprintf(buff_, "logging: %ld[msec], %ld[microsec]", NsStopWatch.timeMillis(), NsStopWatch.timeMicros());
          myPrintln(buff_);
        #endif
      }
    #endif LOGGER
  }

  // rcv Serial data (USB)
  if((num_ = Serial.available()) > 0){
    for(int i=0; i<num_; i++){
      serBuffer_.update(Serial.read());
    }
  }

  // rcv Serial1 data (NMEA)
  if((num_ = Serial1.available()) > 0){
    for(int i=0; i<num_; i++){
      serBuffer1_.update(Serial1.read());
    }
  }

  // rcv Serial2 data (XBEE)
  if((num_ = Serial2.available()) > 0){
    for(int i=0; i<num_; i++){
      serBuffer2_.update(Serial2.read());
    }
  }

  // rcv Serial3 data (GH81)
  if((num_ = Serial3.available()) > 0){
    for(int i=0; i<num_; i++){
      NsGH81.update(Serial3.read());
    }
  }
}

// rcv GH81 location result
void recieveLocation(NSGH81_LOCATION g)
{
  gh81Lat_.set(g.lat_deg, g.lat_min * 0.0001f / 60.0f);
  gh81Lon_.set(g.lon_deg, g.lon_min * 0.0001f / 60.0f);
  gh81Alt_ = g.alt * 0.1f;
  gh81Nsat_ = g.nsat;
  gh81Status_ = g.status; // 0:can't locate, 1:2D location, 2:3D location

  // write to serial
  myPrint("gh81,");
  gh81Lat_.formatD(buff_);
  myPrint(buff_);
  myPrint(",");
  gh81Lon_.formatD(buff_);
  myPrint(buff_);
  myPrint(",");
  dtostrf(gh81Alt_, -1, 1, buff_);
  myPrint(buff_);
  sprintf(buff_, ",%d,%d", gh81Nsat_, gh81Status_);
  myPrintln(buff_);

  // update location data
  updateLocation();
}

// rcv GH81 time result
void recieveTime(NSGH81_TIME g)
{
  sprintf(buff_, "[%02d/%02d %02d:%02d:%02d]", g.month, g.day, g.hour+9, g.min, g.sec);
  NsLCD.moveCursor(0,0);
  NsLCD.print(buff_);
}

// rcv Serial1 event (NMEA)
void recieveSerialNMEA(char* buf, int len)
{
  buffString_  = String(buf);
  if(buffString_ .startsWith("$GPGGA")){
    int cnt = mySplit(buffString_ , ',', buffStringArr_, LEN_BUFF_SARR);
    if(cnt > 9){
      // parse
      nmeaLat_.parseDM(buffStringArr_[2]);
      if(buffStringArr_[3].equals("S")){
        nmeaLat_.multiply(-1.0);
      }
      nmeaLon_.parseDM(buffStringArr_[4]);
      if(buffStringArr_[5].equals("W")){
        nmeaLon_.multiply(-1.0);
      }
      nmeaAlt_ = buffStringArr_[9].toFloat();
      nmeaNsat_ = buffStringArr_[7].toInt();
      nmeaStatus_ = buffStringArr_[6].toInt();
      //float hdop = buffStringArr_[8].toFloat();

      // write to serial
      myPrint("nmea,");
      nmeaLat_.formatD(buff_);
      myPrint(buff_);
      myPrint(",");
      nmeaLon_.formatD(buff_);
      myPrint(buff_);
      myPrint(",");
      dtostrf(nmeaAlt_, -1, 1, buff_);
      myPrint(buff_);
      sprintf(buff_, ",%d,%d", nmeaNsat_, nmeaStatus_);
      myPrintln(buff_);

      // update location data
      updateLocation();
    }
  }
}

void updateLocation()
{
  gpsLat_.set(nmeaLat_);
  gpsLon_.set(nmeaLon_);
  gpsAlt_ = nmeaAlt_;
  gpsNsat_ = nmeaNsat_;
  gpsStatus_ = nmeaStatus_;

  if(gpsStatus_ > 0){
    // update LCD icon
    int nbat = 0;
    if(gpsNsat_ == 0){
      nbat = 0;
    }else if(gpsNsat_ < 4){
      nbat = 1;
    }else if(gpsNsat_ < 7){
      nbat = 2;
    }else{
      nbat = 3;
    }
    NsLCD.displayBatteryIcon(true, nbat);
    NsLCD.displayKeyIcon(true);
    NsLCD.displayUpDownIcon(true, false);
  } else {
    NsLCD.displayBatteryIcon(true, 0);
    NsLCD.displayKeyIcon(false);
    NsLCD.displayUpDownIcon(false, false);
  }
}

// rcv Serial event (USB or XBEE)
void recieveSerialCMD(char* buf, int len)
{
  buffString_ = String(buf);

  // command = "sv???" (servo)
  if(buffString_.startsWith("sv")){
    byte v = (byte)(buffString_.substring(2).toInt());
    if(v > PWM_SERVO_MAX){
      v = PWM_SERVO_MAX;
    } else if(v < PWM_SERVO_MIN){
      v = PWM_SERVO_MIN;
    }
    setPwmServo(v);
  }
  
  // command = "sp???" (speed)
  else if(buffString_.startsWith("sp")){
    byte v = (byte)(buffString_.substring(2).toInt());
    if(v > PWM_SPEED_MAX){
      v = PWM_SPEED_MAX;
    } else if(v < PWM_SPEED_MIN){
      v = PWM_SPEED_MIN;
    }
    setPwmSpeed(v);
  }

  // command = "read [filename]"
  else if(buffString_.startsWith("read ")){
    if(state_ != 0){
      myPrintln("[RSP][ERR]can't open file while state!=0");
    } else {
      String filename = buffString_.substring(5);
      filename.trim();
      if(!SD.exists(filename)){
        myPrintln("[RSP][ERR]can't find file");
      } else {
        File file = SD.open(filename, FILE_READ);
        if(!file){
          myPrintln("[RSP][ERR]can't open file");
        } else {
          myPrintln("[RSP]read file...");
          sprintf(buff_, "--- start [%s] ---", filename.c_str());
          myPrintln(buff_);
          while( file.available() ){
            myWrite(file.read());
          }
          file.close();
          myPrintln("--- end ---");
        }
      }
    }
  }

  // command = "ref"
  else if(buffString_.equals("ref")){
    if(state_ != 0){
      myPrintln("[RSP][ERR]can't get ref while state!=0");
    } else {
      myPrint("ref,");
      refLat0_.formatD(buff_);
      myPrint(buff_);
      myPrint(",");
      refLon0_.formatD(buff_);
      myPrint(buff_);
      myPrint(",");
      refLat1_.formatD(buff_);
      myPrint(buff_);
      myPrint(",");
      refLon1_.formatD(buff_);
      myPrintln(buff_);
    }
  }

  // command = "init gyro"
  else if(buffString_.equals("init gyro")){
    if(state_ != 0){
      myPrintln("[RSP][ERR]can't init gyro while state!=0");
    } else {
      initGyro();
      NsLCD.clear();
      sprintf(buff_, ",sv%03dsp%03d", bPwmServo_, bPwmSpeed_);
      NsLCD.moveCursor(5,1);
      NsLCD.print(buff_);
      myPrintln("[RSP]init gyro done");
    }
  }

  // command = "start"
  else if(buffString_.equals("start")){
    startAutoDrive();
  }
  
  // command = "s" or "stop"
  else if(buffString_.equals("s") || buffString_.equals("stop")){
    stopAutoDrive();
  }

  // unknown command
  else {
    sprintf(buff_, "[CMD][ERR]unknown command (%s)", buf);
    myPrintln(buff_);
    myPrintln("valid command = read [filename], init gyro, start, stop, s, sv???, sp???");
  }
}

void initGyro()
{
  NsLCD.clear();
  NsLCD.moveCursor(0,0);
  NsLCD.print("init gyro...");
  NsGyro.initOrigin(10, 500);
  NsLCD.moveCursor(0,1);
  NsLCD.print("AD0 = ");
  NsLCD.moveCursor(6,1);
  dtostrf(NsGyro.getYawAD0(), -1, 1, buff_);
  NsLCD.print(buff_);
  delay(2000);
}

void setPwmServo(byte v)
{
  bPwmServo_ = v;
  pwmServo_.write(v);
  sprintf(buff_, ",sv%03d", v);
  NsLCD.moveCursor(5,1);
  NsLCD.print(buff_);
}

void setPwmSpeed(byte v)
{
  bPwmSpeed_ = v;
  pwmSpeed_.write(v);
  sprintf(buff_, "sp%03d", v);
  NsLCD.moveCursor(11,1);
  NsLCD.print(buff_);
}

void startAutoDrive()
{
  if(state_ == 1){
    myPrintln("[RSP][ERR]already started");
  } else {
    NsLCD.displayAntennaIcon(true);
    state_ = 1;
  
    // setup logging
    #ifdef LOGGER
      if(SD.exists(FILENAME_LOG)){
        if(!SD.remove(FILENAME_LOG)){
          NsLCD.clear();
          NsLCD.moveCursor(0,0);
          NsLCD.print("[ERR]SD.remove()");
          sprintf(buff_, "(%s)", FILENAME_LOG);
          NsLCD.moveCursor(0,1);
          NsLCD.print(buff_);
          waitSW1();
        }
      }
      fileLog_ = SD.open("log.csv", FILE_WRITE);
      if(!fileLog_){
        NsLCD.clear();
        NsLCD.moveCursor(0,0);
        NsLCD.print("[ERR]SD.open()");
        sprintf(buff_, "(%s)", FILENAME_LOG);
        NsLCD.moveCursor(0,1);
        NsLCD.print(buff_);
        waitSW1();
      }
      fileLog_.println("msec,c_dirId,c_dir,g_dir,dir");
    #endif

    myPrintln("[RSP]started");
  }
}

void stopAutoDrive()
{
  // reset servo
  setPwmServo(PWM_SERVO_ZER);
  setPwmSpeed(PWM_SPEED_ZER);

  if(state_ == 0){
    myPrintln("[RSP][ERR]already stopped");
  } else {
    NsLCD.displayAntennaIcon(false);
    state_ = 0;
  
    // finish logging
    #ifdef LOGGER
      if(fileLog_){
        fileLog_.close();
      }
    #endif

    myPrintln("[RSP]stopped");
  }
}

