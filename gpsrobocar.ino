// GPS robot car control sketch
// NIKO[2016/09/11]
// NIKO[2009/10/01]
// NIKO[2009/09/29]

#include <math.h>
#include "Wire.h"
#include "Servo.h"
#include "NsLCD.h"
#include "NsGH81.h"
#include "NsSerialBuffer.h"

// DEBUG mode
#define DEBUG

// serial baud rate
#define BAUD_RATE_USB  9600  // Serial
#define BAUD_RATE_NMEA 9600  // Serial1 (SiRF star III)
#define BAUD_RATE_XBEE 9600  // Serial2 (XBee)
#define BAUD_RATE_GH81 9600  // Serial3 (gps robot car shield)

// GPS module's LED pins
#define GPS_LED1 9
#define GPS_LED2 8
//#define GPS_SW1 10

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
#define SEN_CPS_0   47
#define SEN_CPS_1   45
#define SEN_CPS_2   43

// sensor interval
#define SEN_INTERVAL_MSEC 100

// serial port setting pins (GND to use)
#define PIN_CMD_USB  52
#define PIN_CMD_XBEE 53

// length of buffers
#define LEN_BUFF      128
#define LEN_BUFF_SER  128
#define LEN_BUFF_SER1 128
#define LEN_BUFF_SER2 128
#define LEN_BUFF_SARR 20

//////////////////////////////////////

char buff_[LEN_BUFF];
String buffStringArr_[LEN_BUFF_SARR];
NsSerialBuffer serBuffer_;
NsSerialBuffer serBuffer1_;
NsSerialBuffer serBuffer2_;

boolean useUSB_;
boolean useXBEE_;

byte cmd1_ = 90;
byte cmd2_ = 90;

Servo pwmServo_;
Servo pwmSpeed_;

unsigned long timeNew_;
unsigned long timeOld_;
unsigned long timeOld2_;

// sensing data
float gps_lat_    = 0.0;
float gps_lon_    = 0.0;
float gps_alt_    = 0.0;
int   gps_nsat_   = 0;
int   gps_status_ = 0;
/*
int   acc_x_      = 0;
int   acc_y_      = 0;
int   acc_z_      = 0;
int   gyr_pitch_  = 0;
int   gyr_yaw_    = 0;
byte  dir         = 0;
*/


void setup()
{
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

  // initialize sensor pins
  pinMode(SEN_CPS_0,INPUT);
  pinMode(SEN_CPS_1,INPUT);
  pinMode(SEN_CPS_2,INPUT);

  // initialize serial ports
  Serial.begin(BAUD_RATE_USB);
  Serial1.begin(BAUD_RATE_NMEA);
  Serial2.begin(BAUD_RATE_XBEE);
  Serial3.begin(BAUD_RATE_GH81);
  serBuffer_.init(recieveSerialUSB, LEN_BUFF_SER, 1);
  serBuffer1_.init(recieveSerialNMEA, LEN_BUFF_SER1, 2);
  serBuffer2_.init(recieveSerialXBEE, LEN_BUFF_SER2, 1);

  // initialize GH81
  NsGH81.init(recieveLocation,recieveTime,GPS_LED1,GPS_LED2);
  NsGH81.setStartCmd(buff_);
  Serial3.print(buff_);
  
  // check serial port setting pins
  pinMode(PIN_CMD_USB , INPUT_PULLUP); // turn on pullup registor
  pinMode(PIN_CMD_XBEE, INPUT_PULLUP); // turn on pullup registor
  delay(100);
  if(digitalRead(PIN_CMD_USB) == LOW){
    useUSB_ = true;
    NsLCD.displayOutputIcon(true); // Output Icon indicates useUSB
  } else {
    useUSB_ = false;
  }
  if(digitalRead(PIN_CMD_XBEE) == LOW){
    useXBEE_ = true;
    NsLCD.displaySoundIcon(true); // Sound Icon indicates useXBEE
  } else {
    useXBEE_ = false;
  }

  // initialize timer
  timeNew_ = millis();
  timeOld_ = timeNew_;
  timeOld2_ = timeNew_;
}

void loop()
{
  // check timer
  timeNew_ = millis();
  if(timeNew_-timeOld_>1000){
    pwmServo_.write(PWM_SERVO_ZER);
    pwmSpeed_.write(PWM_SPEED_ZER);
    timeOld_ = timeNew_;

    #ifdef DEBUG
      check_mem(); // to check memory leak
    #endif
  }

  // check timer for sensor
  if(timeNew_-timeOld2_>SEN_INTERVAL_MSEC) {
    // sensing
    int vAccX = analogRead(SEN_ACC_X);
    int vAccY = analogRead(SEN_ACC_Y);
    int vAccZ = analogRead(SEN_ACC_Z);
    int vGyrP = analogRead(SEN_GYR_PIT);
    int vGyrY = analogRead(SEN_GYR_YAW);
    byte dir = getCompass();
    sprintf(buff_, "sens,%d,%d,%d,%d,%d,%d", vAccX, vAccY, vAccZ, vGyrP, vGyrY, dir);
    myPrintln(buff_);
    timeOld2_ = timeNew_;
  }

/*
  // rcv command data from PC
  //  - format: servo(0-180),speed(0-180)\n
  if(pcSerial_->available()>0){
    timeOld_ = timeNew_;
    d_ = pcSerial_->read();
    buffSerial_[idxSerial_++] = (char)d_;
    if(idxSerial_ >= LEN_BUFF_SER){
      idxSerial_ = 0;
    }
    if(d_=='\n' && idxSerial_ > 0){
      buffSerial_[idxSerial_-1] = '\0';
      int cmd1,cmd2;
      int cnt = sscanf(buffSerial_,"%d,%d",&cmd1,&cmd2);
      if( cnt==2 ){
        // check values
        cmd1_ = cmd1;
        cmd2_ = cmd2;
        if( cmd1_>PWM_SERVO_MAX ) cmd1_ = PWM_SERVO_MAX;
        else if( cmd1_<PWM_SERVO_MIN ) cmd1_ = PWM_SERVO_MIN;
        if( cmd2_>PWM_SPEED_MAX ) cmd2_ = PWM_SPEED_MAX;
        else if( cmd2_<PWM_SPEED_MIN ) cmd2_ = PWM_SPEED_MIN;
        // display recieved commands
        sprintf(buff_, "c1=%03d,c2=%03d", cmd1_, cmd2_);
        NsLCD.moveCursor(0,0);
        NsLCD.print(buff_);
        // control servo and speed controller
        pwmServo_.write(cmd1_);
        pwmSpeed_.write(cmd2_);
      }else{
        // display command format error
        sprintf(buff_, "error(%d)", cnt);
        NsLCD.moveCursor(0,0);
        NsLCD.print(buff_);
      }
      idxSerial_ = 0;
    }
  }
*/

  // rcv Serial data (USB)
  if(Serial.available()>0){
    serBuffer_.update(Serial.read());
  }

  // rcv Serial1 data (NMEA)
  if(Serial1.available()>0){
    serBuffer1_.update(Serial1.read());
  }

  // rcv Serial2 data (XBEE)
  if(Serial2.available()>0){
    serBuffer2_.update(Serial2.read());
  }

  // rcv Serial3 data (GH81)
  if(Serial3.available()>0){
    NsGH81.update(Serial3.read());
  }
}

// rcv GH81 location result
void recieveLocation(NSGH81_LOCATION g)
{
  // write to serial
  myPrint("gh81,");
  dtostrf(g.lat_deg + g.lat_min * 0.0001 / 60, -1, 6, buff_);
  myPrint(buff_);
  myPrint(",");
  dtostrf(g.lon_deg + g.lon_min * 0.0001 / 60, -1, 6, buff_);
  myPrint(buff_);
  myPrint(",");
  dtostrf(g.alt * 0.1, -1, 1, buff_);
  myPrint(buff_);
  sprintf(buff_, ",%d,%d", g.nsat, g.status);
  myPrintln(buff_);

  // update location data
  //updateLocation();
}

// rcv GH81 time result
void recieveTime(NSGH81_TIME g)
{
  sprintf(buff_, "[%02d/%02d %02d:%02d:%02d]", g.month, g.day, g.hour+9, g.min, g.sec);
  NsLCD.moveCursor(0,0);
  NsLCD.print(buff_);
}

// rcv Serial event (USB)
void recieveSerialUSB(char* buf, int len)
{
  myPrintln(buf);
}

// rcv Serial1 event (NMEA)
void recieveSerialNMEA(char* buf, int len)
{
  String str = String(buf);
  if(str.startsWith("$GPGGA")){
    int cnt = mySplit(str, ',', buffStringArr_, LEN_BUFF_SARR);
    if(cnt > 9){
      // parse
      float lat = convertNMEADegree(buffStringArr_[2]);
      if(buffStringArr_[3].equals("S")){
        lat *= -1.0;
      }
      float lon = convertNMEADegree(buffStringArr_[4]);
      if(buffStringArr_[5].equals("W")){
        lon *= -1.0;
      }
      int status = buffStringArr_[6].toInt();
      int numSat = buffStringArr_[7].toInt();
      float hdop = buffStringArr_[8].toFloat();
      float alt = buffStringArr_[9].toFloat();

      // write to serial
      myPrint("nmea,");
      dtostrf(lat, -1, 6, buff_);
      myPrint(buff_);
      myPrint(",");
      dtostrf(lon, -1, 6, buff_);
      myPrint(buff_);
      myPrint(",");
      dtostrf(alt, -1, 1, buff_);
      myPrint(buff_);
      sprintf(buff_, ",%d,%d,", numSat, status);
      myPrint(buff_);
      dtostrf(hdop, -1, 1, buff_);
      myPrintln(buff_);

      // update location data
      gps_lat_ = lat;
      gps_lon_ = lon;
      gps_alt_ = alt;
      gps_nsat_ = numSat;
      gps_status_ = status;
      updateLocation();
    }
  }
}

void updateLocation()
{
  if(gps_status_ > 0){
    // update LCD icon
    int nbat = 0;
    if(gps_nsat_ == 0){
      nbat = 0;
    }else if(gps_nsat_ < 4){
      nbat = 1;
    }else if(gps_nsat_ < 7){
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


// rcv Serial2 event (XBEE)
void recieveSerialXBEE(char* buf, int len)
{
  myPrintln(buf);
}


void myPrintln(char* b)
{
  if(useUSB_){
    Serial.println(b);
  }
  if(useXBEE_){
    Serial2.println(b);
  }
}

void myPrint(char* b)
{
  if(useUSB_){
    Serial.print(b);
  }
  if(useXBEE_){
    Serial2.print(b);
  }
}

// read compass data
int getCompass()
{
  byte dir;
  byte val0 = digitalRead(SEN_CPS_0);
  byte val1 = digitalRead(SEN_CPS_1);
  byte val2 = digitalRead(SEN_CPS_2);
  if( val0==HIGH ){
    if( val1==HIGH ){
      if( val2==HIGH){
        dir = 7;
      }else{
        dir = 0;
      }
    }else{
      if( val2==HIGH){
        dir = 6;
      }else{
        dir = 5;
      }
    }
  }else{
    if( val1==HIGH ){
      if( val2==HIGH){
        dir = 2;
      }else{
        dir = 1;
      }
    }else{
      if( val2==HIGH){
        dir = 3;
      }else{
        dir = 4;
      }
    }
  }
  return dir;
}

// for debug
// check memory leak
void check_mem() 
{
  uint8_t* stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
  uint8_t* heapptr = stackptr;              // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);              // save value of stack pointer

  sprintf(buff_, "[DEBUG] heap ptr = 0x%08lX , stack ptr = 0x%08lX", (long)heapptr, (long)stackptr);
  myPrintln(buff_);
}


int mySplit(String str, char c, String* arr, int arrSize)
{
  String tmp = str;
  int cnt = 0;
  int idx = 0;
  while(1){
    idx = tmp.indexOf(c);
    if(idx == 0){
      tmp = tmp.substring(1);
    } else if(idx > 0) {
      arr[cnt] = tmp.substring(0, idx);
      tmp = tmp.substring(idx + 1);
      cnt++;
    } else {
      break;
    }
    if( cnt >= arrSize ){
      break;
    }
  }
  return cnt;
}

float convertNMEADegree(String str)
{
  int idx = str.indexOf('.');
  if(idx <= 3){
    return 0.0;
  } else {
    int d = str.substring(0, idx - 2).toInt();
    float m = str.substring(idx - 2).toFloat();
    return d + m / 60.0;
  }
}

