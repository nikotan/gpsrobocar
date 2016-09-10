// GPS robot car control sketch
// NIKO[2009/10/01]
// NIKO[2009/09/29]

#include <math.h>
#include "Wire.h"
#include "Servo.h"
#include "NsLCD.h"
#include "NsGH81.h"

// serial baud rate
#define BAUD_RATE_LOCAL 9600  // Serial
#define BAUD_RATE_NMEA  9600  // Serial1
#define BAUD_RATE_XBEE  9600  // Serial2
#define BAUD_RATE_GH81  9600  // Serial3

// GPS module's LED pins
#define GPS_LED1 9
#define GPS_LED2 8

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

// cmd serial port setting pins
#define PIN_CMD_LOCAL 52
#define PIN_CMD_XBEE  53


char buff_[20];
char buffCmd_[20];
char buffNMEA_[100];
byte idxCmd_  = 0;
byte idxNMEA_ = 0;
byte cmd1_ = 90;
byte cmd2_ = 90;
int  d_ = 0;

HardwareSerial* pcSerial_ = &Serial;
Servo pwmServo_;
Servo pwmSpeed_;

unsigned long timeOld_;
unsigned long timeNew_;


void setup()
{
  // initialize I2C LCD
  Wire.begin( 0 ); // I2C master mode
  NsLCD.init();
  NsLCD.clear();
  NsLCD.displayAntennaIcon(true);
  
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
  Serial.begin(BAUD_RATE_LOCAL);
  Serial1.begin(BAUD_RATE_NMEA);
  Serial2.begin(BAUD_RATE_XBEE);
  Serial3.begin(BAUD_RATE_GH81);

  // initialize cmd serial port
  pinMode(PIN_CMD_LOCAL,INPUT);
  pinMode(PIN_CMD_XBEE ,INPUT);
  digitalWrite(PIN_CMD_LOCAL,HIGH); // turn on pullup registor
  digitalWrite(PIN_CMD_XBEE ,HIGH); // turn on pullup registor
  delay(10);
  if(digitalRead(PIN_CMD_XBEE)==LOW){
    pcSerial_ = &Serial2;
  }

  // initialize GH81
  NsGH81.init(recieveLocation,recieveTime,GPS_LED1,GPS_LED2);
  NsGH81.setStartCmd(buff_);
  Serial3.print(buff_);
  
  // initialize timer
  timeOld_ = millis();
  timeNew_ = millis();
}

void loop()
{
  // check timer
  timeNew_ = millis();
  if(timeNew_-timeOld_>1000){
    pwmServo_.write(PWM_SERVO_ZER);
    pwmSpeed_.write(PWM_SPEED_ZER);
    timeOld_ = timeNew_;
  }

  // rcv command data from PC
  //  - format: servo(0-180),speed(0-180)\n
  if(pcSerial_->available()>0){
    timeOld_ = timeNew_;
    d_ = pcSerial_->read();
    buffCmd_[idxCmd_++] = (char)d_;
    if(idxCmd_>=20){
      idxCmd_ = 0;
    }else if(d_=='\n'){
      buffCmd_[idxCmd_-1] = '\0';
      int cmd1,cmd2;
      int cnt = sscanf(buffCmd_,"%d,%d",&cmd1,&cmd2);
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
      idxCmd_ = 0;
      
      // sensing
      int vAccX = analogRead(SEN_ACC_X);
      int vAccY = analogRead(SEN_ACC_Y);
      int vAccZ = analogRead(SEN_ACC_Z);
      int vGyrP = analogRead(SEN_GYR_PIT);
      int vGyrY = analogRead(SEN_GYR_YAW);
      byte dir = getCompass();
      sprintf(buff_, "sen,%d,%d,%d", vAccX, vAccY, vAccZ);
      pcSerial_->print(buff_);
      sprintf(buff_, ",%d,%d,%d", vGyrP, vGyrY, dir);
      pcSerial_->println(buff_);
    }
  }
  
  // rcv NMEA data
  if(Serial1.available()>0){
    d_ = Serial1.read();
    buffNMEA_[idxNMEA_++] = (char)d_;
    if(idxNMEA_>=100){
      idxNMEA_ = 0;
    }else if(d_=='\n'){
      buffNMEA_[idxNMEA_-2] = '\0';
      pcSerial_->println(buffNMEA_);
      idxNMEA_ = 0;
    }
  }

  // rcv GH81 data
  if(Serial3.available()>0){
    d_ = Serial3.read();
    NsGH81.update(d_);
  }
}

// rcv GH81 location result
void recieveLocation(NSGH81_LOCATION g)
{
  int nbat = 0;
  if(g.nsat==0){
    nbat = 0;
  }else if(g.nsat<4){
    nbat = 1;
  }else if(g.nsat<6){
    nbat = 2;
  }else{
    nbat = 3;
  }
  NsLCD.displayBatteryIcon(true,nbat);
  // write to serial
  pcSerial_->print("gps,");
  sprintf(buff_, "%d,%d,", g.lat_deg, g.lat_min);
  pcSerial_->print(buff_);
  sprintf(buff_, "%d,%d,", g.lon_deg, g.lon_min);
  pcSerial_->print(buff_);
  sprintf(buff_, "%d,%d,%d", g.alt, g.nsat, g.status);
  pcSerial_->println(buff_);
}

// rcv GH81 time result
void recieveTime(NSGH81_TIME g)
{
  sprintf(buff_, "%02d/%02d %02d:%02d:%02d", g.month, g.day, g.hour, g.min, g.sec);
  NsLCD.moveCursor(0,1);
  NsLCD.print(buff_);
}

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
