#include <TinyGPS++.h>
#include <PinChangeInt.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <PID_v1.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include <BMP085.h>
#include <ADXL345.h>
#include <Servo.h>

/*****PIN Definitions**/
#define SOL_ON_MOTOR_PINI      12 //3,5,6,9,10,11
#define SAG_ON_MOTOR_PINI      6
#define SOL_ARKA_MOTOR_PINI    9
#define SAG_ARKA_MOTOR_PINI    3

/*****Receiver PINs**********8 channels*/
#define Aileron_Roll_PIN       A15
#define Elevator_Pitch_PIN     A14
#define Throttle_PIN           A13
#define Rudder_Yaw_PIN         A12
#define AUXCH1                 A11
#define AUXCH2                 A10
#define AUXCH3                 A9
#define AUXCH4                 A8

/**esc ve rc maksimum minimum değerleri**/
#define MIN                    1000 //?
#define MAX                    1900 
/* IMU endpoints and calibration with offsets */
#define accXmax                79
#define accXmin               -79
#define accYmax                77
#define accYmin               -77
#define accZmax                70
#define accZmin               -70
#define accZOffset            -102 // 102 daha çıkartabilmek için çünkü setoffset max -127 128
#define gyroXoffset            60
#define gyroYoffset            11
#define gyroZoffset           -4

/*********10DOF*******/
ADXL345 acc;  int16_t ax, ay, az;
BMP085 baro;  float temperature, pressure, altitude;
HMC5883L mag; int16_t mx, my, mz;
ITG3200 gyro; int16_t gx, gy, gz;

TinyGPSPlus gps;

/*Servo motorlar* a sol ön, b sağ ön, c sol arka, d sağ arka */
Servo          a,b,c,d;

/**********variables**************/
uint16_t       va, vb, vc, vd; // motor hızları
float          ctrl; //rc dğer kontrol
boolean        interruptLock = false;
uint16_t       ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8;

unsigned long  rcLastChange1 = micros();
unsigned long  rcLastChange2 = micros();
unsigned long  rcLastChange3 = micros();
unsigned long  rcLastChange4 = micros(); 
unsigned long  rcLastChange5 = micros(); 
unsigned long  rcLastChange6 = micros(); 
unsigned long  rcLastChange7 = micros(); 
unsigned long  rcLastChange8 = micros();

int32_t lastMicros;

void setup(){
    Wire.begin();
    Serial.begin(57600);
    Serial1.begin(38400); //GPS
    initDOF();
    initRC();
    initMotors();
 }

void loop(){
	/*
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
    {
      displayInfo();
      delay(500);
    }
    */
    baro.setControl(BMP085_MODE_TEMPERATURE);
    
    lastMicros = micros();
    
    while (micros() - lastMicros < baro.getMeasureDelayMicroseconds());
    
    temperature = baro.getTemperatureC();
    baro.setControl(BMP085_MODE_PRESSURE_3);
    
    while (micros() - lastMicros < baro.getMeasureDelayMicroseconds());
    
    pressure = baro.getPressure();
    altitude = baro.getAltitude(pressure);

    Serial.print("T/P/A\t");
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t");
    Serial.print(altitude);
    Serial.println("");

    delay(500);
 }

void displayInfo(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
 }

void initDOF(){
    acc.initialize();  boolean a = acc.testConnection();
    acc.setRange(2); // 8g hassasiyet
    acc.setOffset(35,47,-127);
    baro.initialize(); boolean b = baro.testConnection();
    mag.initialize();  boolean m = mag.testConnection();
    gyro.initialize(); boolean g = gyro.testConnection();
    
    if(!(a & b & m & g))
      initDOF();
 }

void initMotors(){
    a.attach(SOL_ON_MOTOR_PINI);   b.attach(SAG_ON_MOTOR_PINI);
    c.attach(SOL_ARKA_MOTOR_PINI); d.attach(SAG_ARKA_MOTOR_PINI);
    delay(10);
    a.writeMicroseconds(MIN); b.writeMicroseconds(MIN);
    c.writeMicroseconds(MIN); d.writeMicroseconds(MIN);
 }

void updateAllMotors(){
  	a.writeMicroseconds(va);  b.writeMicroseconds(vb);
  	c.writeMicroseconds(vc);  d.writeMicroseconds(vd);
 }

void initRC(){
  PCintPort::attachInterrupt(Aileron_Roll_PIN,   rcInterrupt1,   CHANGE);
  PCintPort::attachInterrupt(Elevator_Pitch_PIN, rcInterrupt2,   CHANGE);
  PCintPort::attachInterrupt(Throttle_PIN,       rcInterrupt3,   CHANGE);
  PCintPort::attachInterrupt(Rudder_Yaw_PIN,     rcInterrupt4,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH1,             rcInterrupt5,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH2,             rcInterrupt6,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH3,             rcInterrupt7,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH4,             rcInterrupt8,   CHANGE);
 }

void kitle(){ interruptLock = true; } //interrupt locks

void birak(){ interruptLock = false; }

/*****Interrupt Service Routines ******/
void rcInterrupt1(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange1;
    if(ctrl<MAX) ch1 = ctrl;
  }
  rcLastChange1 = micros();
 }
void rcInterrupt2(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange2;
    if(ctrl<MAX) ch2 = ctrl;
  }
  rcLastChange2 = micros();
 }
void rcInterrupt3(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange3;
    if(ctrl<MAX) ch3 = ctrl;
  }
  rcLastChange3 = micros();
 }
void rcInterrupt4(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange4;
    if(ctrl<MAX) ch4 = ctrl;
  }
  rcLastChange4 = micros();
 }
void rcInterrupt5(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange5;
    if(ctrl<MAX) ch5 = ctrl;
  }
  rcLastChange5 = micros();
 }
void rcInterrupt6(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange6;
    if(ctrl<MAX) ch6 = ctrl;
  }
  rcLastChange6 = micros();
 }
void rcInterrupt7(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange7;
    if(ctrl<MAX) ch7 = ctrl;
  }
  rcLastChange7 = micros();
 }
void rcInterrupt8(){
  if(!interruptLock)
  {
    ctrl = micros() - rcLastChange8;
    if(ctrl<MAX) ch8 = ctrl;
  }
  rcLastChange8 = micros();
 }
/**************************************/