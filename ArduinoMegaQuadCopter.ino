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
#include "math.h"

#define SOL_ON_MOTOR_PINI 12 //3,5,6,9,10,11
#define SAG_ON_MOTOR_PINI 6
#define SOL_ARKA_MOTOR_PINI 9
#define SAG_ARKA_MOTOR_PINI 3

#define Aileron_Roll_PIN A15 //1
#define Elevator_Pitch_PIN A14 //2
#define Throttle_PIN A13 //3
#define Rudder_Yaw_PIN A12 //4  /*#define AUXCH1 A11 #define AUXCH2 A10 #define AUXCH3 A9 #define AUXCH4 A8*/

#define MIN 956  //kumanda da T1924956 mod
#define MAX 1924
#define SMIN 1000 //servo writemicroseconds minimum
#define SMAX 2000 //maksimum
#define ARM_DELAY 5000
#define accZOffset -204 

#define PITCH_P_VAL 0.4
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1

#define ROLL_P_VAL 0.5
#define ROLL_I_VAL 0
#define ROLL_D_VAL 1

#define YAW_P_VAL 0.1//2
#define YAW_I_VAL 5//5
#define YAW_D_VAL 0//1

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20

#define SENS   0.0078128 // 8/1024
#define TODEG 57.2957795

ADXL345 acc;  int16_t ax, ay, az;
BMP085 baro;  float temperature, pressure, altitude;
HMC5883L mag; int16_t mx, my, mz;
ITG3200 gyro; int16_t gx, gy, gz; 

TinyGPSPlus gps;
Servo          a,b,c,d;

int            velocity;
float 	       velocityLast;
float          bal_roll = 0, bal_pitch = 0, bal_axes = 0; // denge durumları +100 -100
int            va, vb, vc, vd; // motor hızları //1000-2000
float          ctrl; //rc dğer kontrol
boolean        interruptLock = false;
float          ch1, ch1Last, ch2, ch2Last, ch3, ch4, ch4Last;//, ch5, ch6, ch7, ch8;
float          ypr[3]     = {0.0f, 0.0f, 0.0f};
float          yprLast[3] = {0.0f, 0.0f, 0.0f};
float          Gx, Gy, Gz, Xh, Yh, t_roll, deltaT, gyroX, gyroY, gyroZ;


PID yawReg  (&ypr[0], &bal_axes,  &ch4,   YAW_P_VAL,   YAW_I_VAL,   YAW_D_VAL,   DIRECT );
PID pitchReg(&ypr[1], &bal_pitch, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL,   REVERSE);
PID rollReg (&ypr[2], &bal_roll,  &ch1,  ROLL_P_VAL,  ROLL_I_VAL,  ROLL_D_VAL,   REVERSE);

unsigned long  rcLastChange1 = micros();
unsigned long  rcLastChange2 = micros();
unsigned long  rcLastChange3 = micros();
unsigned long  rcLastChange4 = micros(); /*unsigned long  rcLastChange5 = micros(); unsigned long  rcLastChange6 = micros(); unsigned long  rcLastChange7 = micros(); unsigned long  rcLastChange8 = micros();*/

void setup(){
  Wire.begin();
  Serial.begin(57600);
  Serial1.begin(38400);
  initRC();
  initDOF();
  initMotors();
  initReg();
 }
void loop(){
  
 /* kitle();
  va=ch3;
  vb=ch3;
  vc=ch3;
  vd=ch3;
  birak();
  updateMotors();
  */
  getYPR();                   
  computePID();
  calcVel();
  updateMotors();
  
  Serial.print(ypr[0]);
  Serial.print("   ");
  Serial.print(ypr[1]);
  Serial.print("   ");
  Serial.print(ypr[2]);
  Serial.print("   ");
  Serial.print(va);
  Serial.print("   ");
  Serial.print(vb);
  Serial.print("   ");
  Serial.print(vc);
  Serial.print("   ");
  Serial.print(vd);
  Serial.print("   ");
  Serial.print(gyroX);
  Serial.print("   ");
  Serial.print(gyroY);
  Serial.print("   ");
  Serial.print(gyroZ);
  Serial.print("   ");
  
 }
void getYPR(){
  
    acc.getAcceleration(&ax,&ay,&az); az+=accZOffset; //software calibration for z axis
    gyro.getRotation(&gx, &gy, &gz);  gx+=61; gy+=11; gz-=5;
    mag.getHeading(&mx, &my, &mz);

    deltaT = gyro.deltaTime /1000.0; //delta time; every call frequence for gyro ~= 0,0015
    
    Ax = SENS*ax; Ay = SENS*ay; Az = SENS*az;

    t_roll = sqrt((Ax*Ax) + (Az*Az));
    ypr[1] = atan2(Ay,t_roll); // pitch
    ypr[2] = atan2(Ax,Az); //roll

    //Yaw calculations from magnetometer heading
    Xh= mx*cos(ypr[1]) + mz*sin(ypr[1]);
    Yh= mx*sin(ypr[2])*sin(ypr[1]) + my*cos(ypr[2]) - mz*sin(ypr[2])*cos(ypr[1]);
    ypr[0] = atan(Yh/Xh);

    //Convert Radion YPR to Degree
    ypr[0] = ypr[0] *TODEG;
    ypr[1] = ypr[1] *TODEG;
    ypr[2] = ypr[2] *TODEG;

    gyroX += (float)(gx / 14.375) * deltaT;
    gyroY += (float)(gy / 14.375) * deltaT;
    gyroZ += (float)(gz / 14.375) * deltaT;

 }
void computePID(){

  kitle();

  ch2 = map(ch2, MIN, MAX, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, MIN, MAX, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, MIN, MAX, YAW_MIN, YAW_MAX);
  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN)  || (ch1 > ROLL_MAX))  ch1 = ch1Last;
  if((ch4 < YAW_MIN)   || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  

  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  
  birak();

 }
void calcVel(){

  kitle();
  	velocity = map(ch3, MIN, MAX, SMIN, SMAX);
  birak();
  
  velocityLast = velocity;

  va = ((((100+bal_pitch)/100)*velocity         +		((100+bal_roll)/100)*velocity)         /2 ) * ((100+bal_axes)/100);
  vb = ((((100+bal_pitch)/100)*velocity			+		(abs((-100+bal_roll)/100))*velocity)   /2 ) * (abs(-100+bal_axes)/100);
  vc = (((abs((-100+bal_pitch)/100))*velocity	+		((100+bal_roll)/100)*velocity)         /2 ) * (abs(-100+bal_axes)/100); 
  vd = (((abs((-100+bal_pitch)/100))*velocity	+		(abs((-100+bal_roll)/100))*velocity)   /2 ) * ((100+bal_axes)/100);
  
 }
void initReg(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

 }
void initDOF(){
  delay(1100);
  acc.initialize();  boolean a = acc.testConnection();  acc.setRange(1);  acc.setOffset(33,47,-127);
  baro.initialize(); boolean b = baro.testConnection();
  mag.initialize();  boolean m = mag.testConnection();
  gyro.initialize(); boolean g = gyro.testConnection(); gyro.setRate(0);
  if(!(a & b & m & g))
    initDOF();
 }
void initMotors(){
  a.attach(SOL_ON_MOTOR_PINI);   b.attach(SAG_ON_MOTOR_PINI);
  c.attach(SOL_ARKA_MOTOR_PINI); d.attach(SAG_ARKA_MOTOR_PINI);
  delay(100);
  a.writeMicroseconds(MIN); b.writeMicroseconds(MIN);
  c.writeMicroseconds(MIN); d.writeMicroseconds(MIN);
  delay(ARM_DELAY);
 }
void updateMotors(){
  a.writeMicroseconds(va);  b.writeMicroseconds(vb);
  c.writeMicroseconds(vc);  d.writeMicroseconds(vd);
 }
void initRC(){
  PCintPort::attachInterrupt(Aileron_Roll_PIN,   rcInterrupt1,   CHANGE);
  PCintPort::attachInterrupt(Elevator_Pitch_PIN, rcInterrupt2,   CHANGE);
  PCintPort::attachInterrupt(Throttle_PIN,       rcInterrupt3,   CHANGE);
  PCintPort::attachInterrupt(Rudder_Yaw_PIN,     rcInterrupt4,   CHANGE); 
  /*PCintPort::attachInterrupt(AUXCH1,             rcInterrupt5,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH2,             rcInterrupt6,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH3,             rcInterrupt7,   CHANGE); 
  PCintPort::attachInterrupt(AUXCH4,             rcInterrupt8,   CHANGE);*/
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
 }/*
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
 }*/
/**************************************/
