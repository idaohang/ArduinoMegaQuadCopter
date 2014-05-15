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

/*****PIN Definitions**/
#define SOL_ON_MOTOR_PINI 12 //3,5,6,9,10,11
#define SAG_ON_MOTOR_PINI 6
#define SOL_ARKA_MOTOR_PINI 9
#define SAG_ARKA_MOTOR_PINI 3

/*****Receiver PINs**********8 channels*/
#define Aileron_Roll_PIN A15
#define Elevator_Pitch_PIN A14
#define Throttle_PIN A13
#define Rudder_Yaw_PIN A12
#define AUXCH1 A11
#define AUXCH2 A10
#define AUXCH3 A9
#define AUXCH4 A8

/**esc ve rc maksimum minimum değerleri**/
#define MIN 956  //kumanda da T1924956 mod
#define MAX 1924
#define SMIN 1000 //servo writemicroseconds minimum
#define SMAX 2000 //maksimum
#define ARM_DELAY 3000

/* IMU endpoints and calibration with offsets */
#define accXmax 154
#define accXmin -162
#define accYmax 156
#define accYmin -152
#define accZmax 141
#define accZmin -141
#define gyroXoffset 33
#define gyroYoffset 47
#define gyroZoffset -127

#define accZOffset -204 

/*  PID configuration */
#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1
#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1
#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1


/* Flight parameters */
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20


#define SENS  0.0078128 // 8/1024

/*********10DOF*******/
ADXL345 acc;  int16_t ax, ay, az;
BMP085 baro;  float temperature, pressure, altitude;
HMC5883L mag; int16_t mx, my, mz;
ITG3200 gyro; int16_t gx, gy, gz;

TinyGPSPlus gps;

/*Servo motorlar* a sol ön, b sağ ön, c sol arka, d sağ arka */
Servo          a,b,c,d;

/**********variables**************/
int            velocity;
float 	       velocityLast;

float          bal_roll, bal_pitch;
float          bal_axes;

int            va, vb, vc, vd; // motor hızları
int            v_roll, v_pitch;
int 		   v_ad, v_bc;

float          ctrl; //rc dğer kontrol
boolean        interruptLock = false;
float          ch1, ch1Last, ch2, ch2Last, ch3, ch4, ch4Last, ch5, ch6, ch7, ch8;
int32_t        lastMicros; //barometre için

float          ypr[3]     = {0.0f, 0.0f, 0.0f};
float          yprLast[3] = {0.0f, 0.0f, 0.0f};

float          Gx, Gy, Gz;


PID yawReg  (&ypr[0], &bal_axes,  &ch4, YAW_P_VAL,   YAW_I_VAL,   YAW_D_VAL,   DIRECT );
PID pitchReg(&ypr[1], &bal_pitch, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg (&ypr[2], &bal_roll,  &ch1, ROLL_P_VAL,  ROLL_I_VAL,  ROLL_D_VAL,  REVERSE);

unsigned long  rcLastChange1 = micros();
unsigned long  rcLastChange2 = micros();
unsigned long  rcLastChange3 = micros();
unsigned long  rcLastChange4 = micros(); 
unsigned long  rcLastChange5 = micros(); 
unsigned long  rcLastChange6 = micros(); 
unsigned long  rcLastChange7 = micros(); 
unsigned long  rcLastChange8 = micros();

float artist;

void setup(){
  Wire.begin();
  Serial1.begin(38400); //GPS
  Serial.begin(115200);
  initRC();
  initDOF();
  initMotors();
  initBal();
  initReg();
 }

void loop(){
  
  getYPR();                   
  computePID();
  calcVel();
  updateMotors();
  
 }

void getYPR(){

    acc.getAcceleration(&ax,&ay,&az);
    az+=accZOffset; //software calibration for z axis

    Gx = SENS*ax;
    Gy = SENS*ay;
    Gz = SENS*az;

    ypr[0] = 0; // yaw calculation
    ypr[1] = atan( Gy / sqrt((Gx*Gx)+(Gz*Gz)));
    ypr[2] = atan(-Gx / Gz);
 }

void computePID(){

  kitle();

  ch2 = map(ch2, MIN, MAX, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, MIN, MAX, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, MIN, MAX, YAW_MIN, YAW_MAX);
  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  
  ypr[0] = ypr[0] * 180/M_PI; //Dereceye çevir
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  
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

  if((velocity < SMIN) || (velocity > SMAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
  v_roll = (abs(-100+bal_axes)/100)*velocity;
  v_pitch = ((100+bal_axes)/100)*velocity;
  
  va = ( ((100+bal_pitch)/100)*v_pitch			+		((100+bal_roll)/100)*v_roll			) /2;
  vb = ( ((100+bal_pitch)/100)*v_pitch			+		(abs((-100+bal_roll)/100))*v_roll	) /2;
  
  vc = ( (abs((-100+bal_pitch)/100))*v_pitch	+		((100+bal_roll)/100)*v_roll			) /2; 
  vd = ( (abs((-100+bal_pitch)/100))*v_pitch	+		(abs((-100+bal_roll)/100))*v_roll	) /2;
  
 }

void initBal(){

  bal_axes   = 0;
  bal_roll   = 0;
  bal_pitch  = 0;

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
  acc.initialize();  boolean a = acc.testConnection();
  acc.setRange(1); // 8g hassasiyet
  acc.setOffset(33,47,-127);
  Serial.println(acc.getRange());
  Serial.println(acc.getOffsetX());
  Serial.println(acc.getOffsetY());
  Serial.println(acc.getOffsetZ());
  baro.initialize(); boolean b = baro.testConnection();
  mag.initialize();  boolean m = mag.testConnection();
  gyro.initialize(); boolean g = gyro.testConnection();
  
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


