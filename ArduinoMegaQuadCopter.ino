#include <helper_3dmath.h>
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
#define MIN                    956  //kumanda da T1924956 mod
#define MAX                    1924

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

/*  PID configuration */
#define PITCH_P_VAL            0.5
#define PITCH_I_VAL            0
#define PITCH_D_VAL            1

#define ROLL_P_VAL             2
#define ROLL_I_VAL             5
#define ROLL_D_VAL             1
 
#define YAW_P_VAL              2
#define YAW_I_VAL              5
#define YAW_D_VAL              1


/* Flight parameters */
#define PITCH_MIN             -30
#define PITCH_MAX              30
#define ROLL_MIN              -30
#define ROLL_MAX               30
#define YAW_MIN               -180
#define YAW_MAX                180
#define PID_PITCH_INFLUENCE    20
#define PID_ROLL_INFLUENCE     20
#define PID_YAW_INFLUENCE      20

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
int            va, vb, vc, vd; // motor hızları
int            v_ad, v_bc;
float          bal_axes;
float          bal_roll, ball_pitch;

float          ctrl; //rc dğer kontrol
boolean        interruptLock = false;
uint16_t       ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8;
int32_t        lastMicros; //barometre için

Quaternion     q;                          // quaternion for imu output
VectorFloat    gravity;                   // gravity vector for ypr
float          ypr[3]     = {0.0f, 0.0f, 0.0f};       // yaw pitch roll values
float          yprLast[3] = {0.0f, 0.0f, 0.0f};


PID            yawReg  (&ypr[0], &bal_axes,   &ch4, YAW_P_VAL,   YAW_I_VAL,   YAW_D_VAL,   DIRECT  );
PID            pitchReg(&ypr[1], &ball_pitch, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE );
PID            rollReg (&ypr[2], &ball_roll,  &ch1, ROLL_P_VAL,  ROLL_I_VAL,  ROLL_D_VAL,  REVERSE );

unsigned long  rcLastChange1 = micros();
unsigned long  rcLastChange2 = micros();
unsigned long  rcLastChange3 = micros();
unsigned long  rcLastChange4 = micros(); 
unsigned long  rcLastChange5 = micros(); 
unsigned long  rcLastChange6 = micros(); 
unsigned long  rcLastChange7 = micros(); 
unsigned long  rcLastChange8 = micros();



void setup(){
  Wire.begin();
  Serial.begin(57600); //only in debug delete it later
  Serial1.begin(38400); //GPS
  initMotors();
  initDOF();
  initRC();
  initRegulators();
  initBalancing();
 }

void loop(){
 
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors();
 
 }

void initBalancing(){

  bal_axes    = 0;
  ball_roll   = 0;
  ball_pitch  = 0;

}

void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

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
  delay(100);
  a.writeMicroseconds(MIN); b.writeMicroseconds(MIN);
  c.writeMicroseconds(MIN); d.writeMicroseconds(MIN);
  delay(1000);
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
