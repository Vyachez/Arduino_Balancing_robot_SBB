#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define LED 13
#define PWM_L 7
#define PWM_R 5
#define DIR_L1 23
#define DIR_R1 22
#define SPD_INT_L 19
#define SPD_PUL_L 27
#define SPD_INT_R 18
#define SPD_PUL_R 26
#define MPU_INT 2//0
#define K_AGL_AD A0
#define K_AGL_DOT_AD A1
#define K_POS_AD A2
#define K_POS_DOT_AD A3

MPU6050 mpu;            // AD0 low = 0x68

String readString;
int n;

// tunable variables to be transferred from Webinterface via Serial
float KAD;
float KDAD;
float KPAD;
float KPDAD;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double K_angle,K_angle_dot,K_position,K_position_dot;
double K_angle_AD,K_angle_dot_AD,K_position_AD,K_position_dot_AD;
double position_add,position_dot;
double position_dot_filter;
int speed_real_l,speed_real_r;
int pwm,pwm_l,pwm_r;
int Turn_Need,Speed_Need;
float angle, angular_rate;
bool blinkState = false;
int rx_count=0;
byte buf_tmp=0;
uint8_t i2cData[14]; // Buffer for I2C data

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{  
  init_cal();
  init_IO();

  Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
  TWBR = 6; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial2.begin(9600);
  
  // initialize device
  Serial2.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial2.println(F("Testing device connections..."));
  Serial2.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  
  // load and configure the DMP
  Serial2.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(66);//66
  mpu.setYGyroOffset(60);//regulate
  mpu.setZGyroOffset(-10);//-10
  mpu.setZAccelOffset(1164); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial2.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial2.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(MPU_INT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial2.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    delay(3000);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial2.print(F("DMP Initialization failed (code "));
    Serial2.print(devStatus);
    Serial2.println(F(")"));
  }
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial2.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // angle and angular rate unit: radian
//    angle_X = ypr[2] + 0;                  // 0.017 is center of gravity offset
//    angular_rate_X = -((double)gyro[0]/131.0); // converted to radian
    angle = ypr[1] + 0.02;                   // 0.02 is center of gravity offset
    angular_rate = -((double)gyro[1]/131.0); // converted to radian
/*
    Serial.print(angle * RAD_TO_DEG);Serial.print("  ");
    Serial.print(angular_rate * RAD_TO_DEG);Serial.print("  ");
    Serial.print("\n");
*/
    control();
    PWM_calculate();
  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
  } 
}

void init_IO()
{
  // configure I/O
  pinMode(SPD_PUL_L, INPUT);//
  pinMode(SPD_PUL_R, INPUT);//
  pinMode(PWM_L, OUTPUT);//
  pinMode(PWM_R, OUTPUT);//
  pinMode(DIR_L1, OUTPUT);//
  pinMode(DIR_R1, OUTPUT);//
  pinMode(LED, OUTPUT);
  pinMode(A0, INPUT);digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);digitalWrite(A1, HIGH);
  pinMode(A2, INPUT);digitalWrite(A2, HIGH);
  pinMode(A3, INPUT);digitalWrite(A3, HIGH);
  // configure external interruption
  attachInterrupt(SPD_INT_L, speed_int_l, RISING);
  attachInterrupt(SPD_INT_R, speed_int_r, RISING);
}

void init_cal()
{
  K_angle = 34 * 25.6;		//换算系数：256/10 =25.6；
  K_angle_dot = 2 * 25.6;  //2		//换算系数：256/10 =25.6;
  K_position = 0.8 * 0.209; //0.8		//换算系数：(256/10) * (2*pi/(64*12))=0.20944；//256/10:电压换算至PWM，256对应10V；
  K_position_dot = 1.09 * 20.9;	//1.09	//换算系数：(256/10) * (25*2*pi/(64*12))=20.944;
}

void split(String results[], int len, String input, char spChar) {
  String temp = input;
  for (int i=0; i<len; i++) {
    int idx = temp.indexOf(spChar);
    results[i] = temp.substring(0,idx);
    temp = temp.substring(idx+1);
  }  
}

void control()
{
  if (Serial2.available() > 0) {
    char readChar[64];
    Serial2.readBytesUntil(33,readChar,64);
    String read_ = String(readChar);
    //Serial.println(readChar);
    int idx1 = read_.indexOf('%');
    int idx2 = read_.indexOf('$');
    // separate command from associated data
    String cmd = read_.substring(1,idx1);
    String data = read_.substring(idx1+1,idx2);
    
    String sdata[4];
    split(sdata,4,data,'%'); 
    
    sdata[0].toCharArray(readChar, sizeof(readChar));
    KAD = atof(readChar);
    sdata[1].toCharArray(readChar, sizeof(readChar));
    KDAD = atof(readChar);
    sdata[2].toCharArray(readChar, sizeof(readChar));
    KPAD = atof(readChar);
    sdata[3].toCharArray(readChar, sizeof(readChar));
    KPDAD = atof(readChar);
}

/*{
  if(!digitalRead(A0)) Speed_Need = 40;
  else if(!digitalRead(A1)) Speed_Need = -40;

  if(!digitalRead(A2)) Turn_Need = 150;
  else if(!digitalRead(A3)) Turn_Need = 150;
*/
  if(++rx_count > 200)
  {
    rx_count = 0;
    Speed_Need = 0;
    Turn_Need = 0;
  }
}
void PWM_calculate(void)	
{
  
  K_angle_AD = -2.8 + KAD;//(analogRead(K_AGL_AD)-512) * 0.03;
  K_angle_dot_AD = -2.28 + KDAD;//(analogRead(K_AGL_DOT_AD)-512) * 0.03;
  K_position_AD = -0.31 + KPAD;//-0.31(analogRead(K_POS_AD)-512) * 0.0007;
  K_position_dot_AD = -0.31 + KPDAD;//-0.31(analogRead(K_POS_DOT_AD)-512) * 0.0007;
  
  position_dot = (speed_real_l + speed_real_r)*0.5;  //利用编码器求车轮的平均速度

  position_dot_filter*=0.95;		//车轮速度低通滤波
  position_dot_filter+=position_dot*0.05;
  
  position_add+=position_dot_filter;  //求得车轮的位置
  position_add+=Speed_Need;  //求得所期望的车轮位置
  	
  if(position_add<-10000)
    position_add=-10000;
  else if(position_add>10000)
    position_add=10000;

  //求总体的PWM
  pwm = K_angle * angle * K_angle_AD
      + K_angle_dot * angular_rate * K_angle_dot_AD
      + K_position * position_add * K_position_AD
      + K_position_dot * position_dot_filter * K_position_dot_AD;
      
  pwm_r = pwm + Turn_Need;
  pwm_l = pwm + Turn_Need;
  pwm_out(pwm_l,pwm_r);
/*
  Serial.write(char(gyroYrate*RAD_TO_DEG+128));
  Serial.write(char(kalAngleY*RAD_TO_DEG+128));
  Serial.write(char(128));
*/
/*
  Serial.print(gyroYrate*RAD_TO_DEG); Serial.print("\t");
  Serial.print(kalAngleY*RAD_TO_DEG);Serial.print("\t");
  Serial.print("\n");
*/
/*
  Serial.print(K_angle_AD);Serial.print("\t");
  Serial.print(K_angle_dot_AD);Serial.print("\t");
  Serial.print(K_position_AD);Serial.print("\t");
  Serial.print(K_position_dot_AD);Serial.print("\t");
  Serial.print("\r\n");


  Serial.print(speed_real_l);Serial.print("\t");
  Serial.print(speed_real_r);Serial.print("\t");
  Serial.print("\r\n");
*/
/*
  Serial2.print("\n");
  Serial2.print(K_angle_AD);
  Serial2.print("   ");
  Serial2.print(K_angle_dot_AD);
  Serial2.print("   ");
  Serial2.print(K_position_AD);
  Serial2.print("   ");
  Serial2.print(K_position_dot_AD);
*/
  speed_real_l = 0;
  speed_real_r = 0;
}

void pwm_out(int l_val,int r_val)
{
  if (l_val<0)
  {
    digitalWrite(DIR_L1, LOW);
    //Serial.println(" LOW"); 
    l_val=-l_val;
  }
  else
  {
    digitalWrite(DIR_L1, HIGH);
    //Serial.println(" HIGH");
  }
  
  if (r_val<0)
  {
    digitalWrite(DIR_R1, LOW);
    r_val=-r_val;
  }
  else
  {
    digitalWrite(DIR_R1, HIGH);
  }
  l_val=l_val+2;
  r_val=r_val;
  analogWrite(PWM_L, l_val>255? 255:l_val);
  //Serial.print(l_val>255? 255:l_val);
  analogWrite(PWM_R, r_val>255? 255:r_val);
}

void speed_int_l()
{
  if (digitalRead(SPD_PUL_L))
    speed_real_l-=1;
  else
    speed_real_l+=1;
}

void speed_int_r()
{
  if (digitalRead(SPD_PUL_R))
    speed_real_r-=1;
  else
    speed_real_r+=1;
}
