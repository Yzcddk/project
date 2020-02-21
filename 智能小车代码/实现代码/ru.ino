#include <SoftwareSerial.h>
#include<Servo.h>
#include "pid.h"//循迹
SoftwareSerial bluetoothSerial(8,9);


#ifdef ARDUINO_DEBUG//循迹
int debugLeftSpeed;
int debugRightSpeed;
uint8_t debugIrs = 0;
#endif 

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEET 3
#define TURNRIGHT 4


const float motorSpeed = 100; //循迹
const int IR_PIN[] = {A0, A1, A2, A3, A4}; //
const int IN_A1 = 10;   //
const int IN_A2 = 11;   //
const int IN_B1 = 12;  //
const int IN_B2 = 13;  //
const int _pwmLeftPin = 5;
const int _pwmRightPin = 6;
pid_t pid;
float pidValue = 0;
bool turnFlag = false;

//对应的四个电机驱动
int  IN1 = 13;  //右边轮子
int  IN2 = 12;  //右边轮子
int  IN3 = 11;  //左边轮子
int  IN4 = 10;  //左边轮子

int pwmleft  = 5; 
int pwmright = 6;
#define PIN_TRIG 3
#define PIN_ECHO 4

int speed_array[2] = {100,110};//速度数组
int state = 2;//（状态机）0：后退，1：前进，2：静止
int xunji = 0;

void setup() {
  int i;
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  Serial.begin(115200);
  bluetoothSerial.begin(115200);//设置蓝牙的串口的波特率
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);

  pinMode(IN_A1, OUTPUT);//循迹引脚
  pinMode(IN_A2, OUTPUT);
  pinMode(IN_B1, OUTPUT);
  pinMode(IN_B2, OUTPUT);

  for (i = 0; i < 5; i++) 
    pinMode(IR_PIN[i], INPUT);
  pid.sampleTime = SAMPLE_TIME;
  pid.Kp = KP_VALUE;
  pid.Ki = KI_VALUE;
  pid.Kd = KD_VALUE;
  pid.error = 0;
  pid.previous_error = 0;

  Serial.begin(115200);
  delay(5000);
  
  analogWrite(_pwmLeftPin,  motorSpeed );
  analogWrite(_pwmRightPin, motorSpeed );
  Serial.println(1);

}


void leftMotorForward() //左边正转
{
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void leftMotorBack() //左边反转
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rightMotorForward() //右边正转
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void rightMotorBack() //右边发转
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void car_forward()
{
  set_speed(speed_array[0] , speed_array[1]); 
     leftMotorForward();
     rightMotorForward();
     delay(500);
}

void car_back()
{
  set_speed(speed_array[0] , speed_array[1]);  
      leftMotorBack();
      rightMotorBack();
      delay(500);
}

void car_left()
{
  set_speed(100,110); 
      leftMotorBack();
      rightMotorForward();
      delay(500);
}
/*void car_left1()
{
  set_speed(100,110); 
      leftMotorBack();
      rightMotorForward();
}
*/
void car_right()
{
  set_speed(100,110); 
      leftMotorForward();
      rightMotorBack();
      delay(500);
}

void car_stop()
{
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
     delay(500);
}

void set_speed(int speed_value_l,int speed_value_r)
{
    speed_array[0] = speed_value_l;
    speed_array[1] = speed_value_r;
    analogWrite(pwmleft, speed_array[0]);
    analogWrite(pwmright, speed_array[1]);
}
/*float ultrasonic_distance(void) 
{     
float distance; 
float duration; 
digitalWrite(PIN_TRIG,LOW);    
delayMicroseconds(2); 
digitalWrite(PIN_TRIG,HIGH); 
delayMicroseconds(10); 
digitalWrite(PIN_TRIG,LOW);
duration=float(pulseIn(PIN_ECHO,HIGH));
distance = (duration*17)/1000;

return distance;
} 
*/

/*void distances(int distance,int i)
{   
    
     if(distance<=50)
       {
        set_speed(100,110);
           car_back();
          car_left1();
          delay(800);
           return loop(); 
       }
    else
       return  i+=15;
       
}
*/
/*void  duobi()
{
      int i=100;
    while(i<250)
    {  
       set_speed(i,i+10);
       car_forward(); 
       float distance = ultrasonic_distance();
       distances(distance,i);
      }
}
*/

/*void lanya()
{
  char recv;
  bluetoothSerial.listen();
  while(bluetoothSerial.available() > 0)
  {
    recv = bluetoothSerial.read();
    Serial.println(recv);

  }
  delay(1000);
}
*/
/*
void lanyaxiaoche()
{
  char cmd;
  char recv;
  bluetoothSerial.listen();
 while(bluetoothSerial.available()>0)
 {
   Serial.println("recv");
  cmd = bluetoothSerial.read();
  switch(cmd)
  {
    case 0xa:
        car_forward();
       Serial.println("recv");
       break;
     case 0xb:
      car_back();
       Serial.println("recv");
       break;
     case 0xd:
      car_left();
       Serial.println("recv");
       break;
     case 0xc:
      car_right();
       Serial.println("recv");
      break;
      default:
      car_stop();
      break;
  }
 }
}
*/
void lanyaxiaoche()
{
  char cmd;
  bluetoothSerial.listen();
 while(bluetoothSerial.available()>0)
 {
  cmd = bluetoothSerial.read();
  switch(cmd)
  {
    case 0xa:
      if(state == 2)
       {
        car_forward();
        state = 1;
         Serial.println(speed_array[0]);
         Serial.println(speed_array[1]);
         Serial.println(state);
       }
      else if(state == 1)
       {
        addspeed();
        Serial.println(speed_array[0]);
        Serial.println(speed_array[1]);
        Serial.println(state);
       }
      else if(state == 0)
      {
        subspeed();
        Serial.println(speed_array[0]);
        Serial.println(speed_array[1]);
        Serial.println(state);
        if(speed_array[0]  <60)
         { 
          car_stop();
          state = 2;
          speed_array[0] = 100;
          speed_array[1] = 110;
         }
      }  
       break;
     case 0xb:
        if(state == 2)
        {
          car_back();
          state = 0;
          Serial.println(speed_array[0]);
          Serial.println(speed_array[1]);
          Serial.println(state);
        }
        else if(state == 1)
        {
          subspeed();
         Serial.println(speed_array[0]);
         Serial.println(speed_array[1]);
         Serial.println(state);
          if(speed_array[0] < 60)
              {
                car_stop();
                state = 2;
                speed_array[0] = 100;
                speed_array[1] = 110;
              }
         }
         else if(state == 0)
           {
            addspeed();
            Serial.println(speed_array[0]);
            Serial.println(speed_array[1]);
            Serial.println(state);
           }
       break;
     case 0xd:
      car_left();
       break;
       case 0x11:
       char aaa[]={0x7E,0x03,0x15,0x16,0xef};
       voiceSerial.write(aaa,5);
       break;
     case 0xc:
      car_right();
      break;
      default:
      car_stop();
      state = 2;
      break;
     case 0xe:
     if(xunji == 0)
      {
        xunji = 1;
        goForward();
      }
     else if(xunji == 1)
      xunji = 0;
     break;
     
  }
 }
}

void addspeed()
{
  if(speed_array[0] && speed_array[1] < 250)
  {
      set_speed(speed_array[0] += 50,speed_array[1] += 50);
  }  
}

void subspeed()
{
  if(speed_array[0] && speed_array[1] >= 50)
  {
      set_speed(speed_array[0] -= 50,speed_array[1] -= 50);
  }  
}

//get ir data and middle filter
uint8_t getIrData(void)
{
  int i, j;
  uint8_t level;
  uint8_t temp;
  uint8_t irs[9] = {0};

  for (j = 0; j < 9; j ++) {
    
      for (i = 0; i < 5; i++) {
        level = digitalRead(IR_PIN[i]);
        if (level) {
          bitSet(irs[j], i);
        } else {
          bitClear(irs[j], i);
        }
      }
  }

  for (i = 0; i < 9 - 1; i ++) {
    for (j = 0; j < 9 - i - 1; j ++) {
      if (irs[j] > irs[j + 1]) {
        temp = irs[j];
        irs[j] = irs[j + 1];
        irs[j + 1] = temp;
      }
    }
  }
  #ifdef ARDUINO_DEBUG
  debugIrs = irs[4];
#endif 

  return irs[4];
}

int calcErrorByIrsValue(uint8_t irs)
{
  int curError = pid.error;

  switch (irs) {
    case B11110: curError = -8; break;
    
    case B10000:
    case B11000: curError = -7; break;
    
    case B11100: curError = -6; break;
    case B11101: curError = -4; break;
    case B11001: curError = -2; break;
    
    case B00000:
    case B11011: curError = 0;  break;
    
    case B10011: curError = 2;  break;
    case B10111: curError = 4;  break;
    case B00111: curError = 6;  break;
    
    case B00011:
    case B00001: curError = 7;  break;
    
    case B01111: curError = 8;  break;
    case B11111: curError = pid.error > 0 ? 9 : - 9; break;
  }

  return curError;
}
  


void _sortData(int *p, int n)
{
  int temp;
  int i, j;
  
  for (i = 0; i < n - 1; i ++) {
    for (j = 0; j < n - i - 1; j ++) {
      if (p[j] > p[j + 1]) {
        temp = p[j];
        p[j] = p[j + 1];
        p[j + 1] = temp;
      }
    }
  }

  return;
}


void calcCurrentError(void)
{
  int i;
  uint8_t irs;
  float sum = 0;
  int errorData[10];

  for (i = 0; i < 10; i ++) {
    irs =  getIrData();
    errorData[i] =  calcErrorByIrsValue(irs);
  }

  _sortData(errorData, 10);

  for (i = 1; i < 10 - 1; i ++) {
    sum += errorData[i];
  }

  pid.error = sum / 8;

  return;
}

void turnRight(void)
{
  digitalWrite(IN_A1, LOW);
  digitalWrite(IN_A2, HIGH);
  digitalWrite(IN_B1, HIGH);
  digitalWrite(IN_B2, LOW);
}

void turnLeft(void)
{
  digitalWrite(IN_A1, HIGH);
  digitalWrite(IN_A2, LOW);
  digitalWrite(IN_B1, LOW);
  digitalWrite(IN_B2, HIGH);
}

void goForward(void)
{

  digitalWrite(IN_A1, LOW);
  digitalWrite(IN_A2, HIGH);
  digitalWrite(IN_B1, LOW);
  digitalWrite(IN_B2, HIGH);
}

void motorControl(float pidValue, bool turnFlag)
{
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  

  leftMotorSpeed  = constrain((motorSpeed + pidValue), -255, 255);
  rightMotorSpeed = constrain((motorSpeed - pidValue), -255, 255);

  if (turnFlag) {
    if (abs(leftMotorSpeed) > abs(rightMotorSpeed)) {
      leftMotorSpeed  = abs(leftMotorSpeed);
      rightMotorSpeed = leftMotorSpeed;
    } else {
      rightMotorSpeed =  abs(rightMotorSpeed);
      leftMotorSpeed  = rightMotorSpeed;
    }
  } else {
    leftMotorSpeed  = leftMotorSpeed  > 0 ? leftMotorSpeed  : -leftMotorSpeed;
    rightMotorSpeed = rightMotorSpeed > 0 ? rightMotorSpeed : -rightMotorSpeed;
  }

  analogWrite(_pwmLeftPin,  leftMotorSpeed );
  analogWrite(_pwmRightPin, rightMotorSpeed);

#ifdef ARDUINO_DEBUG
  debugLeftSpeed  = leftMotorSpeed ;
  debugRightSpeed = rightMotorSpeed;
#endif 

  return;
}

bool calculatePid(float *pValue)
{
  float P = 0;
  static float I = 0 ;
  float D = 0 ;
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  int timeChange = now - lastTime;

  if (timeChange < pid.sampleTime) {
    return false;
  }

  P = pid.error;
  I = I + pid.error;
  D = pid.error - pid.previous_error;

  *pValue = (pid.Kp * P) + (pid.Ki * I) + (pid.Kd * D) + 1;
  *pValue = constrain(*pValue, -motorSpeed,motorSpeed);

  pid.previous_error = pid.error;
  lastTime = now;

  return true;
}

#if ARDUINO_DEBUG
void print_debug()
{
  int i;
  String irs2bin = String(debugIrs, 2);
  int len = irs2bin.length();
  if (len < 5) {
    for (i = 0; i < 5 - len; i++) {
      irs2bin = "0" + irs2bin;
    }
  }
  
  Serial.print("IRS : ");
  Serial.print(irs2bin);
  Serial.print("   ML:");
  Serial.print(debugLeftSpeed);
  Serial.print("   MR:");
  Serial.print(debugRightSpeed);
  Serial.print("  ERROR:");
  Serial.print(pid.error, OCT);
  Serial.println();
}
#endif

void calcDirection(void)
{
  
  if (pid.error >= 7 && pid.error <= 9) {
    turnLeft();
    turnFlag = true;
  } else if (pid.error >= -9 && pid.error <= -7) {
    turnRight();
    turnFlag = true;
  } else {
    goForward();
    turnFlag = false;
  }

  return;
}


void loop() 
{   
    lanyaxiaoche();
    if(xunji == 1)
    {
     
      Serial.print(xunji + "ok");
       bool ok;
  float  pidValue;

  calcCurrentError();
  ok = calculatePid(&pidValue);
  if (ok) {
    calcDirection();
    motorControl(pidValue, turnFlag);
  }

  //delay(500);
  
#if ARDUINO_DEBUG
  print_debug();
  delay(1000);
#endif 

      }
}

