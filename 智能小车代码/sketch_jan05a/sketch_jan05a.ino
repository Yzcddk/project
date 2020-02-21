#include <SoftwareSerial.h>
#include<Servo.h>

SoftwareSerial bluetoothSerial(8,9);//蓝牙rx，tx


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEET 3
#define TURNRIGHT 4

int dis;

//对应的四个电机驱动
int  IN1 = 13;  //右边轮子
int  IN2 = 12;  //右边轮子
int  IN3 = 11;  //左边轮子
int  IN4 = 10;  //左边轮子

int pwmleft  = 5; //使能a，b
int pwmright = 6;
#define PIN_TRIG 3//超声波串口
#define PIN_ECHO 4

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  Serial.begin(115200);
  bluetoothSerial.begin(115200);//设置蓝牙的串口的波特率
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
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
     leftMotorForward();
     rightMotorForward();
     delay(500);
}

void car_back()
{
      leftMotorBack();
      rightMotorBack();
      delay(1000);
}

void car_left()
{
      leftMotorBack();
      rightMotorForward();
      delay(800);
}
/*void car_left1()
{
      leftMotorBack();
      rightMotorForward();
}*/

void car_right()
{
      leftMotorForward();
      rightMotorBack();
      delay(1000);
}

void car_stop()
{
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
     delay(1000);
}
/*void car_stop1()
{
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
     delay(50);
}*/


void set_speed(int speed_value,int speed_value1)
{
     analogWrite(pwmleft, speed_value);
     analogWrite(pwmright, speed_value1);
}

float ultrasonic_distance(void) 
{     
float distance;//超声波发射 
float duration; //超声波接收
digitalWrite(PIN_TRIG,LOW);    
delayMicroseconds(2); 
digitalWrite(PIN_TRIG,HIGH); 
delayMicroseconds(10); 
digitalWrite(PIN_TRIG,LOW);
duration=float(pulseIn(PIN_ECHO,HIGH));//计算超声波接收时间
distance = (duration*17)/1000;

return distance;
} 

void bluetooth()

{
  char cmd;
  bluetoothSerial.listen();
  while(bluetoothSerial.available()>0)
  {
    cmd = bluetoothSerial.read();
    switch(cmd)
    {
      case 0xa:
        car_forward();
        set_speed(180,190);
        delay(500);
        bluetoothSerial.listen();
        while(bluetoothSerial.available()>0)
        {//蓝牙读取
        cmd=bluetoothSerial.read();
        if(cmd==0xa)
        {
          set_speed(255,255);//加速
         }
        if(cmd==0xb)
        {
          set_speed(100,110);//减速
         }
        }
        break;
     case 0xb:
        car_back();
        delay(2000);
        car_stop();
        break;
     case 0xc:
        car_left();
        delay(500);
        car_stop();
        break;
     case 0xd:
        car_right();
        delay(500);
        car_stop();
        break;
     default:
        car_stop();
        break;
      }
    }
  }
  
int distances(int distance)//超声波
{
  if(distance<10)
  {
    //car_left();
    delay(500);
    dis = 1;
    }return dis;
  }
  
void loop()
{
     bluetooth();
}

