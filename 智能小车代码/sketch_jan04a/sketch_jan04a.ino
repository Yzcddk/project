#include <SoftwareSerial.h>
#include<Servo.h>
SoftwareSerial bluetoothSerial(8,9);


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEET 3
#define TURNRIGHT 4

//对应的四个电机驱动
int  IN1 = 13;  //右边轮子|L298N接口IN1，IN2，IN3，IN4
int  IN2 = 12;  //右边轮子
int  IN3 = 11;  //左边轮子
int  IN4 = 10;  //左边轮子

int pwmleft  = 5; //L298N使能接口ENA，ENB
int pwmright = 6;//使能接口
#define PIN_TRIG 3//发射超声波
#define PIN_ECHO 4//接受超声波

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
  digitalWrite(IN3, HIGH);//设置端口电平
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
     delay(500);
}

void set_speed(int speed_value,int speed_value1)
{
     analogWrite(pwmleft, speed_value);//设置PWM脉冲，调节发动机转速
     analogWrite(pwmright, speed_value1);
}
float ultrasonic_distance(void) //超声波距离监控
{     
float distance; 
float duration;//时长
digitalWrite(PIN_TRIG,LOW);    
delayMicroseconds(2); 
digitalWrite(PIN_TRIG,HIGH); 
delayMicroseconds(10); 
digitalWrite(PIN_TRIG,LOW);
duration=float(pulseIn(PIN_ECHO,HIGH));
distance = (duration*17)/1000;

return distance;
} 

void distances(int distance,int i)
{   
    
     if(distance<=50)
       {
        set_speed(100,110);
           car_back();
          //car_left1();
          delay(800);
           return loop(); 
       }
    else
       return  i+=15;
       
}
void  duobi()
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

void lanya()
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
      delay(2000);
      car_stop();
       Serial.println("recv");
      break;
     case 0xb:
      car_back();
      delay(2000);
      car_stop();
       Serial.println("recv");
      break;
     case 0xc:
      car_left();
      delay(500);
      car_stop();
       Serial.println("recv");
      break;
     case 0xd:
      car_right();
      delay(500);
      car_stop();
       Serial.println("recv");
      break;
      default:
      car_stop();
      break;
  }
 }
}
void loop() 
{
  // put your main code here, to run repeatedly:
    set_speed(100,110); 
    lanyaxiaoche() ;
  
  return ;
}

