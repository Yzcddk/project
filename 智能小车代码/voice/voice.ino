#include <SoftwareSerial.h>
SoftwareSerial voiceSerial(2,3);//rx,tx

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);//设置蓝牙的串口的波特率
   voiceSerial.begin(9600);
 
    
}

void music()
{
     char gbk[] = {0x7E,0x05,0x41,0x00,0x05,0x41,0xEF};
    voiceSerial.write(gbk,7);
    delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
   music();
   // char xunh[] = {0x7E, 0x04, 0x33, 0x02, 0x35, 0xEF};
   // delay(10000);
}
