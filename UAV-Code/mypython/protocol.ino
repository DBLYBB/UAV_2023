//通过串口发送帧头0XAB+8byte数据分别对应8个gpio输出+帧尾0XFF
//波特率460800/38400
//型号ESP32C3/Arduino nano
int received_tail=0x00;
int pin[8]={3,4,5,6,7,9,10,11};
int rxbuffer[8]={0,0,0,0,0,0,0,0};
void setup() { 
  Serial.begin(38400);
  for(int i=0;i<8;i++)
  {
    pinMode(pin[i],OUTPUT);
  }
} 
void loop() { 
  while (Serial.available()) { // 内部循环 - 只要有可用的串口数据，就一直接收
    if (Serial.read() == 0xAA) { // 如果接收到帧头，则继续接收后续9个字节
    //Serial.print("head received");
      for (int i = 0; i < 8; i++) {
        while (!Serial.available()); // 等待下一个字节的到来
        rxbuffer[i] = (int)Serial.read(); // 存储每个字节
      }
      while (!Serial.available()); // 等待最后一个字节的到来
      //Serial.println("tail read");
      received_tail = Serial.read(); // 存储最后一个字节
      //Serial.print(received_tail);
      if (received_tail == 0xFF) { 
        for(int i =0;i<6;i++)
        {
          digitalWrite(pin[i],rxbuffer[i]);
          Serial.print(digitalRead(pin[i]));
        }
        for(int i =6;i<8;i++)
        {
          analogWrite(pin[i],rxbuffer[i]);
          Serial.print(analogRead(pin[i]));
        }
      }
    }
  }
}
void ssend()
{
  static char state=0;
  if(state==0)
  {
    Serial.write(0xAA);
    state=1;
  }
  else if (state==1)
  {
    Serial.write(rxbuffer[0]);
    state=2;
  }
  else if (state==2)
  {
    Serial.write(rxbuffer[1]);
    state=3;
  }
  else if (state==3)
  {
    Serial.write(rxbuffer[2]);
    state=4;
  }
  else if (state==4)
  {
    Serial.write(0xFF);
    state=0;
  }
}