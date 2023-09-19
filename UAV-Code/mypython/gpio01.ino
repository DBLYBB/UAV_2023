#include<Stepper.h>
int received_tail = 0x00;
int inpin[7] = {3, 4, 5, 6, 7, 8, 9,};
int outpin[5]={2,10,11,12,13};
int rxbuffer[10] = {0, 0, 0, 0, 0, 0, 0, 0};
int txbuffer[3] = {0, 0, 0};
Stepper stepper(64,13,11,10,12);
const int debounceDelay = 100; // 防抖延迟时间，单位为毫秒
int state = 0;
int mode = 1;
int taskStatus = 0;
int steppersp=190;
int stepperstep=0;
 void setup() {
  Serial.begin(38400);
  for (int i = 0; i < 7; i++) {
    pinMode(inpin[i], INPUT);
  }
  for (int i = 0; i < 5; i++) {
    pinMode(outpin[i], OUTPUT);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
  }
  stepper.setSpeed(steppersp);
}

 void loop() {
  checkButtonPresses();
   while (Serial.available()) {
    if (Serial.read() == 0xAA) {
      for (int i = 0; i < 2; i++) {
        while (!Serial.available());
        rxbuffer[i] = (int)Serial.read();
      }
      while (!Serial.available());
      received_tail = Serial.read();
      if (received_tail == 0xFF) {
          digitalWrite(outpin[0],rxbuffer[0]);
          stepperstep=rxbuffer[1]-64;
          stepper.step(stepperstep);
          Serial.print(rxbuffer[0]);
          Serial.print(stepperstep);
      }
    }
  }
  //ssend();
  delay(40);
}

unsigned long lastDebounceTime[3] = {0, 0, 0}; // 上次按键状态改变的时间
 void checkButtonPresses() {
  if (digitalRead(inpin[0]) == HIGH) {
    if (millis() - lastDebounceTime[0] >= debounceDelay) {
      incrementMode();
    }
    lastDebounceTime[0] = millis();
  }
  if (digitalRead(inpin[1]) == HIGH) {
    if (millis() - lastDebounceTime[1] >= debounceDelay) {
      readBinaryNumber();
    }
    lastDebounceTime[1] = millis();
  }
  if (digitalRead(inpin[2]) == HIGH) {
    if (millis() - lastDebounceTime[2] >= debounceDelay) {
      setTaskStatus();
    }
    lastDebounceTime[2] = millis();
  } 
}


 void incrementMode() {
  mode++;
  if (mode > 3) {
    mode = 1;
  }
}
void readBinaryNumber() {
  char binaryNumber = 0;
  for (int i = 3; i <= 6; i++) {
    binaryNumber = (binaryNumber << 1)+digitalRead(inpin[i]);
  } // 将二进制转换为十六进制
  txbuffer[1] = txbuffer[2];
  txbuffer[2] = binaryNumber;
}

 void setTaskStatus() {
   taskStatus = !taskStatus;
  txbuffer[0] = taskStatus;
}
 void ssend() {
  if (state == 0) {
    Serial.write(0xAA);
    state = 1;
  } else if (state == 1) {
    Serial.write(txbuffer[0]);
    state = 2;
  } else if (state == 2) {
    Serial.write(txbuffer[1]);
    state = 3;
  } else if (state == 3) {
    Serial.write(txbuffer[2]);
    state = 4;
  } else if (state == 4) {
    Serial.write(mode);
    state = 5;
  } else if (state == 5) {
    Serial.write(0xFF);
    state = 0;
  }
}
