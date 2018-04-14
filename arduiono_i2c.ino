#include<Servo.h>
#include<Wire.h>

//create servo objects for servo 1 and 2
Servo servo1;
Servo servo2;


void setup() {
  //set up the servo pins
  servo1.attach(10); //pin10 on arduino for servo1
  servo2.attach(11); //pin11 on arduino for servo2

  //setup the i2c bus
  Wire.begin(8); //set address to 8
  Wire.onReceive(event);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
 delay(1);
}

void event(int howMany){
  int num_bytes = Wire.available();
  //Serial.print("number of bytes: ");
  //Serial.println(num_bytes);
  for(int i=0; i<num_bytes;i++){
    int val = Wire.read();//get 10 bit value from RPi
    val = map(val,0,255,0,180); //map the value to a dgree command
    if(i==1){
      servo1.write(val);
      //Serial.print("servo1: ");
      //Serial.println(val);
    }
    if(i==2){
      servo2.write(val);
      //Serial.print("servo2: ");
      //Serial.println(val);
    }
  }
}

