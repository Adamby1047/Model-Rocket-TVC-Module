#include <Servo.h> 

// Initialize Constants
Servo ServoX;
Servo ServoY;
const int servoX_pin = 7;
const int servoY_pin = 6;
const int servoX_center = 1630;
const int servoY_center = 1610;
const float servoX_linkageR = 40.881; //us
const float servoY_linkageR = 41.247; //us

// Setup
void setup() {
  // Begin Serial Monitor
  Serial.begin(115200);
  Serial.println("Setting up Servos");

  //Attach and center servos
  ServoX.attach(servoX_pin,500,2500);
  ServoY.attach(servoY_pin,500,2500);
  ServoX.writeMicroseconds(servoX_center);
  ServoY.writeMicroseconds(servoY_center);

  Serial.println("Servos are setup and ready!");
  delay(2000);
}

void setXservo(int microsec){
  float gimbal_ang = (microsec - servoX_center)/(servoX_linkageR);
  Serial.println("Setting gimbal to " + String(gimbal_ang) + " deg, in X direction.");;
  ServoX.writeMicroseconds(microsec);
}

void setYservo(int microsec){
  float gimbal_ang = (microsec - servoY_center)/(servoY_linkageR);
  Serial.println("Setting gimbal to " + String(gimbal_ang) + " deg, in Y direction.");;
  ServoY.writeMicroseconds(microsec);
}

// Main loop
void loop() {
  float t = millis();
  // Step 1 (++)
  setXservo(servoX_center + 327);
  setYservo(servoY_center + 330);
  //Serial.print(", " + String(t) + " ms");
  delay(250);

  // Step 2 (-+)
  setXservo(servoX_center - 327);
  setYservo(servoY_center + 330);
  delay(250);  

  // Step 3 (--)
  setXservo(servoX_center - 327);
  setYservo(servoY_center - 330);
  delay(250);  

  // Step 4 (+-)
  setXservo(servoX_center + 327);
  setYservo(servoY_center - 330);
  delay(250);  

  /*if(t > 15000){
    ServoX.writeMicroseconds(servoX_center);
    ServoY.writeMicroseconds(servoY_center);
    while(1){};
  }*/

  //Start loop over 
  
}
