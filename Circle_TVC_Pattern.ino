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
int currentX = servoX_center;
int currentY = servoY_center;

// Setup
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Setting up Servos");

  //Attach and center servos
  ServoX.attach(servoX_pin,500,2500);
  ServoY.attach(servoY_pin,500,2500);
  ServoX.writeMicroseconds(currentX);
  ServoY.writeMicroseconds(currentY);

  Serial.println("Servos are setup and ready!");
  delay(2000);
}

// Helper function to print relevant gimbal values upon writing pwm signal to servo
void setXservo(int microsec){
  float gimbal_ang = (microsec - servoX_center)/(servoX_linkageR);
  Serial.println("Setting gimbal to " + String(gimbal_ang) + " deg, in X direction.");;
  ServoX.writeMicroseconds(microsec);
}

// Helper function to print relevant gimbal values upon writing pwm signal to servo
void setYservo(int microsec){
  float gimbal_ang = (microsec - servoY_center)/(servoY_linkageR);
  Serial.println("Setting gimbal to " + String(gimbal_ang) + " deg, in Y direction.");;
  ServoY.writeMicroseconds(microsec);
}

// Main loop
void loop() {
  float t = millis();

  // radii of servos for 8 degrees of freedom
  int xrad = 327;
  int yrad = 330;

  // Initialize current servo positions
  currentX = servoX_center;
  currentY = servoY_center + 330;

  setXservo(currentX);
  setYservo(currentY);
  
  delay(2.5);

  // Loop from 0 to 360 degrees
  for(int i = 0; i < 360; i++){
    float irad = (i*PI)/180;

    // Calculate necessary X and Y servo positions to obtain a circular pattern
    currentX = (int) servoX_center + sin(irad)*327;
    currentY = (int) servoY_center + cos(irad)*330;

    setXservo(currentX);
    setYservo(currentY);

    delay(2.5);
    
  }

  // Reset after 15 seconds
  if(t > 15000){
    setXservo(servoX_center);
    setYservo(servoY_center);
    while(1){};
  }

  delay(2.5);
  //Start loop over 
  
}
