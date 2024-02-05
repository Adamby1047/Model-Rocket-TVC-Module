#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h> 


//Define Servos and IMU
Servo ServoX;
Servo ServoY;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//Define pin connections
const int servoX_pin = 6;
const int servoY_pin = 7;

//Define Constants
const float setpointX = 0.0; //deg
const float setpointY = 0.0; //deg
const int servoX_center = 1630; //pwm 
const int servoY_center = 1610; //pwm
const float servoX_linkageR = 40.881; // us/deg
const float servoY_linkageR = 41.247; // us/deg
const int xlim = 8; //gimbal max range
const int ylim = 8; //gimbal max range
const unsigned long sampleTime = 10; //milliseconds 

//Define PID gains
float Kp = 0.35;//placeholders 
float Ki = 0.0; 
float Kd = 0.085;

//Define variables
float PID_outX; float PID_outY;
float curr_errorX; float curr_errorY;
float prev_errorX; float prev_errorY;
int pwmX = servoX_center; int pwmY = servoY_center;
float orienX; float orienY; float orienZ;
float pid_xP; float pid_xI; float pid_xD; 
float pid_yP; float pid_yI; float pid_yD;
unsigned long prevTimer; unsigned long currentTimer; float dt;
float looptime;
float actlooptime;
int t = 0; 


//Runs once
void setup() {
  
  //Initialize serial monitor
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Setting up PID controller...");
  
  //Initialize IMU
  if(!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //Initialize servos
  ServoX.attach(servoX_pin,500,2500);
  ServoY.attach(servoY_pin,500,2500);
  ServoX.writeMicroseconds(pwmX); 
  ServoY.writeMicroseconds(pwmY); 
  delay(200);

  //Properly remap IMU sensor fusion axes
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

  //Initialize time 
  prevTimer = micros();

  //Delay to let processes take place
  Serial.println("waiting...");
  delay(4000);

}

void loop() {
  //Start loop time counter
  unsigned long looptimeStart = micros();
  
  //Update all sensor values (orientation for now)
  sensors_event_t event;
  bno.getEvent(&event);
  orienX = event.orientation.y;
  orienY = event.orientation.z + 90.0;
  orienZ = event.orientation.x;

  //Run PID computational function
  PID();

  //print values to serial monitor
  printData();

  //end loop timer
  unsigned long looptimeEnd = micros();

  //Calculate and delay as needed by the desired sampling time
  looptime = looptimeEnd - looptimeStart;
  looptime = looptime / 1000.0;
  if(looptime < sampleTime){
    delay(round(sampleTime - looptime));
  }

  unsigned long actlooptimeEnd = micros();
  actlooptime = (actlooptimeEnd  - looptimeStart) / 1000.0;
  //print values to serial monitor
  printData();

}

//PID function
void PID() {

  //Update time
  currentTimer = micros();
  
  //Upon first loop instance, set the previous error accordingly
  if(t == 0){
    prev_errorX = orienX;
    prev_errorY = orienY;
  }
  t = 1;

  //Update dt
  dt = (currentTimer - prevTimer) / 1000000.0;
  
  //Update current error
  curr_errorX = orienX - setpointX;
  curr_errorY = orienY - setpointY;

  //Update proportional term
  pid_xP = Kp * curr_errorX;
  pid_yP = Kp * curr_errorY;

  //Update derivative term
  pid_xD = Kd * (curr_errorX - prev_errorX) / dt;
  pid_yD = Kd * (curr_errorY - prev_errorY) / dt;

  //Update Integral term
  pid_xI += Ki * curr_errorX * dt;
  pid_yI += Ki * curr_errorY * dt;

  //Update previous errors
  prev_errorX = curr_errorX;
  prev_errorY = curr_errorY;

  //Update previous time
  prevTimer = currentTimer;

  //Update PID sums
  PID_outX = pid_xP + pid_xI + pid_xD;
  PID_outY = pid_yP + pid_yI + pid_yD;

  //Limit PID outputs based on gimbal limits
  if(PID_outX > xlim){
    PID_outX = xlim;
  }else if(PID_outX < -xlim){
    PID_outX = -xlim;
  }else{
    PID_outX = PID_outX;
  }

  if(PID_outY > ylim){
    PID_outY = ylim;
  }else if(PID_outY < -ylim){
    PID_outY = -ylim;
  }else{
    PID_outY = PID_outY;
  }  

  //Convert the PID output to pwm signals for the servos
  pwmX = servoX_center - (PID_outX * servoX_linkageR);
  pwmY = servoY_center - (PID_outY * servoY_linkageR);

  //Write the signals to the servos
  ServoX.writeMicroseconds(pwmX);
  ServoY.writeMicroseconds(pwmY);
  
}

//Print necessary PID values for analysis/debugging
void printData() {
  Serial.print("looptime(ms): ");
  Serial.print(actlooptime,3);
  Serial.print(",");
  
  Serial.print("\tX orien(deg): ");
  Serial.print(orienX,3);
  Serial.print(",");
  
  Serial.print("\tY Orien(deg): ");
  Serial.print(orienY,3);
  Serial.print(",");
  
  Serial.print("\tZ Orien(deg): ");
  Serial.print(orienZ,3);
  Serial.print(",");

  Serial.print("\tpid_xP (deg): ");
  Serial.print(pid_xP,3);
  Serial.print(",");

  Serial.print("\tpid_yP (deg): ");
  Serial.print(pid_yP,3);
  Serial.print(",");

  Serial.print("\tpid_xI (deg): ");
  Serial.print(pid_xI,3);
  Serial.print(",");

  Serial.print("\tpid_yI (deg): ");
  Serial.print(pid_yI,3);
  Serial.print(",");

  Serial.print("\tpid_xD (deg): ");
  Serial.print(pid_xD,3);
  Serial.print(",");

  Serial.print("\tpid_yD (deg): ");
  Serial.print(pid_yD,3);
  Serial.print(",");

  Serial.print("\tPID_out (X)(deg): ");
  Serial.print(PID_outX,3);
  Serial.print(",");

  Serial.print("\tPID_out (Y)(deg): ");
  Serial.print(PID_outY,3);
  
  Serial.println("");
  
}


//Display the BNO sensor calibration status
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
