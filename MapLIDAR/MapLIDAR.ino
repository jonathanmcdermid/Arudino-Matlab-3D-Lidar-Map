#include <Servo.h>
#include <SoftwareSerial.h>

//turn on dry run to simulate the system with a set sensor reading

#define DRY_RUN 0

//each servo has a factory max and min pulse duration in microseconds

#define V_MIN_PULSE_DURATION 1120
#define V_MAX_PULSE_DURATION 2420
#define H_MIN_PULSE_DURATION 1000
#define H_MAX_PULSE_DURATION 2000

//in MATLAB we use the write() function which uses an angle value
//to specify what rotation the servo shaft should be at

//the horizontal servo for this program is continuous rotation, and not
//supported by MATLAB or Arduino's servo functions. if we ask it to go to a
//certain position, it will rotate continuously

//to get around this, we have to send pulses for specific amounts of time to
//make it rotate the correct amount, and then input the pulse that stops 
//rotation (NEUTRAL ANGLE)

#define CW_ANGLE_MIN 100
#define CCW_ANGLE_MIN 80
#define CW_ANGLE_FULL 180
#define CCW_ANGLE_FULL 0
#define NEUTRAL_ANGLE 90
#define MAX_TILT_ANGLE 72
#define MIN_TILT_ANGLE 170

//these variables specify the number of steps each servo should have in its
//full range. the number of points in our cloud will be V_STEPS*H_STEPS
//on a dry run, you can change these and observe the point cloud changes

#define V_STEPS 20
#define H_STEPS 80

//these variables are estimates, as no two servos are the same. the speed 
//will also depend on the direction, voltage, and duration of time we want
//it to turn. if H_STEPS is changed, the scanner will likely over/under
//rotate to varying degrees

#define FULL_TILT_DURATION 1000
#define FULL_ROTATE_DURATION 2000
#define MIN_TILT_DURATION FULL_TILT_DURATION/V_STEPS
#define MIN_ROTATE_DURATION 1.6*FULL_ROTATE_DURATION/H_STEPS

Servo vServo, hServo;
SoftwareSerial Serial1(2,3);

void setup(){
  Serial.begin(9600);
  Serial1.begin(115200);
  
  //theta, phi, and rho are our three spherical coordinates
  
  float phi = 0;
  float phiIncrement = -180/(V_STEPS - 1);
  float rho = 10;

  if(!DRY_RUN){
    
    hServo.attach(13, H_MIN_PULSE_DURATION, H_MAX_PULSE_DURATION);
    vServo.attach(8, V_MIN_PULSE_DURATION, V_MAX_PULSE_DURATION);
    //wait for servos to reach starting position
    hServo.write(NEUTRAL_ANGLE);
    vServo.write(MAX_TILT_ANGLE);
    delay(FULL_TILT_DURATION);
    hServo.detach();
    vServo.detach();
    
    for (float theta = 0; theta < 360 - 360/H_STEPS; theta += 360/H_STEPS){
      hServo.attach(13, H_MIN_PULSE_DURATION, H_MAX_PULSE_DURATION);
      hServo.write(CW_ANGLE_FULL);
      delay(MIN_ROTATE_DURATION);
      hServo.write(NEUTRAL_ANGLE);
      delay(MIN_ROTATE_DURATION);
      hServo.detach();
      phiIncrement = -phiIncrement;
      while (true){
        vServo.attach(8, V_MIN_PULSE_DURATION, V_MAX_PULSE_DURATION);
        vServo.write(MAX_TILT_ANGLE + (MIN_TILT_ANGLE - MAX_TILT_ANGLE)*phi/180);
        delay(MIN_TILT_DURATION);
        vServo.detach();
        rho = lidarDistance();
        printCartesian(rho, radians(theta), radians(phi));
        if (phi + phiIncrement > 180|| phi + phiIncrement < 0)
          break;
        else
          phi = phi + phiIncrement;
      }
    }
    //set the servos back to their starting position (as close as possible)
    hServo.attach(13, H_MIN_PULSE_DURATION, H_MAX_PULSE_DURATION);
    hServo.write(CCW_ANGLE_FULL);
    delay(FULL_ROTATE_DURATION);
    hServo.write(NEUTRAL_ANGLE);
    delay(MIN_ROTATE_DURATION);
    hServo.detach();
  }
  else {
    for (float theta = 0; theta < 360 - 360/H_STEPS; theta += 360/H_STEPS){
      phiIncrement = -phiIncrement;
      while (true){
        printCartesian(rho, radians(theta), radians(phi));
        if (phi + phiIncrement > 180|| phi + phiIncrement < 0)
          break;
        else
          phi = phi + phiIncrement;
      }
    }
  }
}

void loop(){
  
}

//function for converting spherical coordinates to cartesian

void printCartesian(float rho, float theta, float phi){
    float x = rho*sin(phi)*cos(theta);
    float y = rho*sin(theta)*sin(phi);
    float z = rho*cos(phi);
    Serial.print("x");
    Serial.print(x);
    Serial.print("y");
    Serial.print(y);
    Serial.print("z");
    Serial.print(z);
    Serial.write(13);
    Serial.write(10);
}

float lidarDistance() { 
  int dist;
  int check;
  int i;
  int uart[9];   //save data measured by LiDAR
  const int HEADER=0x59;   //frame header of data package
  while(1){
    if(Serial1.available()) {   //check if serial port has data input
      if(Serial1.read() == HEADER) {   //assess data package frame header 0x59
        uart[0]=HEADER;
        if (Serial1.read() == HEADER) {//assess data package frame header 0x59
          uart[1] = HEADER;
          for (i = 2; i < 9; i++) { //save data in array
            uart[i] = Serial1.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
            dist = uart[2] + uart[3] * 256; //calculate distance value
            if(dist < 400)
              return (float) dist;
          }
        }
      }
    }
  }
}
