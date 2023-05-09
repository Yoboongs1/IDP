#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

Servo servoArm;  // create servo object to control the arm
Servo servoGrab;  // create servo object to control the grab

//line sensors
#define pin1 2
#define pin2 3
#define pin3 4
#define pin4 5

//blinking amber LED
#define amberPin 6

//Colour sensing infrared sensor
#define IR 7

// Button to start motion
#define buttonPin 8

//Servo pins
#define grabPin 9 //this controls the claw
#define armPin 10 //this controls the arm of the grabbing mechanism


//Ultrasonic sensor pins - trig generates ultrasound, echo reads ultrasound - 1 trig is used as a common source for both ultrasonic sensors on the front and the rear
#define trig 11
#define echo 12 // ultrasonic sensor on the front
#define echo2 13 //ultrasonic sensor on the side


//setting up motor pins
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

//Parameters for ultrasonic sensor
long distance;
long duration;

//this counts the number of branches on the left after grabbing
int leftBranchCount = 0;
//this parameter will be used to select which branch the robot will drop the block on after grabbing
int targetBranch = 0;

//line sensor readings
bool sensor1;
bool sensor2;
bool sensor3;
bool sensor4;

//state before block is grabbed
int beforeGrab = 0;

//parameter set so that the motor speed only changes if the previous and current states are different
int prevStatus = 0;

//Setting up motor speed
int ms1 = 165;
int ms2 = 165;
int ms3 = 0;
int ms4 = 0;

//list of functions
void buttonPress();
int status();
bool lineSensor(int currentValue);
void amberLed();
void startMotor();
int calculateDistanceFront();
int calculateDistanceSide();
void steer(int statusValue);
void detectColour();
void start();
void grab();
void drop();
void stopRobot();
void moveForward();

void setup() {
  //setting pins as either input or output
  pinMode(buttonPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(IR, INPUT);
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  //allocating servos to grabbing mechanism
  servoGrab.attach(grabPin);
  servoArm.attach(armPin);
  //initialsing the position of the grabbing claw 
  servoArm.write(135);
  servoGrab.write(143);
  Serial.begin(9600);
}


void loop() {
  AFMS.begin();
  //initially set the motor speed to zero
  motor1-> setSpeed(0);
  motor2-> setSpeed(0);
  Serial.println("begun");
  //the amber LED is off before the button is presed
  digitalWrite(amberPin, LOW);
  //while loop prevents the execution of subsequent functions before press of button
  while(digitalRead(buttonPin) !=1 ){}
  Serial.println("start button pressed");
  //motor starts moving
  startMotor();
  //amber LED starts blinking when the start button is pressed
  digitalWrite(amberPin, HIGH);
  //separate function implemented to locate the robot to the main loop 
  start();
  Serial.println("Start ended");
  int currentStatus = status();
  int sideDistance = calculateDistanceSide();
  beforeGrab = 1;
  while (beforeGrab == 1){
    //only update movement when there is a change of status
    if (prevStatus != currentStatus){
      steer(currentStatus);
    }
    //assigns the last status as prevstatus and calls the status function to find the current status
    prevStatus = currentStatus;
    currentStatus = status();
    // sideDistance = calculateDistanceSide();

  }
  while (beforeGrab == 2){
    //might have to stop the motors here for a while
    //we detect colour after the tunnel so that rightBranchCount is not affected by the grab branches
    if (prevStatus != currentStatus){
      steer(currentStatus);
      Serial.println(status());
    }
    prevStatus = currentStatus;
    currentStatus = status();
    int distanceFront = calculateDistanceFront();
    int grabCounter = 0;
    if (distanceFront == 15 and grabCounter == 0){
        Serial.println("grab");
        grab();
        grabCounter ++;
    }
  }
  //this happens after grabbing the block
  //while (beforeGrab == 3){
    //if (tunnel){
      //move forward for certain amount of time
    //}
    //else{
      //currentStatus = status();
      //if (prevStatus != currentStatus){
      //only update movement when there is a change
        //steer(currentStatus);
      //}
      //prevStatus = currentStatus;
      //currentStatus = status();
    //}

  //}
}

//as we start, we move forward and turn at the second double-branch
void start(){
  Serial.println("starting");
  pause(1500);
  Serial.println("waited 1 second to clear first line");
  while (digitalRead(pin4)==0){}
  Serial.println("detected pin1");
  pause(1500);
  motor1 -> run(FORWARD);
  Serial.println("now turning left");
  int turnTime = millis();
  while (digitalRead(pin3) == 0 or (millis()-turnTime) <2000){}
  //if ((millis()-turnTime) > 2000) {Serial.println("did not detect pin in time");}
  Serial.println("finished turning left");
  
}

// the pause function tells the robot to move according to its last steer for a specified time
void pause(int waitingTime){
  int currentTime = millis();
  while ((millis()-currentTime) <waitingTime){}   
}


//initialising the motor
void startMotor(){
  motor1->setSpeed(ms1);
  motor2->setSpeed(ms2);
  motor1-> run(FORWARD);
  motor2-> run(FORWARD);
}

//sees whether there is a block at the front
int calculateDistanceFront(){
  //clear trig
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  //Sets the trig on HIGH state for 10 us - generating ultrasound wave
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  //Reads the echo, returns the sound wave travel time in us
  duration = pulseIn(echo, HIGH);
  //Calculate distance
  return duration * 0.034/2;
}

//sees whether there is a block on the side
int calculateDistanceSide(){
  //clear trig
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  //Sets the trig on HIGH state for 10 us - generating ultrasound wave
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  //Reads the echo, returns the sound wave travel time in us
  duration = pulseIn(echo2, HIGH);
  //Calculate distance
  return duration * 0.034/2;
}


//we will change the motor speed and direction (movement) according to the status of the line sensors - line follower
void steer(int statusValue){
  if (statusValue == 1){
    motor2-> run(FORWARD);
    motor1-> run(FORWARD);
    int tunnelDistance = calculateDistanceSide();
    //tunnel navigation
    if (tunnelDistance >=8 and tunnelDistance <=14){
      Serial.println("too far left in tunnel, turning right");
      ms1 = 200;
      ms2 = 100;
      motor1 -> setSpeed(ms1);
      motor2 -> setSpeed(ms2);
    }
    else if (tunnelDistance >= 3 and tunnelDistance <= 7){
      Serial.println("too far in right, turning left");
      ms1 = 100;
      ms2 = 200;
      motor1 -> setSpeed(ms1);
      motor2 -> setSpeed(ms2);
    }
    else{
      Serial.println("forward");
      ms1 = 210;
      ms2 = 210;
      motor1->setSpeed(ms1);
      motor2->setSpeed(ms2);
    }
    set straight motion speed high to go over the ramp
    ms1 = 250;
    ms2 = 250;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);

  }
  //slightly adjust the robot to the left
  else if (statusValue == 2){
    motor1-> run(BACKWARD);
    motor2-> run(FORWARD);
    ms1 = 240;
    ms2 = 240;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);
  }
  //slightly adjust the robot to the right
  else if (statusValue == 3){
    motor1-> run(FORWARD);
    motor2-> run(BACKWARD);
    ms1 = 240;
    ms2 = 240;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);
  }
  else if (statusValue == 4){
    stopRobot();
    int distanceSide = calculateDistanceSide();
    Serial.print("distance Side: ");
    Serial.println(distanceSide);
    if (distanceSide < 12){
      // ms1 = 180;
      // ms2 = 180;
      // motor1 -> run(FORWARD);
      // motor2 -> run(FORWARD);
      // motor1 -> setSpeed(ms1);
      // motor2 -> setSpeed(ms2);
      // pause(300);
      // stopRobot();
      // motor2 -> run(BACKWARD);
      // pause(300);
      // stopRobot();
      // grab();
      // motor1 -> run(BACKWARD);
      // motor2 -> run(FORWARD);
      // pause(300);

      //
      // ms1 = 200;
      // ms2 = 200;
      // motor1 -> run(FORWARD);
      // motor2 -> run(FORWARD);
      // motor1 -> setSpeed(ms1);
      // motor2 -> setSpeed(ms2);
      // pause(800);
      // Serial.println("move back a little");
      // stopRobot();
      ms2 = 250;
      motor2 -> run(FORWARD);
      motor1 -> setSpeed(0);
      motor2 -> setSpeed(ms2);
      int blockDistance = calculateDistanceFront();
      while (blockDistance >20){
        blockDistance = calculateDistanceFront();
        Serial.print("turning to grab");
        Serial.println(blockDistance);
      }
      ms1 = 150;
      ms2 = 150;
      motor1 -> run(FORWARD);
      motor2 -> run(FORWARD);
      motor1 -> setSpeed(ms1);
      motor2 -> setSpeed(ms2);
      while (blockDistance != 10){
        blockDistance = calculateDistanceFront();
      }
      stopRobot();
      grab();
      detectColour();
      motor1 -> setSpeed(0);
      ms2 = 250;
      motor2 -> run(BACKWARD);
      motor2 -> setSpeed(ms2);
      while (digitalRead(pin2)){
        Serial.println("turning back");
      }
      Serial.println("finished turning back");
    }
    else{
      ms1 = 210;
      ms2 = 210;      
      motor1-> run(BACKWARD);
      motor2-> run(BACKWARD);
      motor1-> setSpeed(ms1);
      motor2-> setSpeed(ms2);
    }
  }
  //rotating and dropping
  else if (statusValue == 5){
    ms1 = 240;
    ms2 = 240;
    motor1 -> run(FORWARD);
    motor2 -> run(BACKWARD);
    motor1 -> setSpeed(ms1);
    motor2 -> setSpeed(ms2);
    delay(1000);
    motor1 -> run(BACKWARD);
    delay(1200);
    stopRobot();
    drop();
  }
  
}

//to temporarily stop the robot without using delays.
void stopRobot(){
  motor1 -> setSpeed(0);
  motor2 -> setSpeed(0);
  //turn amber LED off when not moving
  digitalWrite(amberPin, LOW);
}

// this function will detect the colour of the block and set a target branch according to the colour
void detectColour(){
  bool colour = digitalRead(IR);
  //true indicates a blue block, false indicates a red block
  if (colour == true){
    targetBranch = 3;
  }
  else {
    targetBranch = 1;
  }
}

// this function shows different combinations of line sensors and the according movement change of the robot.
int status(void){
  // These are digital readings (either on or off) of the four line sensors
  sensor1 = digitalRead(pin1);
  sensor2 = digitalRead(pin2);
  sensor3 = digitalRead(pin3);
  sensor4 = digitalRead(pin4);
  //different states are allocated to different combinations of line sensors
  if (!sensor1 and !sensor2 and !sensor3 and !sensor4){
    return 1;
  }
  else if (!sensor1 and sensor2 and !sensor3 and !sensor4){
    return 2;
  }
  else if (!sensor1 and !sensor2 and sensor3 and !sensor4){
    return 3;
  }
  else if (!sensor1 and sensor2 and sensor3 and !sensor4){
    return 3;
  }
  else if (sensor4 and beforeGrab == 1){
    return 4;
  }
  else if (sensor1 and beforeGrab == 1){
    //we ignore the releasing branches before we grab the block
    return 1;
  }
  else if (sensor4 and beforeGrab == 2){
    //we ignore the grabbing branches after we grab the block
    return 1;
  }
  else if (sensor1 and beforeGrab == 2 and leftBranchCount != targetBranch){
    int counterTime = millis();
    leftBranchCount ++;
    motor1 -> run(BACKWARD);
    motor2 -> run(BACKWARD);
    motor1 -> setSpeed(200);
    motor2 -> setSpeed(200);
    pause(2000);
    return 1;
  }
  else if (sensor1 and beforeGrab == 2 and leftBranchCount == targetBranch){
    return 5;
  }
  else{
    return 1;
  }
}

void moveForward(){
  ms1 = 250;
  ms2 = 200;
  motor1 -> run(BACKWARD);
  motor2 -> run(BACKWARD);
  motor1 -> setSpeed(ms1);
  motor2 -> setSpeed(ms2);
  delay(1600);
  Serial.println("moved forward");
  stopRobot();
  forwardCounter += 1;
}

//Grabbing mechanism
void grab(){
  //open the claws
  servoGrab.write(90);
  delay(500);
  //lower the arms
  servoArm.write(50);
  delay(500);
  //close the claws
  servoGrab.write(143);
  delay(500);
  //raise the arms
  servoArm.write(135); // 90 for grabber, 135 for arm
  delay(500);
  //indicate we have grabbed the block
  beforeGrab = 2;
}

// this will tell the vehicle to release the block 
void drop(){
  servoGrab.write(90);
}
