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

#define amberPin 6
//Ultrasonic sensor pins - trig generates ultrasound, echo reads ultrasound
#define trig 11
#define echo 12

// Button to start
#define buttonPin 8

//Servo pins
#define grabPin 9 //servo 2
#define armPin 10 //servo 1

//Colour sensing infrared sensor
#define IR 7

//Ultrasonic sensors on the side
//#define trig2 12
#define echo2 13

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

//Parameters for ultrasonic sensor
long distance;
long duration;

int leftBranchCount = 0;
int targetBranch = 0;
bool sensor1;
bool sensor2;
bool sensor3;
bool sensor4;

int beforeGrab = 0;
bool grabbed = false;

int forwardCounter = 0;

int pickTime;

//Parameters for blinking amber LED light when button is pressed
bool amberLedState = LOW;
bool buttonState = LOW;
int buttonCounter = 0;
const long flashingInterval = 500;
unsigned long prevTime = 0;

int prevStatus = 0;
int grabCounter = 0;
//parameters assigned to the distance between the block and the robot
float averageDistance;
int totalDistance;

//Setting up motor speed
int ms1 = 160;
int ms2 = 160;
int ms3 = 0;
int ms4 = 0;

//functions
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

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(IR, INPUT);
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  pinMode(amberPin, OUTPUT);
  servoGrab.attach(grabPin);
  servoArm.attach(armPin);
  servoArm.write(135);
  servoGrab.write(147);
  Serial.begin(9600);

}


void loop() {
  // put your main code here, to run repeatedly:
  //buttonPress();
  AFMS.begin();
  motor1-> setSpeed(0);
  motor2-> setSpeed(0);
  Serial.println("begun");
  digitalWrite(amberPin, LOW);
  while(digitalRead(buttonPin) !=1 ){}
  Serial.println("start button pressed");
  startMotor();
  digitalWrite(amberPin, HIGH);
  start();
  Serial.println("Start ended");
  // as we start, we move forward and turn at the second double-branch
  int currentStatus = status();
  // int sideDistance = calculateDistanceSide();
  beforeGrab = 1;
  while (beforeGrab == 1){
    //only update movement when there is a change of status
    //Serial.print("Distance side: ");
    //Serial.println(sideDistance);
    // if (prevStatus != currentStatus){
    //   steer(currentStatus);
    // }
    // prevStatus = currentStatus;
    // currentStatus = status();
    // Serial.print("Current status: ");
    // steer(currentStatus);
    // Serial.println(currentStatus);
    steer(currentStatus);
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
  // motor3->setSpeed(ms3);
  // motor4->setSpeed(ms4);
  motor1-> run(BACKWARD);
  motor2-> run(BACKWARD);
  // motor3-> run(BACKWARD);
  // motor4-> run(BACKWARD);
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
    motor2-> run(BACKWARD);
    motor1-> run(BACKWARD);
    // int tunnelDistance = calculateDistanceSide();
    // if (tunnelDistance >=8 and tunnelDistance <=14){
    //   Serial.println("too far left in tunnel, turning right");
    //   ms2 = 100;
    //   ms1 = 200;
    //   motor1 -> setSpeed(ms1);
    //   motor2 -> setSpeed(ms2);
    // }
    // else if (tunnelDistance >= 3 and tunnelDistance <= 7){
    //   Serial.println("too far in right, turning left");
    //   ms1 = 100;
    //   ms2 = 200;
    //   motor1 -> setSpeed(ms1);
    //   motor2 -> setSpeed(ms2);
    // }
    // else{
    //   Serial.println("forward");
    //   ms1 = 210;
    //   ms2 = 210;
    //   motor1->setSpeed(ms1);
    //   motor2->setSpeed(ms2);
    // }
    //set straight motion speed high to go over the ramp
    ms1 = 250;
    ms2 = 250;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);

  }
  else if (statusValue == 2){
    motor2-> run(BACKWARD);
    motor1-> run(FORWARD);
    ms1 = 240;
    ms2 = 240;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);
  }
  else if (statusValue == 3){
    motor2-> run(FORWARD);
    motor1-> run(BACKWARD);
    ms1 = 240;
    ms2 = 240;
    motor1->setSpeed(ms1);
    motor2->setSpeed(ms2);
  }
  else if (statusValue == 4){
    Serial.println("Detected that it needs to grab");
    moveForward();
    grab();
    stopRobot();
    delay(3000);
    digitalWrite(amberPin, HIGH);
    pickTime = millis();
  }
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
  //set amber to zero
  motor1 -> setSpeed(0);
  motor2 -> setSpeed(0);
  digitalWrite(amberPin, LOW);
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
// this function will detect the colour of the block and set a target branch
void detectColour(){
  bool colour = digitalRead(IR);
  if (colour == true){
    targetBranch = 3;
  }
  else {
    targetBranch = 1;
  }
  // if blue block --> targetBranch = 2, if red block --> targetBranch = 1; 
  //if (red){
    //targetBranch = 0;
  //}
  //else if (blue){
    //targetBranch = 1;
  //}

}

// this function shows different combinations of line sensors and the according movement change of the robot.
int status(void){
  // These are readings of the four line sensors
  sensor1 = digitalRead(pin1);
  sensor2 = digitalRead(pin2);
  sensor3 = digitalRead(pin3);
  sensor4 = digitalRead(pin4);
  Serial.print("line sensors: ");
  Serial.print(sensor1);
  Serial.print(sensor2);
  Serial.print(sensor3);
  Serial.println(sensor4);
  
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
  else if (sensor3 and sensor4 and !sensor1 and forwardCounter == 0){
    return 4;
  }
  //else if (sensor1 and !sensor4 and grabCounter != 0 and (millis()-pickTime > 18500)){
    //return 5;}
  else if (sensor3 and sensor4 and !sensor1 and forwardCounter != 0){
    //we ignore the releasing branches before we grab the block
    return 1;
  }
  else if (sensor1 and !sensor2 and !sensor3 and !sensor4){
    return 2;
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
    //return 5;
    return 1;
  }
  else{
    return 1;
  }
}

//Grabbing mechanism
void grab(){
  servoGrab.write(80);
  delay(500);
  servoArm.write(40);
  delay(500);
  servoGrab.write(147);
  delay(500);
  servoArm.write(140); // 90 for grabber, 135 for arm
  delay(500);
  grabCounter += 1;
}

// this will tell the vehicle to rotate and release the block 
void drop(){
  servoGrab.write(90);
}
