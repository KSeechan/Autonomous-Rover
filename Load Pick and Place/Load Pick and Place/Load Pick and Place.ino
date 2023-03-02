//Project Team 3 Obstacle Avoidance Code
#include <NewPing.h>
#define MAX_DISTANCE 250  // Maximum distance we want to ping for (in centimeters).

#include <Servo.h>
Servo myservo;  // variable for servo motor fro gripper

//Define LED Pins
#define redLED A3
#define greenLED A4
#define blueLED A2
int brightness = 150;

// assign the trigger and echo pins of the ultrasonic sensors
#define trig_U1 47  //pulse sent from front sensor
#define echo_U1 46  //pulse received from front sensor

#define trig_U2 44  //pulse sent from right-front sensor
#define echo_U2 45  //pulse received from right-front sensor

#define trig_U3 40  //pulse sent from right sensor
#define echo_U3 41  //pulse received from right sensor

#define trig_U4 36  //pulse sent from back sensor
#define echo_U4 37  //pulse received from back sensor

#define trig_U5 33  //pulse sent from left sensor
#define echo_U5 32  //pulse received from left sensor

#define trig_U6 28  //pulse sent from left-front sensor
#define echo_U6 29  //pulse received from left-front sensor

#define trig_U7 23  //pulse sent from block detection sensor
#define echo_U7 22  //pulse receved from block detection sensor

//NewPing Setup of Pins and Maximum distance
NewPing sonar_front(trig_U1, echo_U1, MAX_DISTANCE);
NewPing sonar_right_front(trig_U2, echo_U2, MAX_DISTANCE);
NewPing sonar_right(trig_U3, echo_U3, MAX_DISTANCE);
NewPing sonar_back(trig_U4, echo_U4, MAX_DISTANCE);
NewPing sonar_left(trig_U5, echo_U5, MAX_DISTANCE);
NewPing sonar_left_front(trig_U6, echo_U6, MAX_DISTANCE);
NewPing sonar_block(trig_U7, echo_U7, MAX_DISTANCE);

//assign the L298N Pins
//Left Motor
int enA = 8;   //controls speed of left motor
int in1 = 10;  //controls direction of left motor
int in2 = 11;  //controls direction of left motor

//Right Motor
int enB = 9;   //controls speed of right motor
int in3 = 12;  //controls direction of right motor
int in4 = 13;  //controls direction of right motor

//assign distances(cm) to represent clear paths
int min_front = 17;
int min_left_front = 10;
int min_right_front = 10;
int min_left = 15;
int min_right = 15;

//variables to store sensor data
float front_dist;
float right_front_dist;
float right_dist;
float back_dist;
float left_dist;
float left_front_dist;
float block;

// booleans to help with localization and block pick up
bool state1;
bool state2;


void setup() {

  //set both booleans to false
  state1 = false;
  state2 = false;

  //initialize both serial communications
  Serial.begin(9600);
  Serial2.begin(9600);

  //assign servo pin and set to open position
  myservo.attach(52);
  myservo.write(180);

  //set ledPins as outputs
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  //Set all motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  delay(3000);  //delay before starting
}

void loop() {
  //get distances from sensors
  front_dist = sonar_front.ping_cm();
  right_front_dist = sonar_right_front.ping_cm();
  right_dist = sonar_right.ping_cm();
  back_dist = sonar_back.ping_cm();
  left_dist = sonar_left.ping_cm();
  left_front_dist = sonar_left_front.ping_cm();
  block = sonar_block.ping_cm();


  //getting sensor data via bluetooth
  if (Serial2.available()) {

    char ch = Serial2.read();

    if (ch == 'g') {
      Serial2.println("Front:");
      Serial2.println(front_dist);
      Serial2.println("Right-Front:");
      Serial2.println(right_front_dist);
      Serial2.println("Right:");
      Serial2.println(right_dist);
      Serial2.println("Back:");
      Serial2.println(back_dist);
      Serial2.println("Left");
      Serial2.println(left_dist);
      Serial2.println("Left-Front:");
      Serial2.println(left_front_dist);
      Serial2.println("Block:");
      Serial2.println(block);
    }
  }

  // Sensor data from serial monitor
  if (Serial.available()) {

    char ch = Serial.read();

    if (ch == 'g') {
      Serial.println("Front:");
      Serial.println(front_dist);
      Serial.println("Right-Front:");
      Serial.println(right_front_dist);
      Serial.println("Right:");
      Serial.println(right_dist);
      Serial.println("Back:");
      Serial.println(back_dist);
      Serial.println("Left");
      Serial.println(left_dist);
      Serial.println("Left-Front:");
      Serial.println(left_front_dist);
      Serial.println("Block:");
      Serial.println(block);
    }
  }

  //start by obstacle avoiding
  obstacle_avoid();

  //LOCALIZATION CODE
  //Cross Intersection Case 1 Front Facing North
  if ((front_dist > 30 && front_dist < 45) && (back_dist > 55 && back_dist < 75) && (right_dist > 55 && right_dist < 75) && (left_dist > 55 && left_dist < 75)) {
    drive_break(500);  //stop
    analogWrite(greenLED, brightness);  //turn on LED
    delay(2000);  //wait for 2 seconds
    analogWrite(greenLED, 0);  //turn off LED
    while (right_dist > 39) {  //rotate to align front of robot to face West
      turn_left(300);
      drive_break(500);
      right_dist = sonar_right.ping_cm();
    }
    drive_break(2000);  //wait 2 seconds
    drive_straight(1000);  //drive forward to break out of if statement
    state1 = true;  //set state1 to true to indicate robot is localized
  }

  //Cross Intersection Case 2 Front Facing East
  if ((front_dist > 55 && front_dist < 75) && (back_dist > 55 && back_dist < 75) && (right_dist > 55 && right_dist < 75) && (left_dist > 30 && left_dist < 45)) {
    drive_break(500);  //stop
    analogWrite(greenLED, brightness);  //turn on LED
    delay(2000);  //wait for 2 seconds
    analogWrite(greenLED, 0);  //turn off LED
    while (right_dist > 39) {  //rotate to aligh front of robot to face West
      turn_left(300);
      drive_break(500);
      right_dist = sonar_right.ping_cm();
    }
    drive_break(2000);  //wait 2 seconds
    drive_straight(1000);  //drive forward to break out of if statement
    state1 = true;  //set state1 to true to indicate robot is localized
  }

  //Cross Intersection Case 3 Front Facing South
  if ((front_dist > 55 && front_dist < 75) && (back_dist > 30 && back_dist < 45) && (right_dist > 55 && right_dist < 75) && (left_dist > 55 && left_dist < 75)) {
    drive_break(500);  //stop
    analogWrite(greenLED, brightness);  //turn on LED
    delay(2000);  //wait for 2 seconds
    analogWrite(greenLED, 0);  //turn off LED
    while (right_dist > 39) {  //rotate to align front of robot to face West
      turn_right(300);
      drive_break(500);
      right_dist = sonar_right.ping_cm();
    }
    drive_break(2000);  //wait 2 seconds
    drive_straight(1000);  //drive forward to break out of if statement
    state1 = true;  //set state1 to true to indicate robot is localized
  }

  //Cross Intersection Case 4 Front Facing West
  if ((front_dist > 55 && front_dist < 75) && (back_dist > 55 && back_dist < 75) && (right_dist > 30 && right_dist < 45) && (left_dist > 55 && left_dist < 75)) {
    drive_break(500);  //stop
    analogWrite(greenLED, brightness);  //turn on LED
    delay(2000);  //wait for 2 seconds
    analogWrite(greenLED, 0);  //turn off LED
    drive_break(2000);  //wait for 2 seconds
    drive_straight(1000);  //drive forward to break out of if statement
    state1 = true;  //set state1 to true to indicate robot is localized
  }

  //Drives to loading zone once localization has been completed.
  if (state1 == true) {
    obstacle_avoid();  //obstacle avoids until it reaches the loading zone
    if ((front_dist > 55 && front_dist < 75) && (left_dist > 1 && left_dist < 10) && (right_dist > 1 && right_dist < 10) && (back_dist > 35 && back_dist < 40)) {
      drive_break(2000);  //once at loading zone, stop
      analogWrite(redLED, brightness);  //turn on LED
      delay(2000);  //wait 2 seconds
      analogWrite(redLED, 0);  //turn off LED
      block_search();  //implement block search (once block is picked up, state2 is set to true within this function )
    }
  }

  //searches for a delivery zone.
  if (state2 == true) {
    state1 = false;  //since the robot now has the block, state1 reset to false to prevent the robot from going back into the loading zone and searching for a block
    obstacle_avoid();  //obstacle avoid until all a delivery zone is found.
    if ((front_dist < min_front) && (left_dist < min_left) && (right_dist < min_right)) {
      drive_break(2000);  //once at delivery zone, stop
      analogWrite(redLED, brightness);  //turn on LED
      analogWrite(blueLED, brightness);  //turn on LED
      delay(2000);  //wait for 2 seconds
      analogWrite(redLED, 0);  //turn off LED
      analogWrite(blueLED, 0);  //turn off LED
      myservo.write(180);  //release block;
      state2 = false;  //set state2 back to false to exit this if statement
    }
  }
}


//obstacle avoidance function
void obstacle_avoid() {
  //drives straight as long as front 3 sensors are clear
  if (front_dist >= min_front && left_front_dist >= min_left_front && right_front_dist >= min_right_front) {
    drive_straight(0);
  }

  //adjust to the right
  if (left_front_dist < min_left_front) {
    drive_break(300);
    turn_right(150);
  }

  //adjust to the left
  if (right_front_dist < min_right_front) {
    drive_break(300);
    turn_left(300);
  }

  //turn right if the front is blocked but the left_front and right_front are clear 
  if (front_dist < min_front && left_front_dist >= min_left_front && right_front_dist >= min_right_front) {
    drive_break(300);
    turn_right(150);
  }
}

//This functions allows the rover to drive straight.
//ENB needs to be 3 more.
void drive_straight(int x) {
  // turn on left motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 50);

  // turn on right motor forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 53);

  delay(x);
}


//This function turns the rover to the right by setting left motor forward and right motor backward.
void turn_left(int x) {
  //left motor reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 50);

  //right motor forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 53);

  delay(x);
}

//This function turns the rover to the left by setting the left motor reverse and right motor forward.
void turn_right(int x) {
  //left motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 50);

  //right motor reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 53);

  delay(x);
}

//turns functions stops the rover by turning off all the motors
void drive_break(int x) {
  //right motor stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  //left motor stop
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);

  delay(x);
}

//this function searches for and picks up the block
void block_search() {
  drive_straight(200);  //drive forward a bit once in the loading zone
  drive_break(500); //stop
  while (block > 35) {  //sweep the rover slowly to the left
    turn_left(100);
    drive_break(500);
    block = sonar_block.ping_cm();
  }
  if (block <= 35) {  //if the block sensor registers a value lower than 35cm, there is a block present
    while (block > 3) {  //while the block is greater than 3cm away, drive towards it
      drive_straight(300);
      drive_break(200);
      block = sonar_block.ping_cm();
    }
    if (block <= 2) {  //if the block is less than or equal to 2cm, indiacte the block has been found and close the grippers
      drive_break(0);  //completely stop
      analogWrite(blueLED, brightness);  //turn on LED
      delay(3000);  //wait for 3 seconds
      myservo.write(100);  //close grippers
      analogWrite(blueLED, 0);  //turn off LED
      state2 = true;  //set state2 to true, this allows the rover to search for a delivery zone
    }
  }
}