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

float front_dist;
float right_front_dist;
float right_dist;
float back_dist;
float left_dist;
float left_front_dist;
float block;

bool state1;
bool state2;




void setup() {

  state1 = false;
  state2 = false;

  Serial.begin(9600);
  Serial2.begin(9600);

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

  obstacle_avoid();
}


//obstacle avoidance function
void obstacle_avoid() {
  if (front_dist >= min_front && left_front_dist >= min_left_front && right_front_dist >= min_right_front) {
    drive_straight(0);
  }

  if (left_front_dist < min_left_front) {
    drive_break(300);
    turn_right(150);
  }

  if (right_front_dist < min_right_front) {
    drive_break(300);
    turn_left(300);
  }

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

//This function turns the rover 90 degs to the left by setting the left motor reverse and right motor forward.
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

