// Add libraries
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define Ultrasound Pins
const int ECHOPIN = 13 ;
const int TRIGPIN = 12;

// Define Ultrasound Speed LED Pin
const int SLEDPIN = 11;

// Define Ultrasound Proximity LED Pin
const int PLEDPIN = 6;

// Define Line Sensor Pins
const int LINEPIN_1 = 7;
const int LINEPIN_2 = 8;

// Define Servo Pin
const int SERPIN_1 = 9;
const int SERPIN_2 = 10;

// Initialise global varibles
long duration; // duration of pulse
double distance = 0; // distance
double prev_dist = 0; // previous distance 
double D_dist = 0; // change in distance from last measurement
double obsped; // speed of object
const double obsped_thresh = -20; // speed threshold of object
const int dist_thresh = 15; // distance threshold
int S_r; // line sensor right
int S_l; // line sensor left
bool FLAG_slow_down; // flag to slow down motor
bool FLAG_stop; // flag to stop motor
int count_l; // left turn counter
int count_r; // right turn counter
int count_thresh = 8; // threshold for number of turn counts
int pos = 0; // servo position
int c; // servo counter

// Create servo objects
Servo servo_1;
Servo servo_2;

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor_2 = AFMS.getMotor(1);

void setup() {
  // Assign PIN inputs
  pinMode(LINEPIN_1, INPUT);
  pinMode(LINEPIN_2, INPUT);
  pinMode(ECHOPIN, INPUT);

  // Assign PIN outputs
  pinMode(TRIGPIN, OUTPUT);
  pinMode(SLEDPIN, OUTPUT);
  pinMode(PLEDPIN, OUTPUT);

  // Attach servos
  servo_1.attach(SERPIN_1);
  servo_2.attach(SERPIN_2);

  // Intilaise Servo positons 
  servo_1.write(00);
  servo_2.write(70);

  // Begin Serial monitor
  Serial.begin(9600);
  AFMS.begin();
  c = 0;
}

void loop() {

// Programming Servo Motors

if (servo_2.read() == 80 && servo_1.read() == 0) {
servo_1.write(5);
servo_2.write(80);
}
else if (servo_2.read() == 80 && servo_1.read() == 5) {
servo_1.write(5);
servo_2.write(70);
}
else if (servo_2.read() == 70 && servo_1.read() == 5) {
servo_1.write(0);
servo_2.write(70);
}
else if (servo_2.read() == 70 && servo_1.read() == 0) {
servo_1.write(0);
servo_2.write(80);
}



// Programming Object Detection 

// Intialise motor speed and turning speed
int motor_speed = 70;
int motor_speed_t = 20;
if (count_l > count_thresh || count_r > count_thresh) {
motor_speed = 80;
motor_speed_t = 35;
}
  
// Force inital trigger state to LOW
digitalWrite(TRIGPIN, LOW);
delayMicroseconds(2);
  
// Generate ultrasound wave
digitalWrite(TRIGPIN, HIGH);
delayMicroseconds(10);

// Read response time
duration = pulseIn(ECHOPIN, HIGH); 

// Find distance (speed of sound * time / 2)
distance = duration * 0.034 / 2;

// Find change in distance 
D_dist = (distance - prev_dist);

// Find speed of obstacle (change in distance / time between pulses)
obsped = D_dist / 0.1;

// If object is within 40cm and approaching quickly, issue SLOW DOWN flag
if (obsped < obsped_thresh && distance < 40) {
Serial.println("WARNING OBJECT APROACHING QUICKLY");
digitalWrite(SLEDPIN, HIGH);
FLAG_slow_down = 1;
} else {
digitalWrite(SLEDPIN, LOW);
FLAG_slow_down = 0;
}

// If object in within tolerance (6cm), issue STOP flag 
if (distance < dist_thresh) {
Serial.println("WARNING obstruction");
digitalWrite(PLEDPIN, HIGH);
FLAG_stop = 1;
} else {
digitalWrite(PLEDPIN, LOW);
FLAG_stop = 0;
}
prev_dist = distance;

// Slow down flag slows down motors
if (FLAG_slow_down) {
motor_speed = motor_speed / 2;
Serial.println("go slow");
}

// Stop flag halts motors
if (FLAG_stop == 1) {
motor_1->setSpeed(RELEASE);
motor_2->setSpeed(RELEASE);
motor_speed = 0;
Serial.println("    stop - obs");
}


// Programming Line Detection

// Read in the values of left and right line sensors
S_r = digitalRead(LINEPIN_1);
S_l = digitalRead(LINEPIN_2);
Serial.println(S_l, S_r);

// If both values are high instruct both motors forwards
if (S_l == HIGH && S_r == HIGH) {
motor_1->setSpeed(motor_speed);
motor_2->setSpeed(motor_speed);
Serial.println("    FORWARD");
motor_1->run(FORWARD);
motor_2->run(FORWARD);

count_l = 0;
count_r = 0;
}

// If only left value is high, buggy turns right
if (S_l == HIGH && S_r == LOW) {
motor_1->setSpeed(motor_speed_t + count_l * 2);
motor_2->setSpeed(motor_speed);
motor_1->run(BACKWARD);
motor_2->run(FORWARD);
Serial.println("    left");
count_l = count_l + 1;
} 

// If only right value is high, buggy turns left
if (S_l == LOW && S_r == HIGH) {
motor_1->setSpeed(motor_speed);
motor_2->setSpeed(motor_speed_t + count_r * 2);
motor_1->run(FORWARD);
motor_2->run(BACKWARD);
Serial.println("    right");
count_r = count_r + 1;
}

// If both values are low, buggy stops
if (S_l == LOW && S_r == LOW) {
motor_1->setSpeed(30);
motor_2->setSpeed(30);
Serial.println("    STOP - BOTH BLACK");
motor_1->run(BACKWARD);
motor_2->run(BACKWARD);
Serial.println("   ");
}

// Display number of consecutive instructions turning left/right
Serial.print("        COUNT L R: ");
Serial.print(count_l);
Serial.print("       ");
Serial.println(count_r);

}
