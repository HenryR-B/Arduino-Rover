//#include <SharpIR.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <Servo.h>

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define IR_SENSOR_PIN A0

#define ONE_SECOND 1000

#define PING_DISTANCE 20

#define LED1 3
#define LED2 4
#define LED3 5
#define LED4 6
#define LED5 7

#define SERVO_PIN 10

#define BATTERY_PIN A1

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//SharpIR irSensor(GP2Y0A02YK0F, IR_SENSOR_PIN);

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *leftMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor1 = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor2 = AFMS.getMotor(3);

Servo servo1;

int distanceFront;
int distance2;
int pos = 0;

double currentMillis;
double lastMillis = 0;
double turnMillis = 0;
double servoCurrentMillis;
double servoLastMillis = 0;
int lastLED = LED5;
int currentLED = LED1;
boolean leftTurn = false;

int servoPos = 90;
int lastServoPos = 90;
boolean increasing = true;

double voltage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  
  AFMS.begin();
  leftMotor1->setSpeed(156);
  leftMotor2->setSpeed(156);
  rightMotor1->setSpeed(150);
  rightMotor2->setSpeed(150);  
  
  servo1.attach(SERVO_PIN);

  for (pos = 90; pos <= 135; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);   
    delay(15);              // tell servo to go to position in variable 'pos'
  }
  for (pos = 135; pos >= 35; pos--) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);
    delay(15);  
  }
  for (pos = 45; pos <= 90; pos++) {
    servo1.write(pos);
    delay(15);
  }
  
}

void loop() {
  servoCurrentMillis = millis();

  voltage = analogRead(BATTERY_PIN);
  Serial.println(voltage);


  if (servoCurrentMillis - servoLastMillis >= 15) {
    if (servoPos < 145 && increasing == true ) {
      servoPos += 2;
      increasing = true;
      
      servoLastMillis = millis();
    }
    else if (servoPos >= 145 && increasing == true ) {
      increasing = false;
      servoLastMillis = millis();
    }
    else if (servoPos <= 35 && increasing == false ) {
      increasing = true;

      servoLastMillis = millis();
    }
    else if ( increasing == false ) {
      servoPos -= 2;
      increasing = false;

      servoLastMillis = millis();
    }
  }

  servo1.write(servoPos);

  check();

  
  
}

void check() {
  distanceFront = sonar.ping_cm();
  //distance2 = irSensor.getDistance( false );
  
  
  currentMillis = millis();
  if ( distanceFront >= PING_DISTANCE && distanceFront != 0 ) {
    if (currentMillis - turnMillis >= ONE_SECOND) {
      forward();
    }
    if ( currentMillis - lastMillis >= 100) {
      currentLED = lastLED + 1;
      if (currentLED > LED5 ) {
        currentLED = LED1;
      }
      lastMillis = millis();
      lastLED = currentLED;
    }

    digitalWrite(currentLED, HIGH);
    for (int j = LED1; j <= LED5; j++) {
        if (j != currentLED) {
          digitalWrite(j, LOW);
        }
    }
          
  }
  else if ( distanceFront == 0 ) {
    for (int i = LED1; i <= LED5; i++) {
      digitalWrite(i, LOW);
    }
  }
  /*
  else if (distance2 <= 30) {
    backward();
    delay(500);
    turnLeft();
    delay(500);
  }
  */
  else {
    for (int i = LED1; i <= LED5; i++) {
      digitalWrite(i, HIGH);
    }
    if (servoPos <= 90 ) {
      turnRight();
      delay(400);
    }
    else {
      turnLeft();
      delay(400);
    }
    turnMillis = millis();
  }
  
}


void forward() {
  leftMotor1->run(FORWARD);
  leftMotor2->run(BACKWARD);
  rightMotor1->run(FORWARD);
  rightMotor2->run(BACKWARD);
}

void backward() {
  leftMotor1->run(BACKWARD);
  leftMotor2->run(FORWARD);
  rightMotor1->run(BACKWARD);
  rightMotor2->run(FORWARD);
}

void turnLeft() {
  leftMotor1->run(BACKWARD);
  leftMotor2->run(FORWARD);
  rightMotor1->run(FORWARD);
  rightMotor2->run(BACKWARD);
}

void turnRight() {
  leftMotor1->run(FORWARD);
  leftMotor2->run(BACKWARD);
  rightMotor1->run(BACKWARD);
  rightMotor2->run(FORWARD);
}
