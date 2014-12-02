#include <Servo.h> 

// Wheels
#define pinDirectionA 12 // B = left wheel
#define pinDirectionB 13
#define pinSpeedA 3
#define pinSpeedB 11

// speed potentiometer
#define pinSpeetPot 2
uint32_t wheelSpeed = 0;

// servo for sensor
#define pinSensorServo 5
Servo sensorServo;
int servoAngle = 90;
uint8_t servoDirection = true;

// sonar sensor
unsigned int pingTime;

uint32_t scanAngleMin = 0;
uint32_t scanAngleMax = 180;
uint32_t scanAngle = 90;
uint32_t scanDistance;

#define pinDistanceTrigger 9
#define pinDistanceEcho 8
#define distanceThreshold 30
#define distanceSquirt 20

// valve ports
#define pinValveOpen 2
#define pinValveClose 4

long distance;

// startbutton
#define pinStartButten 10
uint32_t startButtonTimePast = 0;

// actions
byte action;
#define ACTION_STANDBY 0
#define ACTION_SEARCH_TARGET 1
#define ACTION_SQUIRT 2

void setup() {
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinSpeedA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  pinMode(pinSpeedB, OUTPUT);
  pinMode(pinSpeetPot, INPUT);
  pinMode(pinDistanceTrigger, OUTPUT);
  pinMode(pinDistanceEcho, INPUT);
  pinMode(pinStartButten, INPUT_PULLUP);
  pinMode(pinValveClose, OUTPUT);
  pinMode(pinValveOpen, OUTPUT);
  
  sensorServo.attach(pinSensorServo);
  sensorServo.write(servoAngle);
  
  Serial.begin(19200);
  delay(1000);
     
  action = ACTION_STANDBY;
  
}
 
void loop() {
  readStartButton();
  Serial.println("------------------");
  Serial.print("action: ");
  Serial.println(action);
  
  switch (action) {
    case ACTION_STANDBY:
      stopWheels();
      distance = measureDistance();
      Serial.println(distance);
      delay(500);
      break;
    
    case ACTION_SEARCH_TARGET:
      newscan();
      if (isPositionOk()) {
        if (isDistanceOk()) {
          action = ACTION_SQUIRT;
        } else {
          postionToTarget();
        }
      } else {
        postionToTarget();
        turnToTarget();
      }
      break;
      
    case ACTION_SQUIRT:
      Serial.println("Spuit!");
      digitalWrite(pinValveOpen, 1);
      digitalWrite(pinValveClose, 0);
      delay(6000);
      digitalWrite(pinValveOpen, 0);
      digitalWrite(pinValveClose, 1);
      delay(100);
      digitalWrite(pinValveOpen, 0);
      digitalWrite(pinValveClose, 0);
      action = ACTION_STANDBY;
      break;
  }
}

void readStartButton() {
  if (!digitalRead(pinStartButten)) {
    if (action == ACTION_STANDBY) {
      action = ACTION_SEARCH_TARGET;
    } else {
      action = ACTION_STANDBY;
    }
  }
}

void turnToTarget() {
  readWheelSpeed();
  turnWheels(((scanAngle < 90) ? 0 : 1), wheelSpeed, 200);
}

void postionToTarget() {
  if (measureFrontDistance() < distanceSquirt) {
    moveBackward(500);
  } else {
    moveForward(500);
  }
}

long measureDistance() {
    long duration, distance;
    digitalWrite(pinDistanceTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(pinDistanceTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinDistanceTrigger, LOW);
    duration = pulseIn(pinDistanceEcho, HIGH);
    distance = ((duration / 2) / 29.1) + 2;
    return distance;    
}

long measureFrontDistance() {
  sensorServo.write(90);
  delay(400);
  return measureDistance();
}

bool isPositionOk() {
 return ((scanAngle >= 88) && (scanAngle <= 93)); 
}

bool isDistanceOk() {
  scanDistance = measureFrontDistance();
  return (((distanceSquirt + 1) >= scanDistance) && ((distanceSquirt - 1) <= scanDistance));
}

void newscan() {
    scanAngleMin = 0;
    scanAngleMax = 180;
    servoAngle = 10;
    int successCount = 0;
    int servoAngleIncrement = 2;
    scanDistance = 100;

    while (servoAngle < 170) {
      sensorServo.write(servoAngle);
      distance = measureDistance();
      servoAngle = servoAngle + servoAngleIncrement;
      delay(20);
      
      if (scanAngleMin == 0) { // proberen begin van object te scannen
        if (distance < scanDistance) {
          scanDistance = distance;
        }
        if (distance < distanceThreshold) {
          successCount++;
        } else {
          successCount = 0;  
        }
        if (successCount >= 8) { //als er acht keer na elkaar een korte afstand gemeten is
          scanAngleMin = servoAngle - (2 * servoAngleIncrement);
          successCount = 0;
        }
      } else if (scanAngleMax == 180) {
        if (distance > distanceThreshold) {
          successCount++;
        } else {
          successCount = 0;  
        }
        if (successCount >= 3) { //als er drie keer na elkaar een lange afstand gemeten is
          scanAngleMax = servoAngle - (3 * servoAngleIncrement);
          successCount = 0;
        }
      }
    }
    
    scanAngle = (scanAngleMin + scanAngleMax) / 2;
    Serial.print("Object found: ");
    Serial.println(scanAngle);
    
    sensorServo.write(90);  
}


/*
 *  movement
 */
void readWheelSpeed() {
  wheelSpeed = analogRead(pinSpeetPot);
  Serial.println("wheelspeed: ");
  Serial.println(wheelSpeed);
  wheelSpeed = map(wheelSpeed, 0, 1023, 50, 255);
}

void stopWheels() {
  analogWrite(pinSpeedA, 0);
  analogWrite(pinSpeedB, 0);
}

void moveWheels(uint8_t dir) {
  readWheelSpeed();
  digitalWrite(pinDirectionA, dir);
  digitalWrite(pinDirectionB, dir);
  analogWrite(pinSpeedA, wheelSpeed);
  analogWrite(pinSpeedB, wheelSpeed);
}

void turnWheels(uint8_t dir, uint32_t velocity, uint8_t moveDelay) {
  stopWheels();
  if (dir == 0) {
    Serial.println("turn left");
    digitalWrite(pinDirectionA, HIGH);
    digitalWrite(pinDirectionB, LOW);
  } else {
    Serial.println("turn right");
    digitalWrite(pinDirectionA, LOW);
    digitalWrite(pinDirectionB, HIGH);
  }
  analogWrite(pinSpeedA, velocity);
  analogWrite(pinSpeedB, velocity);
  delay(moveDelay);
  stopWheels();
}

void moveForward(uint8_t moveDelay) {
  stopWheels();
  moveWheels(HIGH);
  delay(moveDelay);
  stopWheels();
}

void moveBackward(uint8_t moveDelay) {
  stopWheels();
  moveWheels(LOW);
  delay(moveDelay);
  stopWheels();
}
