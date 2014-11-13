void moveWheels(uint8_t dir = HIGH);

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

bool startScan;
uint32_t scanAngleMin = 0;
uint32_t scanAngleMax = 180;

#define pinDistanceTrigger 9
#define pinDistanceEcho 8
#define distanceThreshold 30

long distance;

// startbutton
uint32_t startButtonTimePast = 0;


// actions
byte action;
#define ACTION_STANDBY 0
#define ACTION_SEARCH_TARGET 1
#define ACTION_MOVE_TO_TARGET 3
#define ACTION_STOPPING 4


void setup() {
  pinMode(pinDirectionA, OUTPUT); 
  pinMode(pinSpeedA, OUTPUT); 
  pinMode(pinDirectionB, OUTPUT);
  pinMode(pinSpeedB, OUTPUT);
  
  pinMode(pinSpeetPot, INPUT);
  
  pinMode(pinDistanceTrigger, OUTPUT);
  pinMode(pinDistanceEcho, INPUT);
  
  pinMode(4,INPUT);
  
  sensorServo.attach(pinSensorServo);
  sensorServo.write(servoAngle);
  
  Serial.begin(9600);
  delay(1000);
     
  action = ACTION_SEARCH_TARGET;
}
 
void loop() {
  readStartButton();
  
  
  switch (action) {
    case ACTION_STANDBY:
      break;
    
    case ACTION_SEARCH_TARGET:
      //scan();
      startScan = true;
      newscan();
      action = ACTION_STANDBY;
      break;
      
    case ACTION_MOVE_TO_TARGET:
    
    //  moveWheels();
    
      break;
      
    case ACTION_STOPPING:
      stopWheels();
      action = ACTION_STANDBY;
      break;
  }

}

void readStartButton() {
  if (millis() > (startButtonTimePast + 200)) {
    startButtonTimePast = millis();
    bool buttonPressed = 0;
    while (digitalRead(4)) {
      buttonPressed = true;
    }
    if (buttonPressed) {
      if (action == ACTION_STANDBY) {
        action = ACTION_SEARCH_TARGET;
      } else {
        action = ACTION_STOPPING;
      }
    }
  }
}

void moveWheels(uint8_t dir) {
  stopWheels();

  wheelSpeed = analogRead(pinSpeetPot);
  Serial.println(wheelSpeed);
  wheelSpeed = map(wheelSpeed, 0, 1014, 50, 180);
  Serial.println(wheelSpeed);

  digitalWrite(pinDirectionA, dir);
  digitalWrite(pinDirectionB, dir);
  analogWrite(pinSpeedA, wheelSpeed);
  analogWrite(pinSpeedB, wheelSpeed);
}

void stopWheels() {
  analogWrite(pinSpeedA, 0);
  analogWrite(pinSpeedB, 0);
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

void newscan()
{
  if (startScan) {
    startScan = false;
    scanAngleMin = 0;
    scanAngleMax = 180;
    servoAngle = 30;
    int successCount = 0;
    int servoAngleIncrement = 2;

    while (servoAngle < 170) {
      sensorServo.write(servoAngle);
      distance = measureDistance();
      servoAngle = servoAngle + servoAngleIncrement;
      Serial.print(servoAngle);
      Serial.print(": ");
      Serial.println(distance);
      delay(20);
      
      if (scanAngleMin == 0) { // proberen begin van object te scannen
        if (distance < distanceThreshold) {
          successCount++;
        } else {
          successCount = 0;  
        }
        if (successCount >= 3) { //als er drie keer na elkaar een korte afstand gemeten is
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
    
    Serial.print("Object: ");
    Serial.print(scanAngleMin);
    Serial.print(" - ");
    Serial.print(scanAngleMax);
    
    sensorServo.write(90);
  }
}
