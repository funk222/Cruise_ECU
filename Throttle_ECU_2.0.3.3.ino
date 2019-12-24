// 2.0.3.3

#include <CAN.h>
#define POT_VALUE_MAX 900
#define POT_VALUE_MIN 85
#define PERM_ERROR 1
#define MAX_ANGLE 3000


boolean debug = false;
boolean controls_allowed = false;

unsigned char pins[5] = {4, 5, 6, 7};
int potPin = A0;
int solenoid_pin = 6;
int M_Speed = 255;
int M_pin1 = 4;
int M_pin2 = 5;
int M_PWMPin = 9;
int currentAngle = 0;

int gas = 0;
int angle = 0;


unsigned long lastmillis = 0;
unsigned long lagtime = 0;

void setup() {
  
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), cancel, LOW);
  pinMode(potPin, INPUT);
  for (int ii = 0; ii <= 5; ii++) {
    pinMode(pins[ii], OUTPUT);
  }
  if (!CAN.begin(500E3)) {
    while (1) {
      Serial.println("Starting CAN failed!");
    }
  }
}

void loop() {

  int target = get_angle();
 // Serial.println(target);
 
  if (controls_allowed) {
    digitalWrite(solenoid_pin, HIGH);
  }
  if (!controls_allowed) {
    digitalWrite(solenoid_pin, LOW);
  }

  if ((millis() - lagtime) >= 50) {
    //read potpin every 50 ms
    currentAngle = ((float)analogRead(potPin) - POT_VALUE_MIN) / (POT_VALUE_MAX - POT_VALUE_MIN) * MAX_ANGLE;
    lagtime = millis();
  }
  if (controls_allowed && (abs(currentAngle - target) > PERM_ERROR)) //if the shaft is not at the required value,
  {
    analogWrite(M_PWMPin, M_Speed);
    if (currentAngle < target) {
      digitalWrite(M_pin1, HIGH);
      digitalWrite(M_pin2, LOW);
    }
    else if (currentAngle > target) {
      digitalWrite(M_pin1, LOW);
      digitalWrite(M_pin2, HIGH);
    }
  }
  else {
    analogWrite(M_PWMPin, 0);
  }
}

int get_angle() {

  CAN.parsePacket();

  switch (CAN.packetId())
  {
    case 0x343:
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
      }      
        gas = (dat[0] << 8 | dat[1] << 0);
        
//clip if gas exceeds limits

        if ((gas >= 0) && (gas < 3000)) {
        controls_allowed = true;
        angle = gas;
        }
      
        else {
        controls_allowed = false;
        angle = 0;
 Serial.println("BRAKING");
        }
        
  break;      
  }
  
  return angle;

}

void cancel() {
  controls_allowed = false;
  gas = 0;
  angle = 0;
  digitalWrite(solenoid_pin, LOW);
  Serial.println("INTERUPPED");
}
