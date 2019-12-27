// 2.0.3.2.1

#include <CAN.h>
#define POT_VALUE_MAX 900
#define POT_VALUE_MIN 92
#define PERM_ERROR 1
#define MAX_ANGLE 1500

boolean controls_allowed = false;

//PINS 
const byte interruptPin = 3;
int potPin = A0;
int solenoid_pin = 6;
int M_pin1 = 4;
int M_pin2 = 5;
int M_PWMPin = 9;

//
int M_Speed = 255;
int currentAngle = 0;
int gas = 0;
int angle = 0;

//
unsigned long lastmillis = 0;
unsigned long lagtime = 0;

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), cancel, LOW);
  Serial.begin(9600);
  
    pinMode(potPin, INPUT);
    pinMode(2, OUTPUT);
    pinMode(M_pin1, OUTPUT);
    pinMode(M_pin2, OUTPUT);
    pinMode(solenoid_pin, OUTPUT);
    pinMode(M_PWMPin, OUTPUT);


    
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

  if ((millis() - lagtime) >= 5) {
    //read potpin every 100 ms
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
    case 0x1d2:
      uint8_t dat8[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat8[ii]  = (char) CAN.read();
      }
      //CHECK CHECKSUM FIRST
      if (dat8[7] == can_cksum(dat8, 7, 0x1d2)) {
        //check acc active bit
        if ((dat8[0] >> 5) & 0x01) {
          //enable controls
          controls_allowed = true;
//         Serial.println("controls_allowed!");
        }
      }
      else {
        //can checksum is bad
        Serial.println("ERROR ln 103 BAD CHECKSUM 0x1d2");
        controls_allowed = false;
      }
      break;

    case 0x343:
      lastmillis = millis();
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
      }
      
        //respect PCM_CANCEL
        if ((dat[3] << 7 & 0x80) == 1) {
          controls_allowed = false;
          Serial.println("Line 118 CANCEL!");
        }
        //bitwise math to get target accel value in m/s^2
        gas = (dat[0] << 8 | dat[1] << 0);
        
        //clip if gas exceeds limits
        if ((gas >= 0) && (gas < 3000)) {
          angle = gas;
     Serial.println(gas);
        }
      
      else {
   Serial.println("BRAKE ln 131");
//     controls_allowed = false;
     angle = 0;
      }
      break;      
         }
  


  
// SAFETY CHECK - did we recieve 343 in a reasonable amount of time?
 if ((millis() - lastmillis) > 10000) {
      Serial.println("ERROR Line 143 BAD CHECKSUM 0x343");
      controls_allowed = false;
      gas = 0;
      angle = 0;
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

//TODO: use CRC-8
//TOYOTA CAN CHECKSUM
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}
