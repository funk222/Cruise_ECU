/*
****** PINOUTS ******

    BUTTONS
  MAIN_ON to pin 8
  SET (-) to pin 9
  RES (+) to pin 6
  CANCEL to pin 7

    CAN BOARD
  INT to pin 2
  SCK to pin 13
  MOSI to pin 11
  MISO to pin 12
  CS to pin 10

    VSS
  VSS to pin 3 via pullup resistor

    INTERRUPT
  INTERRUPT OUT to pin A4

  CAN tranciever code credit
  Copyright (c) Sandeep Mistry. All rights reserved.
  Licensed under the MIT license. See LICENSE file in the project root for full license information.
*/

#include <CAN.h>
//Variables for button presses
unsigned char pins[6] = {8, 9, 6, 7, 5, 4};
int buttonstate0 = 0;
int lastbuttonstate0 = 0;
int buttonstate1 = 0;
int lastbuttonstate1 = 0;
int buttonstate2 = 0;
int lastbuttonstate2 = 0;
int buttonstate3 = 0;
int lastbuttonstate3 = 0;
int gas_pedal_state;
int brake_pedal_state;

#define min_set_speed 7

//check_can
long check_can = 0;

//debouncing
long millis_held;
unsigned long firstTime;

//flags for cruise state
boolean flag1 = false;
boolean flag2 = false;

//VSS signal
int inc = 0;
int half_revolutions = 0;
int spd;
unsigned long lastmillis;
unsigned long duration;
uint8_t encoder = 0;

//are we using metric or standard?
boolean metric = true;

//CAN default messages
uint8_t set_speed = 0x0;
int addr[] = {0xfd, 0xfe, 0xff};
int data[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7};

//input smoothing
const int numReadings = 160;

float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender");

  //interrupt for VSS calculation

  attachInterrupt(3, rpm_fan, FALLING);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    // Serial.println("Starting CAN failed!");
    while (1);
  }

  //Setup Pins
  for (inc = 0; inc <= 4; inc++) {
    pinMode(pins[inc], INPUT);
  }
  pinMode(A4, OUTPUT);

  //initialize smoothing inputs
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  digitalWrite(pins[5], HIGH);

}

void loop() {
  //only run program while CAN is available
  attachInterrupt(1, rpm_fan, FALLING);
  digitalWrite(A4, HIGH);


  //speed calculation
  // speed = (1 / your pulses per distance) / (pulse duration in sec)
  // IE if your sensor is 4000 pulses/km then use 0.00025 (1/4000)

  if (half_revolutions >= 1) {
    detachInterrupt(1);
    duration = (micros() - lastmillis);
    spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
    lastmillis = micros(); // Uptade lasmillis
    half_revolutions = 0;
    attachInterrupt(1, rpm_fan, FALLING);
    if (!metric) {
      spd = spd * 1.60934;
    }
  }
  //Serial.println(spd);
  if ((micros() - lastmillis) > 2500000) {
    spd = 0;
  }

 

  //smoothing
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = spd;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  //Serial.println(average);
  delay(1);


  //Fingerprint msgs
  for (int ii = 0; ii < 3; ii++) {
    CAN.beginPacket(addr[ii]);
    for (int i = 0; i < 8; i++) {
      CAN.write(data[i]);
    }
    CAN.endPacket();
  }

  //Send data if button was pushed
  buttonstate0 = digitalRead(pins[0]);
  buttonstate1 = digitalRead(pins[1]);
  buttonstate2 = digitalRead(pins[2]);
  buttonstate3 = digitalRead(pins[3]);
  gas_pedal_state = 0;
  brake_pedal_state = 0;

  if (buttonstate0 != lastbuttonstate0) {
    if (buttonstate0 == LOW) {
      firstTime = millis();
      if (millis_held > 50) {
        flag1 = !flag1;
      }
    }
  }
  if (buttonstate3 != lastbuttonstate3) {
    if (buttonstate3 == LOW) {
      firstTime = millis();
      if (millis_held > 50) {
        flag2 = false;
        digitalWrite(A4, LOW);
      }
    }
  }
  if (flag1 == true) {
    if (!flag2) {
      if ((buttonstate1 == LOW) && (buttonstate1 != lastbuttonstate1)) {
        //SET button
        flag2 = true;
        //set the current speed as set speed, but only if it's over 5km/h
        if (round(average) < min_set_speed) {
          set_speed = min_set_speed;
        }
        else {
          set_speed = round(average);
        }
      }
      if (set_speed > 0) {
        if (buttonstate2 == LOW) {
          firstTime = millis();
          if (millis_held > 50) {
            flag2 = true;
          }
        }
      }
    }
    //else cruise is already engaged!
    else {
      if (buttonstate1 == LOW) {
        if (set_speed > min_set_speed) {
          firstTime = millis();
          if (millis_held > 50) {
            if (metric) {
              set_speed -= 1;
            }
            else {
              set_speed -= round(1 * 1.60934);
            }
          }
        }
        else if (set_speed == min_set_speed) {
          set_speed -= 0;
        }
      }
      if (buttonstate2 == LOW) {
        firstTime = millis();
        if (millis_held > 50) {
          if (metric) {
            set_speed += 1;
          }
          else {
            set_speed += round(1 * 1.60934);
          }
        }
      }
    }
  }

  millis_held = (millis() - firstTime);


  if (flag1 == false) {
    flag2 = false;
    set_speed = 0;
  }

  if (gas_pedal_state | brake_pedal_state) {
    flag2 = false;
  }

  //0x1d2 msg PCM_CRUISE
  uint8_t dat[8];
  dat[0] = (flag2 << 5) & 0x20 | (!gas_pedal_state << 4) & 0x10;
  dat[1] = 0x0;
  dat[2] = 0x0;
  dat[3] = 0x0;
  dat[4] = 0x0;
  dat[5] = 0x0;
  dat[6] = (flag2 << 7) & 0x80;
  dat[7] = can_cksum(dat, 7, 0x1d2);
  CAN.beginPacket(0x1d2);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat[ii]);
  }
  CAN.endPacket();

  //0x1d3 msg PCM_CRUISE_2
  uint8_t dat2[8];
  dat2[0] = 0x0;
  dat2[1] = (flag1 << 7) & 0x80 | 0x28;
  dat2[2] = set_speed;
  dat2[3] = 0x0;
  dat2[4] = 0x0;
  dat2[5] = 0x0;
  dat2[6] = 0x0;
  dat2[7] = can_cksum(dat2, 7, 0x1d3);
  CAN.beginPacket(0x1d3);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat2[ii]);
  }
  CAN.endPacket();

  //0xaa msg defaults 1a 6f WHEEL_SPEEDS
  uint8_t dat3[8];
  uint16_t wheelspeed = 0x1a6f + (average * 100);
  dat3[0] = (wheelspeed >> 8) & 0xFF;
  dat3[1] = (wheelspeed >> 0) & 0xFF;
  dat3[2] = (wheelspeed >> 8) & 0xFF;
  dat3[3] = (wheelspeed >> 0) & 0xFF;
  dat3[4] = (wheelspeed >> 8) & 0xFF;
  dat3[5] = (wheelspeed >> 0) & 0xFF;
  dat3[6] = (wheelspeed >> 8) & 0xFF;
  dat3[7] = (wheelspeed >> 0) & 0xFF;
  CAN.beginPacket(0xaa);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat3[ii]);
  }
  CAN.endPacket();

  //0x3b7 msg ESP_CONTROL
  uint8_t dat5[8];
  dat5[0] = 0x0;
  dat5[1] = 0x0;
  dat5[2] = 0x0;
  dat5[3] = 0x0;
  dat5[4] = 0x0;
  dat5[5] = 0x0;
  dat5[6] = 0x0;
  dat5[7] = 0x08;
  CAN.beginPacket(0x3b7);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat5[ii]);
  }
  CAN.endPacket();

  //0x620 msg STEATS_DOORS
  uint8_t dat6[8];
  dat6[0] = 0x10;
  dat6[1] = 0x0;
  dat6[2] = 0x0;
  dat6[3] = 0x1d;
  dat6[4] = 0xb0;
  dat6[5] = 0x40;
  dat6[6] = 0x0;
  dat6[7] = 0x0;
  CAN.beginPacket(0x620);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat6[ii]);
  }
  CAN.endPacket();

  // 0x3bc msg GEAR_PACKET
  uint8_t dat7[8];
  dat7[0] = 0x0;
  dat7[1] = 0x0;
  dat7[2] = 0x0;
  dat7[3] = 0x0;
  dat7[4] = 0x0;
  dat7[5] = 0x80;
  dat7[6] = 0x0;
  dat7[7] = 0x0;
  CAN.beginPacket(0x3bc);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat7[ii]);
  }
  CAN.endPacket();

  // 0x2c1 msg GAS_PEDAL
  uint8_t dat10[8];
  dat10[0] = (!gas_pedal_state << 3) & 0x08;
  dat10[1] = 0x0;
  dat10[2] = 0x0;
  dat10[3] = 0x0;
  dat10[4] = 0x0;
  dat10[5] = 0x0;
  dat10[6] = 0x0;
  dat10[7] = 0x0;
  CAN.beginPacket(0x2c1);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat10[ii]);
  }
  CAN.endPacket();

  //0x224 msg fake brake module
  uint8_t dat11[8];
  dat11[0] = 0x0;
  dat11[1] = 0x0;
  dat11[2] = 0x0;
  dat11[3] = 0x0;
  dat11[4] = 0x0;
  dat11[5] = 0x0;
  dat11[6] = 0x0;
  dat11[7] = 0x8;
  CAN.beginPacket(0x224);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat11[ii]);
  }
  CAN.endPacket();

  // 0xb4 speed and encoder for throttle ECU

uint16_t kmh = (average * 100);
CAN.beginPacket(0xb4);
CAN.print(kmh);
CAN.endPacket();



  lastbuttonstate0 = buttonstate0;
  lastbuttonstate1 = buttonstate1;
  lastbuttonstate2 = buttonstate2;
  lastbuttonstate3 = buttonstate3;


  //obey PCM Cancel Command
  CAN.parsePacket();
  if (CAN.packetId() == 0x343) {
    check_can = millis();
    uint8_t dat12[8];
    for (int ii = 0; ii <= 7; ii++) {
      dat12[ii]  = (char) CAN.read();
    }
    if ((dat12[3] & 0x01) == 1) {
      flag2 = false;
      Serial.println("CANCEL!");
    }
  }

  //Serial.println(millis() - check_can);
  Serial.println(flag1);
  Serial.println(flag2);
  Serial.println(set_speed);

  if ((millis() - check_can) > 10000) {
    flag1 = false;
    flag2 = false;
    set_speed = 0;
  }

}

void rpm_fan() {
  half_revolutions++;
  if (encoder > 255) {
    encoder = 0;
  }
  encoder++;
}

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
