// MAIN ECU

#include <Arduino.h>

#include <CAN.h>


//______________BUTTONS AND SWITCHES
https://github.com/funk222/Cruise_ECU/blob/master/main.cpp
int BlinkerPinLeft = 4;
int BlinkerPinRight = 5;
int button4 = 8; // Cruise Main
int button3 = 7; // +
int button2 = 6; // -
int button1 = 9; // Set
//int CluchSwitch = A4;
//boolean ClutchSwitchState = false; //消滅這個
int buttonstate4;
int lastbuttonstate4;
int buttonstate3;
int lastbuttonstate3;
int buttonstate2;
int lastbuttonstate2;
int buttonstate1;
int lastbuttonstate1;
boolean lastGAS_RELEASED = true; //消滅這個
boolean lastBRAKE_PRESSED = false; //消滅這個

//VSS signal
unsigned char VssPin = A4; //voltage reading from A4
int Voltage; //analog.read the voltage
float spd;
float a = 0.00008; //Curve fitting of spd = a * Voltage * Voltage + b * Voltage + c;
float b = 0.06; //Curve fitting of spd = a * Voltage * Voltage + b * Voltage + c;


//______________VALUES SEND ON CAN
boolean OP_ON = false;
boolean MAIN_ON = true;
uint8_t set_speed = 0x0;
double average = 0; 
boolean blinker_left = true;
boolean blinker_right = true;
float LEAD_LONG_DIST = 0;
float LEAD_REL_SPEED = 0;
float LEAD_LONG_DIST_RAW = 0;
float LEAD_REL_SPEED_RAW = 0;
boolean BRAKE_PRESSED = false;
boolean GAS_RELEASED = true;

  //speed calculation

  Voltage = analogRead (VssPin);
  //Serial.println(Voltage);
  spd = 1.609 *(a*Voltage*Voltage+b*Voltage);
  //Serial.println(spd);
  average = spd;



/*
//______________FOR SMOOTHING SPD
const int numReadings = 160;
float readings[numReadings];
int readIndex = 0; 
double total = 0;
*/

/*
//______________FOR READING VSS SENSOR
const byte interruptPin = 3;
int inc = 0;
int half_revolutions = 0;
int spd;
unsigned long lastmillis;
unsigned long duration;
uint8_t encoder = 0;
*/
/*
void rpm() {
  half_revolutions++;
  if (encoder > 255)
  {
    encoder = 0;
  }
  encoder++;
}
*/

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

void setup() {
  
//Serial.begin(9600);
CAN.begin(500E3);

//pinMode(interruptPin, INPUT_PULLUP); //VSS速度讀取
//attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);
pinMode(button1, INPUT);//
pinMode(button2, INPUT);
pinMode(button3, INPUT);
pinMode(button4, INPUT);
pinMode(BlinkerPinLeft, INPUT_PULLUP); //左轉向燈
pinMode(BlinkerPinRight, INPUT_PULLUP); //右轉向燈


/*
//______________initialize smoothing inputs
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
*/  
}

void loop() {
/*  
//______________READ SPD SENSOR
attachInterrupt(1,rpm, FALLING);

if (half_revolutions >= 1) {
    detachInterrupt(1);
    duration = (micros() - lastmillis);
    spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
    lastmillis = micros(); 
    half_revolutions = 0;
    attachInterrupt(1, rpm, FALLING);
  }

//______________SMOOTH SPD TO AVERAGE
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
  average = total / numReadings*3/40;
  // send it to the computer as ASCII digits
  
 */
//______________READING BUTTONS AND SWITCHES // Clutch 讓它一直關閉
//ClutchSwitchState = false;//digitalRead(CluchSwitch);
buttonstate4 = digitalRead(button4);
buttonstate3 = digitalRead(button3);
buttonstate2 = digitalRead(button2);
buttonstate1 = digitalRead(button1);

blinker_left = false; // digitalRead(BlinkerPinLeft);
blinker_right = false; // digitalRead(BlinkerPinRight);

/*
Serial.print("average:");
Serial.print(average);
Serial.print("     ");
Serial.print("OP_ON:");
Serial.print(OP_ON);
Serial.print("     ");
Serial.print("buttonstate4:");
Serial.print(buttonstate4);
Serial.println("     ");
*/

//______________SET OP OFF WHEN BRAKE IS PRESSED

       if (BRAKE_PRESSED == true)
       {
       OP_ON = false;
       }
 
    
//______________SET OP OFF WHEN GAS IS PRESSED
       if (GAS_RELEASED == false)
       {
       OP_ON = false;
       }
//______________SET BUTTON NR4
if (buttonstate4 != lastbuttonstate4)
    {
       if (buttonstate4 == LOW)
       {
          if (OP_ON == true)
          {
          OP_ON = false;
          }
          else
          {
          OP_ON = true;
          set_speed = (average + 3);
          }
        }
     }
     
//______________SET BUTTON NR3
if (buttonstate3 != lastbuttonstate3)
    {
       if (buttonstate3 == LOW)
       {
       set_speed = set_speed + 5;
       }
    }
//______________SET BUTTON NR2
if (buttonstate2 != lastbuttonstate2)
   {
       if (buttonstate2 == LOW)
       {
       set_speed = set_speed - 5;
       }
    }

    
//______________LIMIT FOR SETSPEED
if (set_speed > 200)
    { 
      set_speed = 0;
    }
    

//______________SET BUTTON NR1
if (buttonstate1 != lastbuttonstate1)
   {
       if (buttonstate1 == LOW)
       {
       OP_ON = false;
       }
   }


//______________SET CLUTCH SWITCH

 /*
  if (ClutchSwitchState == LOW)
   {
  //  ("Clutch Pedal is pressed");
   }
 */

lastbuttonstate1 = buttonstate1;
lastbuttonstate2 = buttonstate2;
lastbuttonstate3 = buttonstate3;
lastbuttonstate4 = buttonstate4;
//lastBRAKE_PRESSED = BRAKE_PRESSED;
//lastGAS_RELEASED = GAS_RELEASED;


//______________SENDING_CAN_MESSAGES

  //0x1d2 msg PCM_CRUISE
  uint8_t dat_1d2[8];
  dat_1d2[0] = (OP_ON << 5) & 0x20 | (GAS_RELEASED << 4) & 0x10;
  dat_1d2[1] = 0x0;
  dat_1d2[2] = 0x0;
  dat_1d2[3] = 0x0;
  dat_1d2[4] = 0x0;
  dat_1d2[5] = 0x0;
  dat_1d2[6] = (OP_ON << 7) & 0x80;
  dat_1d2[7] = can_cksum(dat_1d2, 7, 0x1d2);
  CAN.beginPacket(0x1d2);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d2[ii]);
  }
  CAN.endPacket();

  //0x1d3 msg PCM_CRUISE_2
  uint8_t dat_1d3[8];
  dat_1d3[0] = 0x0;
  dat_1d3[1] = (MAIN_ON << 7) & 0x80 | 0x28;
  dat_1d3[2] = set_speed;
  dat_1d3[3] = 0x0;
  dat_1d3[4] = 0x0;
  dat_1d3[5] = 0x0;
  dat_1d3[6] = 0x0;
  dat_1d3[7] = can_cksum(dat_1d3, 7, 0x1d3);
  CAN.beginPacket(0x1d3);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d3[ii]);
  }
  CAN.endPacket();

  //0xaa msg defaults 1a 6f WHEEL_SPEEDS
  uint8_t dat_aa[8];
  uint16_t wheelspeed = 0x1a6f + (average * 100);
  dat_aa[0] = (wheelspeed >> 8) & 0xFF;
  dat_aa[1] = (wheelspeed >> 0) & 0xFF;
  dat_aa[2] = (wheelspeed >> 8) & 0xFF;
  dat_aa[3] = (wheelspeed >> 0) & 0xFF;
  dat_aa[4] = (wheelspeed >> 8) & 0xFF;
  dat_aa[5] = (wheelspeed >> 0) & 0xFF;
  dat_aa[6] = (wheelspeed >> 8) & 0xFF;
  dat_aa[7] = (wheelspeed >> 0) & 0xFF;
  CAN.beginPacket(0xaa);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_aa[ii]);
  }
  CAN.endPacket();

  //0x3b7 msg ESP_CONTROL
  uint8_t dat_3b7[8];
  dat_3b7[0] = 0x0;
  dat_3b7[1] = 0x0;
  dat_3b7[2] = 0x0;
  dat_3b7[3] = 0x0;
  dat_3b7[4] = 0x0;
  dat_3b7[5] = 0x0;
  dat_3b7[6] = 0x0;
  dat_3b7[7] = 0x08;
  CAN.beginPacket(0x3b7);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3b7[ii]);
  }
  CAN.endPacket();

  //0x620 msg STEATS_DOORS
  uint8_t dat_620[8];
  dat_620[0] = 0x10;
  dat_620[1] = 0x0;
  dat_620[2] = 0x0;
  dat_620[3] = 0x1d;
  dat_620[4] = 0xb0;
  dat_620[5] = 0x40;
  dat_620[6] = 0x0;
  dat_620[7] = 0x0;
  CAN.beginPacket(0x620);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_620[ii]);
  }
  CAN.endPacket();

  // 0x3bc msg GEAR_PACKET
  uint8_t dat_3bc[8];
  dat_3bc[0] = 0x0;
  dat_3bc[1] = 0x0;
  dat_3bc[2] = 0x0;
  dat_3bc[3] = 0x0;
  dat_3bc[4] = 0x0;
  dat_3bc[5] = 0x80;
  dat_3bc[6] = 0x0;
  dat_3bc[7] = 0x0;
  CAN.beginPacket(0x3bc);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3bc[ii]);
  }
  CAN.endPacket();
/*
  //0x614 msg steering_levers 轉向相關的
  uint8_t dat_614[8];
  dat_614[0] = 0x29;
  dat_614[1] = 0x0;
  dat_614[2] = 0x01;
  dat_614[3] = (blinker_left << 5) & 0x20 |(blinker_right << 4) & 0x10;
  dat_614[4] = 0x0;
  dat_614[5] = 0x0;
  dat_614[6] = 0x76;
  dat_614[7] = can_cksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_614[ii]);
  }
  CAN.endPacket();
*/


//______________READING CAN
  CAN.parsePacket();

  //128x2e6 msg LEAD_INFO
  if (CAN.packetId() == 0x2e6)
      {
      uint8_t dat_2e6[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_2e6[ii]  = (char) CAN.read();
        }
        LEAD_LONG_DIST_RAW = (dat_2e6[0] << 8 | dat_2e6[1] << 3); 
        LEAD_REL_SPEED_RAW = (dat_2e6[2] << 8 | dat_2e6[3] << 4);
        }
  //______________CONVERTING INTO RIGHT VALUE USING DBC SCALE
  LEAD_LONG_DIST = (LEAD_LONG_DIST_RAW * 0.005);
  LEAD_REL_SPEED = (LEAD_REL_SPEED_RAW * 0.009);

  /*
  //0x224 msg BRAKE_MODULE --- WE are using the 0x3b7 message, which is ESP_CONTROL to reduce traffic on the can network
    if (CAN.packetId() == 0x3b7)
      {
      uint8_t dat_3b7[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_3b7[ii]  = (char) CAN.read();
        }
        BRAKE_PRESSED = (dat_3b7[0] << 5);
        }

  */

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

/*
    //0x2c1 msg GAS_PEDAL
    if (CAN.packetId() == 0x2c1)
      {
      uint8_t dat_2c1[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_2c1[ii]  = (char) CAN.read();
        }
        GAS_RELEASED = (dat_2c1[0] << 3);
        }
*/

  // 0x2c1 msg GAS_PEDAL
  uint8_t dat10[8];
  dat10[0] = (0 << 3) & 0x08;
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
  
//______________LOGIC FOR LANE CHANGE RECOMENDITION
if (set_speed >= ((average * 100) + 15))
   {
   if (LEAD_REL_SPEED  <= 15)
      {
      if (LEAD_LONG_DIST <= 100)
         {    
         blinker_left = false;
         }
      }
   }

}



