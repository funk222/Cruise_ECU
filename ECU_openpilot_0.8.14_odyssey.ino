// =========================================================================
// Option1 容错版：在 ECU_fixed 基础上补齐可能缺失的 Corolla 车身类报文
// 目标：匹配 openpilot 0.8.13 + 0813NoLFS（Toyota single-CAN 适配）
//
// 关键补齐：
// - 0x620 BODY_CONTROL_STATE
// - 0x614 BLINKERS_STATE
// - 0x622 LIGHT_STALK
// - 0x2C1 GAS_PEDAL（原版已发送，这里保留）
//
// 注意：
// 1) 如果你的车上真实 BCM/组合开关已经在发这些报文，请把 EMIT_BCM_FALLBACK 设为 false，
//    避免同 ID 多节点冲突。
// 2) 这个版本优先“先启动起来”，默认把门/座椅/转向灯/灯光状态都置为安全中性值。
// =========================================================================

#include <Arduino.h>
#include <CAN.h>

const byte CAN_CS_PIN = 10;
const byte CAN_INT_PIN = 2;

#define TIRE_CIRCUMFERENCE 1.993
#define PULSES_PER_REV 50
const float VSS_SCALE = (TIRE_CIRCUMFERENCE * 36.0f) / (float)PULSES_PER_REV;

const byte pinVssInterrupt = 3;
const int pinButtonMain = 8;
const int pinButtonPlus = 7;
const int pinButtonMinus = 6;
const int pinButtonCancel = 9;

const int pinBrakeInput = A0;
const int pinGasInput = A1;

unsigned long last_20ms_tick = 0;
unsigned long last_12ms_tick = 0;
unsigned long last_50ms_tick = 0;
unsigned long last_100ms_tick = 0;
unsigned long last_vss_tick = 0;
unsigned long last_btn_tick = 0;

const unsigned long BTN_DEBOUNCE_DELAY = 50;
const uint8_t MIN_SET_SPEED_KPH = 10;
const uint8_t MAX_SET_SPEED_KPH = 180;
// 速度显示微调系数：
// - 默认 1.00：按几何参数原始计算
// - 显示偏高：调小（如 0.96）
// - 显示偏低：调大（如 1.04）
const float TOYOTA_WHEEL_SPEED_SCALE = 0.33f;

// 如果真实 BCM 没有这些报文，保持 true；如果真实 BCM 已经发送，请改成 false
const bool EMIT_BCM_FALLBACK = true;

unsigned long can_tx_fail_count = 0;

boolean OP_ON = false;
boolean MAIN_ON = true;
uint8_t set_speed = 0;
float current_speed = 0.0;
uint8_t cruise_state = 7;  // 0=off, 7=standby, 8=active

boolean btn_main_last = HIGH;
boolean btn_plus_last = HIGH;
boolean btn_minus_last = HIGH;
boolean btn_cancel_last = HIGH;

boolean BRAKE_PRESSED = false;
boolean GAS_RELEASED = true;

volatile unsigned long pulse_count = 0;
void vss_pulse_counter() { pulse_count++; }

uint8_t clamp_set_speed(float speed_kph) {
  int value = (int)speed_kph;
  if (value < MIN_SET_SPEED_KPH) return MIN_SET_SPEED_KPH;
  if (value > MAX_SET_SPEED_KPH) return MAX_SET_SPEED_KPH;
  return (uint8_t)value;
}

void can_send_8(uint16_t addr, const uint8_t *dat) {
  CAN.beginPacket(addr);
  CAN.write(dat, 8);
  if (!CAN.endPacket()) can_tx_fail_count++;
}

uint8_t toyota_checksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint16_t checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int i = 0; i < len; i++) checksum += dat[i];
  return (uint8_t)(checksum & 0xFF);
}

void setup() {
  CAN.setPins(CAN_CS_PIN, CAN_INT_PIN);
  if (!CAN.begin(500E3)) {
    while (1) { }
  }

  pinMode(pinVssInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinVssInterrupt), vss_pulse_counter, FALLING);

  pinMode(pinButtonMain, INPUT_PULLUP);
  pinMode(pinButtonPlus, INPUT_PULLUP);
  pinMode(pinButtonMinus, INPUT_PULLUP);
  pinMode(pinButtonCancel, INPUT_PULLUP);
  pinMode(pinBrakeInput, INPUT_PULLUP);
  pinMode(pinGasInput, INPUT_PULLUP);
}

void loop() {
  unsigned long now = millis();

  if (now - last_vss_tick >= 100) {
    last_vss_tick = now;
    noInterrupts();
    unsigned long pulses = pulse_count;
    pulse_count = 0;
    interrupts();
    current_speed = (float)pulses * VSS_SCALE;
  }

  BRAKE_PRESSED = (digitalRead(pinBrakeInput) == LOW);
  GAS_RELEASED = (digitalRead(pinGasInput) == LOW);
  if (BRAKE_PRESSED || !GAS_RELEASED) OP_ON = false;

  if (now - last_btn_tick >= BTN_DEBOUNCE_DELAY) {
    last_btn_tick = now;
    boolean btn_main = digitalRead(pinButtonMain);
    boolean btn_plus = digitalRead(pinButtonPlus);
    boolean btn_minus = digitalRead(pinButtonMinus);
    boolean btn_cancel = digitalRead(pinButtonCancel);

    if (btn_main == LOW && btn_main_last == HIGH) {
      MAIN_ON = !MAIN_ON;
      if (!MAIN_ON) OP_ON = false;
    }

    if (MAIN_ON && btn_minus == LOW && btn_minus_last == HIGH) {
      if (!OP_ON && current_speed > 5.0f) {
        OP_ON = true;
        set_speed = clamp_set_speed(current_speed + 3.0f);
      } else if (OP_ON && set_speed > 10) {
        set_speed -= 5;
        if (set_speed < MIN_SET_SPEED_KPH) set_speed = MIN_SET_SPEED_KPH;
      }
    }

    if (OP_ON && btn_plus == LOW && btn_plus_last == HIGH) {
      if (set_speed < MAX_SET_SPEED_KPH) {
        set_speed += 5;
        if (set_speed > MAX_SET_SPEED_KPH) set_speed = MAX_SET_SPEED_KPH;
      }
    }

    if (btn_cancel == LOW && btn_cancel_last == HIGH) OP_ON = false;

    btn_main_last = btn_main;
    btn_plus_last = btn_plus;
    btn_minus_last = btn_minus;
    btn_cancel_last = btn_cancel;
  }

  if (!MAIN_ON) cruise_state = 0;
  else if (OP_ON) cruise_state = 8;
  else cruise_state = 7;

  // 50Hz: PCM_CRUISE / PCM_CRUISE_2 / GAS_PEDAL / BRAKE_MODULE
  if (now - last_20ms_tick >= 20) {
    last_20ms_tick = now;

    uint8_t dat_1d2[8] = {0};
    dat_1d2[0] = (OP_ON ? 0x20 : 0x00) | (GAS_RELEASED ? 0x10 : 0x00);
    dat_1d2[6] = (cruise_state & 0x0F) << 4;  // CRUISE_STATE nibble
    dat_1d2[7] = toyota_checksum(dat_1d2, 7, 0x1D2);
    can_send_8(0x1D2, dat_1d2);

    uint8_t dat_1d3[8] = {0};
    dat_1d3[1] = (MAIN_ON ? 0x80 : 0x00);  // MAIN_ON
    dat_1d3[2] = set_speed;                // SET_SPEED
    dat_1d3[7] = toyota_checksum(dat_1d3, 7, 0x1D3);
    can_send_8(0x1D3, dat_1d3);

    uint8_t dat_2c1[8] = {0};
    dat_2c1[0] = (GAS_RELEASED ? 0x08 : 0x00);  // GAS_RELEASED bit
    can_send_8(0x2C1, dat_2c1);

    uint8_t dat_224[8] = {0};
    dat_224[0] = (BRAKE_PRESSED ? 0x20 : 0x00); // BRAKE_PRESSED bit
    can_send_8(0x224, dat_224);
  }

  // 83Hz: WHEEL_SPEEDS
  if (now - last_12ms_tick >= 12) {
    last_12ms_tick = now;

    float wheel_spd_kph = current_speed * TOYOTA_WHEEL_SPEED_SCALE;
    if (wheel_spd_kph < 0.0f) wheel_spd_kph = 0.0f;
    if (wheel_spd_kph > 655.35f) wheel_spd_kph = 655.35f;
    uint16_t raw = (uint16_t)(wheel_spd_kph * 100.0f);

    uint8_t dat_aa[8] = {0};
    dat_aa[0] = (raw >> 8) & 0xFF;
    dat_aa[1] = raw & 0xFF;
    dat_aa[2] = (raw >> 8) & 0xFF;
    dat_aa[3] = raw & 0xFF;
    dat_aa[4] = (raw >> 8) & 0xFF;
    dat_aa[5] = raw & 0xFF;
    dat_aa[6] = (raw >> 8) & 0xFF;
    dat_aa[7] = raw & 0xFF;
    can_send_8(0xAA, dat_aa);
  }

  // 20Hz: radar keepalive
  if (now - last_50ms_tick >= 50) {
    last_50ms_tick = now;
    uint8_t dat_430[8] = {0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00};
    dat_430[7] = toyota_checksum(dat_430, 7, 0x430);
    can_send_8(0x430, dat_430);
  }

  // 10Hz: low-rate + fallback body messages
  if (now - last_100ms_tick >= 100) {
    last_100ms_tick = now;

    uint8_t dat_3b7[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
    can_send_8(0x3B7, dat_3b7);

    if (EMIT_BCM_FALLBACK) {
      // 0x620 BODY_CONTROL_STATE: all doors closed, seatbelt latched, parking brake off
      uint8_t dat_620[8] = {0};
      can_send_8(0x620, dat_620);

      // 0x614 BLINKERS_STATE: TURN_SIGNALS=3 (none), hazard=0
      uint8_t dat_614[8] = {0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00};
      can_send_8(0x614, dat_614);

      // 0x622 LIGHT_STALK: AUTO_HIGH_BEAM=0
      uint8_t dat_622[8] = {0};
      can_send_8(0x622, dat_622);
    }

    uint8_t dat_3bc[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00};
    can_send_8(0x3BC, dat_3bc);
  }

  if (CAN.parsePacket()) {
    while (CAN.available()) CAN.read();
  }
}
