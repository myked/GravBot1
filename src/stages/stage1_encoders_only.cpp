/*
  STAGE 1 -- Encoders only. No Servo/Sabertooth code at all, no motor
  pins touched. This is the safest possible starting point: it can only
  tell you about the encoders, nothing else.

  GOAL
  ----
  With the robot powered but sitting still (wheels not touched), every
  count below should read 0, forever. If any of them drift away from 0
  on their own, that channel has a wiring/noise problem independent of
  anything to do with the motors or Sabertooth - fix that before moving
  on to Stage 2.

  Once resting is clean, gently spin each wheel by hand (one at a time)
  and confirm:
    - only that wheel's counter moves
    - it counts in a consistent direction for a consistent spin direction
    - front/rear on the same side roughly agree in magnitude

  WIRING (unchanged from the full sketch)
  ------
  Left  FRONT A -> pin 2   (INT0, hardware interrupt)
  Left  FRONT B -> pin 4   (plain digital input)
  Right FRONT A -> pin 3   (INT1, hardware interrupt)
  Right FRONT B -> pin 5   (plain digital input)
  Left  REAR  A -> pin 11  (PCINT, pin-change interrupt group)
  Left  REAR  B -> pin 6   (plain digital input)
  Right REAR  A -> pin 12  (PCINT, pin-change interrupt group)
  Right REAR  B -> pin 7   (plain digital input)
*/

#include <Arduino.h>

const int LF_A = 2,  LF_B = 4;   // left front  (hardware interrupt)
const int RF_A = 3,  RF_B = 5;   // right front (hardware interrupt)
const int LR_A = 11, LR_B = 6;   // left rear   (pin-change interrupt)
const int RR_A = 12, RR_B = 7;   // right rear  (pin-change interrupt)

const unsigned long PRINT_PERIOD_MS = 200; // slower, easier to read by eye

volatile long lfTicks = 0, rfTicks = 0, lrTicks = 0, rrTicks = 0; // signed

void lfISR() { lfTicks += (digitalRead(LF_B) == HIGH) ? 1 : -1; }
void rfISR() { rfTicks += (digitalRead(RF_B) == HIGH) ? 1 : -1; }

// Pin-change interrupt handles both rear encoders (pins 11 = PB3, 12 = PB4)
volatile uint8_t lastPINB = 0;
ISR(PCINT0_vect) {
  uint8_t current = PINB;
  uint8_t changed = current ^ lastPINB;

  if ((changed & (1 << PB3)) && (current & (1 << PB3))) { // pin 11 rose
    lrTicks += (digitalRead(LR_B) == HIGH) ? 1 : -1;
  }
  if ((changed & (1 << PB4)) && (current & (1 << PB4))) { // pin 12 rose
    rrTicks += (digitalRead(RR_B) == HIGH) ? 1 : -1;
  }
  lastPINB = current;
}

unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(LF_A, INPUT_PULLUP); pinMode(LF_B, INPUT_PULLUP);
  pinMode(RF_A, INPUT_PULLUP); pinMode(RF_B, INPUT_PULLUP);
  pinMode(LR_A, INPUT_PULLUP); pinMode(LR_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LF_A), lfISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RF_A), rfISR, RISING);

  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT3) | (1 << PCINT4);
  lastPINB = PINB;

  Serial.println("STAGE 1: encoders only, no motor code. Leave wheels still.");
  Serial.println("All counts below should stay at 0 until you spin a wheel by hand.");
  lastPrintTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;

    noInterrupts();
    long lf = lfTicks, rf = rfTicks, lr = lrTicks, rr = rrTicks;
    interrupts();

    Serial.println("LFcum:" + String(lf) +
                    " RFcum:" + String(rf) +
                    " LRcum:" + String(lr) +
                    " RRcum:" + String(rr));
  }
}
