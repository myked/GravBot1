/*
  STAGE 2 -- Open-loop drive, both sides, no PID.

  Builds on Stage 1, which confirmed:
    - all 4 encoders read clean 0 at rest
    - each channel isolated correctly to its own wheel when hand-spun
    - LEFT side (LF/LR) reads POSITIVE for forward rotation, RIGHT side
      (RF/RR) reads NEGATIVE for forward rotation - expected, since the
      right side motors are mirror-mounted relative to the left side.

  This stage adds direct, un-controlled PWM output to both Sabertooth
  channels, WITH the right-side sign correction applied (see
  RIGHT_SIGN below), so "forward" reads positive consistently on both
  sides - matching what the PID in the full sketch will need.

  Starts fully STOPPED on boot - nothing moves until you send a command.

  SERIAL COMMANDS
  ----------------
  L<value>   set left  PWM directly, e.g. "L1600"
  R<value>   set right PWM directly, e.g. "R1600"
  S          stop both sides immediately
  Values are in microseconds, 1000-2000, 1500 = stop. Clamped to range.

  SUGGESTED TEST ORDER
  ---------------------
  1. Send "L1600" only. Confirm: only left wheels turn, LFmms/LRmms are
     POSITIVE, RFmms/RRmms stay near 0.
  2. Send "S" to stop.
  3. Send "R1600" only. Confirm: only right wheels turn, RFmms/RRmms
     are now also POSITIVE (thanks to the sign correction), LFmms/LRmms
     stay near 0.
  4. Send "S" to stop.

  WIRING (unchanged, pin/side mapping confirmed by bench test)
  ------
  Left  FRONT A -> pin 2   (INT0, hardware interrupt)
  Left  FRONT B -> pin 4   (plain digital input)
  Right FRONT A -> pin 3   (INT1, hardware interrupt)
  Right FRONT B -> pin 5   (plain digital input)
  Left  REAR  A -> pin 11  (PCINT, pin-change interrupt group)
  Left  REAR  B -> pin 6   (plain digital input)
  Right REAR  A -> pin 12  (PCINT, pin-change interrupt group)
  Right REAR  B -> pin 7   (plain digital input)
  Sabertooth left  channel -> pin 10 (PWM out from Arduino)
  Sabertooth right channel -> pin 9  (PWM out from Arduino)
  Arduino GND -> Sabertooth 0V
*/

#include <Arduino.h>
#include <Servo.h>

const float ENCODER_CPR       = 400.0;
const float WHEEL_DIAMETER_MM = 120.0;
const float WHEEL_CIRCUMF_MM  = 3.14159265 * WHEEL_DIAMETER_MM;

const int PWM_STOP = 1500;
const int PWM_MIN  = 1000;
const int PWM_MAX  = 2000;

const unsigned long CONTROL_PERIOD_MS = 50;

// Right side reads negative for forward rotation (mirror-mounted
// relative to left) - confirmed by hand-spin test in Stage 1. Flip it
// here so positive consistently means "this side moving robot forward"
// on both sides.
const int RIGHT_SIGN = -1;

const int LF_A = 2,  LF_B = 4;
const int RF_A = 3,  RF_B = 5;
const int LR_A = 11, LR_B = 6;
const int RR_A = 12, RR_B = 7;

Servo saberLeft;
Servo saberRight;

volatile long lfTicks = 0, rfTicks = 0, lrTicks = 0, rrTicks = 0;

void lfISR() { lfTicks += (digitalRead(LF_B) == HIGH) ? 1 : -1; }
void rfISR() { rfTicks += (digitalRead(RF_B) == HIGH) ? 1 : -1; }

volatile uint8_t lastPINB = 0;
ISR(PCINT0_vect) {
  uint8_t current = PINB;
  uint8_t changed = current ^ lastPINB;

  if ((changed & (1 << PB3)) && (current & (1 << PB3))) {
    lrTicks += (digitalRead(LR_B) == HIGH) ? 1 : -1;
  }
  if ((changed & (1 << PB4)) && (current & (1 << PB4))) {
    rrTicks += (digitalRead(RR_B) == HIGH) ? 1 : -1;
  }
  lastPINB = current;
}

int leftPWM  = PWM_STOP;
int rightPWM = PWM_STOP;
unsigned long lastControlTime = 0;

float ticksToMMs(long ticks, float dt) {
  if (dt <= 0) return 0;
  float revs = ticks / ENCODER_CPR;
  return (revs * WHEEL_CIRCUMF_MM) / dt;
}

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == 'L' || cmd == 'l') {
    long requested = Serial.parseInt();
    leftPWM = constrain(requested, PWM_MIN, PWM_MAX);
    saberLeft.writeMicroseconds(leftPWM);
    Serial.println("New left PWM (us): " + String(leftPWM));
  } else if (cmd == 'R' || cmd == 'r') {
    long requested = Serial.parseInt();
    rightPWM = constrain(requested, PWM_MIN, PWM_MAX);
    saberRight.writeMicroseconds(rightPWM);
    Serial.println("New right PWM (us): " + String(rightPWM));
  } else if (cmd == 'S' || cmd == 's') {
    leftPWM = PWM_STOP;
    rightPWM = PWM_STOP;
    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(rightPWM);
    Serial.println("STOP - both sides set to " + String(PWM_STOP));
  }
  // any other character (e.g. stray newline) is ignored
}

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

  saberLeft.attach(10);
  saberRight.attach(9);
  saberLeft.writeMicroseconds(PWM_STOP);
  saberRight.writeMicroseconds(PWM_STOP);
  delay(500);

  Serial.println("STAGE 2: open-loop drive, both sides, no PID. Starting STOPPED.");
  Serial.println("Commands: L<us>  R<us>  S(top)  e.g. L1600");
  lastControlTime = millis();
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    noInterrupts();
    long lf = lfTicks, rf = rfTicks, lr = lrTicks, rr = rrTicks;
    lfTicks = rfTicks = lrTicks = rrTicks = 0;
    interrupts();

    rf *= RIGHT_SIGN;
    rr *= RIGHT_SIGN;

    float lfMMs = ticksToMMs(lf, dt);
    float lrMMs = ticksToMMs(lr, dt);
    float rfMMs = ticksToMMs(rf, dt);
    float rrMMs = ticksToMMs(rr, dt);

    Serial.println("Lpwm:" + String(leftPWM) + " Rpwm:" + String(rightPWM) +
                    " LFmms:" + String(lfMMs) + " LRmms:" + String(lrMMs) +
                    " RFmms:" + String(rfMMs) + " RRmms:" + String(rrMMs));
  }
}
