/*
  STAGE 4 -- Closed-loop PID, BOTH sides together.

  Combines everything validated in Stages 1-3:
    - pin mapping: left=pin10, right=pin9 (confirmed by bench test,
      opposite of what the Sabertooth S1/S2 labeling suggests)
    - right side needs its raw ticks sign-flipped (RIGHT_SIGN) so
      "forward" reads positive on both sides, matching the hand-spin
      test from Stage 1/2
    - PID output slew-rate limiting, so a single bad reading can't
      slam the motor between full-forward/full-reverse and start a
      noise-feedback loop - this was the root cause of the original
      wild instability in the full sketch
    - stiction kick + deadband floor compensation, guarded so they
      only apply when there's an actual nonzero target, not while
      genuinely trying to sit at rest (that guard was a bug we fixed
      in Stage 3 - it was causing bang-bang oscillation at target=0)
    - confirmed minimum reliable PWM offset (~100us / 1600us) - this
      drivetrain can't hold a smooth 20-50mm/s crawl; that's a real
      static-friction characteristic, not a tuning problem. Send a
      realistic target like 500 to start, not something down in the
      20-50 range.

  Both sides share one target speed and one set of PID/kick/deadband
  gains, since it's the same drivetrain hardware on each side.

  SERIAL
  ------
  <number>   set target speed in mm/s for BOTH sides (negative = reverse)
  P<value>   set Kp live, e.g. "P0.3"
  I<value>   set Ki live, e.g. "I0.1"
  D<value>   set Kd live, e.g. "D0"
  K<value>   set stiction kick (us) live, e.g. "K0" to disable
  F<value>   set deadband floor (us) live, e.g. "F0" to disable
  M<value>   set max PWM step per cycle (us) live, e.g. "M40"
  S          stop (target=0, clears both integrals)
  Starts at 0 (stopped) - nothing moves until you send a target.

  WIRING - same as the full sketch (src/main.cpp).
*/

#include <Arduino.h>
#include <Servo.h>

const float ENCODER_CPR       = 400.0;
const float WHEEL_DIAMETER_MM = 120.0;
const float WHEEL_CIRCUMF_MM  = 3.14159265 * WHEEL_DIAMETER_MM;

float targetSpeedMMs = 0.0; // starts stopped, set via serial

// Start conservative and Kp-only, per the tuning procedure in main.cpp.
float Kp = 0.3;
float Ki = 0.0;
float Kd = 0.0;

const int PWM_STOP = 1500;
const int PWM_MIN  = 1000;
const int PWM_MAX  = 2000;

// Live-tunable (K<value> over serial).
int stictionKick = 40;

// Calibrated via Stage 2 open-loop bisection: below ~100us from PWM_STOP
// this drivetrain sits in a stochastic dead zone (same PWM sometimes
// stalls, sometimes breaks free into a full steady spin). Live-tunable
// (F<value> over serial).
int deadbandFloor = 100;

// Max change in written PWM per control cycle (us) - caps how fast the
// output can swing regardless of what the PID computes. Live-tunable
// (M<value> over serial).
int maxPwmStepPerCycle = 20;

const unsigned long CONTROL_PERIOD_MS = 50;

// Right side reads negative for forward rotation (mirror-mounted
// relative to left) - confirmed by hand-spin test.
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

struct PID {
  float integral = 0;
  float lastError = 0;
};
PID leftPID, rightPID;

unsigned long lastControlTime = 0;
int lastAppliedLeftPWM = PWM_STOP;
int lastAppliedRightPWM = PWM_STOP;

float ticksToMMs(long ticks, float dt) {
  if (dt <= 0) return 0;
  float revs = ticks / ENCODER_CPR;
  return (revs * WHEEL_CIRCUMF_MM) / dt;
}

int computePID(PID &state, float target, float measured, float dt) {
  float error = target - measured;

  state.integral += error * dt;
  state.integral = constrain(state.integral, -200, 200);

  float derivative = (dt > 0) ? (error - state.lastError) / dt : 0;
  state.lastError = error;

  float output = Kp * error + Ki * state.integral + Kd * derivative;
  int pwm = PWM_STOP + (int)output;

  if (fabs(target) > 0.1 && fabs(measured) < 1.0) {
    pwm += (output > 0) ? stictionKick : -stictionKick;
  }

  // Only force the floor when we actually want motion (nonzero target) -
  // otherwise a tiny residual-noise correction while trying to sit at
  // target=0 gets blown up into a full-strength drive command.
  int offset = pwm - PWM_STOP;
  if (fabs(target) > 0.1 && offset != 0 && abs(offset) < deadbandFloor) {
    pwm = PWM_STOP + (offset > 0 ? deadbandFloor : -deadbandFloor);
  }

  return constrain(pwm, PWM_MIN, PWM_MAX);
}

int slewLimit(int rawPWM, int &lastApplied) {
  int applied = constrain(rawPWM, lastApplied - maxPwmStepPerCycle,
                                    lastApplied + maxPwmStepPerCycle);
  applied = constrain(applied, PWM_MIN, PWM_MAX);
  lastApplied = applied;
  return applied;
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

  Serial.println("STAGE 4: both-side PID. Starting STOPPED.");
  Serial.println("Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd) +
                  " kick=" + String(stictionKick) + " floor=" + String(deadbandFloor));
  Serial.println("Commands: <number>=target mm/s  P<v> I<v> D<v>=gains  K<v>=kick  F<v>=deadband  M<v>=maxstep  S=stop");
  lastControlTime = millis();
}

void handleSerial() {
  if (!Serial.available()) return;

  int c = Serial.peek();

  if (c == 'P' || c == 'p') {
    Serial.read();
    Kp = Serial.parseFloat();
    Serial.println("Kp = " + String(Kp));
  } else if (c == 'I' || c == 'i') {
    Serial.read();
    Ki = Serial.parseFloat();
    Serial.println("Ki = " + String(Ki));
  } else if (c == 'D' || c == 'd') {
    Serial.read();
    Kd = Serial.parseFloat();
    Serial.println("Kd = " + String(Kd));
  } else if (c == 'K' || c == 'k') {
    Serial.read();
    stictionKick = Serial.parseInt();
    Serial.println("stictionKick = " + String(stictionKick));
  } else if (c == 'F' || c == 'f') {
    Serial.read();
    deadbandFloor = Serial.parseInt();
    Serial.println("deadbandFloor = " + String(deadbandFloor));
  } else if (c == 'M' || c == 'm') {
    Serial.read();
    maxPwmStepPerCycle = Serial.parseInt();
    Serial.println("maxPwmStepPerCycle = " + String(maxPwmStepPerCycle));
  } else if (c == 'S' || c == 's') {
    Serial.read();
    targetSpeedMMs = 0;
    leftPID.integral = 0;  leftPID.lastError = 0;
    rightPID.integral = 0; rightPID.lastError = 0;
    Serial.println("STOP - target 0, integrals cleared");
  } else {
    targetSpeedMMs = Serial.parseFloat();
    leftPID.integral = 0;
    rightPID.integral = 0;
    Serial.println("New target (mm/s): " + String(targetSpeedMMs));
  }
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

    float leftSpeedMMs  = (lfMMs + lrMMs) / 2.0;
    float rightSpeedMMs = (rfMMs + rrMMs) / 2.0;

    int rawLeftPWM  = computePID(leftPID,  targetSpeedMMs, leftSpeedMMs, dt);
    int rawRightPWM = computePID(rightPID, targetSpeedMMs, rightSpeedMMs, dt);

    int leftPWM  = slewLimit(rawLeftPWM, lastAppliedLeftPWM);
    int rightPWM = slewLimit(rawRightPWM, lastAppliedRightPWM);

    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(rightPWM);

    Serial.println("tgt:" + String(targetSpeedMMs) +
                    " LFmms:" + String(lfMMs) +
                    " LRmms:" + String(lrMMs) +
                    " RFmms:" + String(rfMMs) +
                    " RRmms:" + String(rrMMs) +
                    " leftSpeedMMs:" + String(leftSpeedMMs) +
                    " rightSpeedMMs:" + String(rightSpeedMMs) +
                    " Lpwm:" + String(leftPWM) +
                    " Rpwm:" + String(rightPWM));
  }
}
