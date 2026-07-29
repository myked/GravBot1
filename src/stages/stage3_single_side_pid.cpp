/*
  STAGE 3 -- Closed-loop PID, LEFT side only. Right side held at stop.

  Builds on Stage 2, which confirmed both channels drive cleanly under
  load, with the right-side sign correction working. This stage adds
  a single PID loop (left side only) so we can confirm target-tracking
  stability in isolation before combining both sides' loops again -
  that combination is exactly where the full sketch went unstable
  before, so we want one confirmed-good loop first.

  Right side (Sabertooth pin 9) is explicitly held at PWM_STOP the
  whole time - no Servo output changes on it at all.

  SERIAL
  ------
  <number>   set left target speed in mm/s (negative = reverse), e.g. "20"
  P<value>   set Kp live, e.g. "P0.3"
  I<value>   set Ki live, e.g. "I0.1"
  D<value>   set Kd live, e.g. "D0"
  K<value>   set stiction kick (us) live, e.g. "K0" to disable
  F<value>   set deadband floor (us) live, e.g. "F0" to disable
  S          stop (target=0, clears integral windup)
  Starts at 0 (stopped) - nothing moves until you send a target.

  TUNING - per the header comment in the full sketch this project is
  based on: start with Kp only (Ki=Kd=0), raise it until the loop
  tracks the target without overshoot, THEN add a little Ki to kill
  steady-state error, then a little Kd only if it oscillates. That
  procedure was never actually followed before - the full sketch
  shipped with Kp=0.9/Ki=0.4/Kd=0.02 as an untested guess. Starting
  here at Kp-only, low, and Ki=Kd=0 so we can tune it properly and
  interactively instead of reflashing for every gain change.

  WIRING - same as Stage 2.
*/

#include <Arduino.h>
#include <Servo.h>

const float ENCODER_CPR       = 400.0;
const float WHEEL_DIAMETER_MM = 120.0;
const float WHEEL_CIRCUMF_MM  = 3.14159265 * WHEEL_DIAMETER_MM;

float targetSpeedMMs = 0.0; // starts stopped, set via serial

// Start conservative and Kp-only, per the tuning procedure above.
float Kp = 0.3;
float Ki = 0.0;
float Kd = 0.0;

const int PWM_STOP = 1500;
const int PWM_MIN  = 1000;
const int PWM_MAX  = 2000;

// Live-tunable (K<value> over serial) - suspected of re-triggering every
// time the oscillation passes back through near-zero speed, sustaining
// a limit cycle instead of just kicking off a one-time start from rest.
int stictionKick = 40;

// Calibrated via Stage 2 open-loop bisection: below ~100us from PWM_STOP
// this drivetrain sits in a stochastic dead zone (same PWM sometimes
// stalls, sometimes breaks free into a full steady spin - real static
// friction, not a tuning problem). Deadband-compensate here: whenever
// computePID wants a nonzero offset smaller than this, snap it up to
// the floor instead of letting the loop sit and oscillate in the dead
// zone. Live-tunable (F<value> over serial).
int deadbandFloor = 100;

// Max change in written PWM per control cycle (us). Caps how fast the
// output can swing regardless of what the PID computes, so one bad
// reading can't slam the motor from full-forward to full-reverse in
// one step - suspected to be feeding a noise/overcorrection loop.
const int MAX_PWM_STEP_PER_CYCLE = 20;

const unsigned long CONTROL_PERIOD_MS = 50;

const int LF_A = 2,  LF_B = 4;
const int LR_A = 11, LR_B = 6;

Servo saberLeft;
Servo saberRight; // held at stop throughout - right side not under test

volatile long lfTicks = 0, lrTicks = 0;

void lfISR() { lfTicks += (digitalRead(LF_B) == HIGH) ? 1 : -1; }

volatile uint8_t lastPINB = 0;
ISR(PCINT0_vect) {
  uint8_t current = PINB;
  uint8_t changed = current ^ lastPINB;
  if ((changed & (1 << PB3)) && (current & (1 << PB3))) {
    lrTicks += (digitalRead(LR_B) == HIGH) ? 1 : -1;
  }
  lastPINB = current;
}

struct PID {
  float integral = 0;
  float lastError = 0;
};
PID leftPID;

unsigned long lastControlTime = 0;
int lastAppliedPWM = PWM_STOP;

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

void setup() {
  Serial.begin(9600);

  pinMode(LF_A, INPUT_PULLUP); pinMode(LF_B, INPUT_PULLUP);
  pinMode(LR_A, INPUT_PULLUP); pinMode(LR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LF_A), lfISR, RISING);

  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT3);
  lastPINB = PINB;

  saberLeft.attach(10);
  saberRight.attach(9);
  saberLeft.writeMicroseconds(PWM_STOP);
  saberRight.writeMicroseconds(PWM_STOP);
  delay(500);

  Serial.println("STAGE 3: left-side PID only. Right side held at stop.");
  Serial.println("Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
  Serial.println("Commands: <number>=target mm/s  P<v> I<v> D<v>=gains  K<v>=kick  F<v>=deadband  S=stop");
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
  } else if (c == 'S' || c == 's') {
    Serial.read();
    targetSpeedMMs = 0;
    leftPID.integral = 0;
    leftPID.lastError = 0;
    Serial.println("STOP - target 0, integral cleared");
  } else {
    targetSpeedMMs = Serial.parseFloat();
    leftPID.integral = 0; // start each new target from a clean slate
    Serial.println("New left target (mm/s): " + String(targetSpeedMMs));
  }
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    noInterrupts();
    long lf = lfTicks, lr = lrTicks;
    lfTicks = lrTicks = 0;
    interrupts();

    float lfMMs = ticksToMMs(lf, dt);
    float lrMMs = ticksToMMs(lr, dt);
    float leftSpeedMMs = (lfMMs + lrMMs) / 2.0;

    int rawPWM = computePID(leftPID, targetSpeedMMs, leftSpeedMMs, dt);
    int leftPWM = constrain(rawPWM, lastAppliedPWM - MAX_PWM_STEP_PER_CYCLE,
                                     lastAppliedPWM + MAX_PWM_STEP_PER_CYCLE);
    leftPWM = constrain(leftPWM, PWM_MIN, PWM_MAX);
    lastAppliedPWM = leftPWM;

    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(PWM_STOP); // explicitly re-assert stop

    Serial.println("tgt:" + String(targetSpeedMMs) +
                    " LFmms:" + String(lfMMs) +
                    " LRmms:" + String(lrMMs) +
                    " leftSpeedMMs:" + String(leftSpeedMMs) +
                    " rawPwm:" + String(rawPWM) +
                    " Lpwm:" + String(leftPWM));
  }
}
