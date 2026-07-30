/*
  Slow, high-torque wheel control for A4WD1 (30:1, 12V gearmotors)
  Driver: Sabertooth 2x12 in Microcontroller / Independent mode
          DIP switches: 1,4,6 DOWN | 2,3,5 UP
  Feedback: quadrature encoders on all 4 motors
  Arduino: Uno

  WIRING
  ------
  Left  FRONT A -> pin 2   (INT0, hardware interrupt)
  Left  FRONT B -> pin 4   (plain digital input)
  Right FRONT A -> pin 3   (INT1, hardware interrupt)
  Right FRONT B -> pin 5   (plain digital input)
  Left  REAR  A -> pin 11  (PCINT, pin-change interrupt group)
  Left  REAR  B -> pin 6   (plain digital input)
  Right REAR  A -> pin 12  (PCINT, pin-change interrupt group)
  Right REAR  B -> pin 7   (plain digital input)
  Sabertooth S2 (left side)  -> pin 10 (PWM out from Arduino)
  Sabertooth S1 (right side) -> pin 9  (PWM out from Arduino)
  Arduino GND -> Sabertooth 0V

  NOTE: confirmed by bench test that pin 9 drives the Sabertooth's
  right-side output and pin 10 drives the left-side output - opposite
  of what the S1/S2 labeling might suggest. If you rewire the
  Sabertooth screw terminals so S1=left/S2=right, swap the pin numbers
  in saberLeft.attach()/saberRight.attach() below to match.

  STABILITY FIXES (from staged bring-up in src/stages/) - see those
  files for the full investigation:
    1. DIP switches must be firmly seated in Independent mode. A half
       seated switch silently runs Mixed mode instead, where one
       Sabertooth input becomes "throttle" and the other becomes
       "turn" - both wheels then react to both PID loops at once,
       which reads exactly like uncontrollable oscillation.
    2. Right-side encoders read NEGATIVE for forward rotation (mirror-
       mounted relative to left) - see RIGHT_SIGN below. Without this,
       the right PID loop sees inverted feedback and fights itself.
    3. PID output is now slew-rate limited (MAX_PWM_STEP_PER_CYCLE) -
       without it, a single noisy reading can slam the motor from
       full-forward to full-reverse in one 50ms step, and that abrupt
       reversal induces real electrical/mechanical noise that
       corrupts the next reading too - a self-sustaining feedback
       loop, which was the root cause of the original wild instability.
    4. Below ~100us PWM offset this drivetrain sits in a stochastic
       dead zone (see MIN_RELIABLE_PWM_OFFSET) - the deadband
       compensation in computePID() snaps any nonzero-target output up
       to that floor rather than letting it hover in the unreliable
       zone. It's guarded to only apply when the target is actually
       nonzero, so the robot still coasts to a genuine stop at
       target=0 instead of jittering.

  HC-05 Bluetooth (wireless monitoring/tuning from a paired PC or phone):
  HC-05 VCC -> Uno 5V
  HC-05 GND -> Uno GND
  HC-05 TX  -> Uno pin 0 (RX) (direct - 3.3V out is fine into Uno RX)
  HC-05 RX  -> Uno pin 1 (TX) (THROUGH a 1k/2k voltage divider - Uno's 5V
                                TX would over-drive the HC-05's 3.3V RX pin)
  Wired directly to the hardware Serial pins (0/1), NOT SoftwareSerial.
  SoftwareSerial was tried first, but it unconditionally claims all
  three AVR pin-change-interrupt vectors as soon as any SoftwareSerial
  object is used, regardless of which pins you pick - that's a hard
  conflict with the rear encoders' hand-written ISR(PCINT0_vect) (link
  error: "multiple definition of __vector_3"). AltSoftSerial avoids
  that but needs Timer1, which the Servo library already owns
  exclusively for both Sabertooth channels. Hardware Serial sidesteps
  both problems entirely.

  TRADEOFF: pins 0/1 are shared with the USB-to-serial chip, so USB
  Serial Monitor and a live Bluetooth connection can't both be active
  at once, and the HC-05's RX line should be disconnected (or switched
  out) while uploading new code via USB, since stray Bluetooth traffic
  can interfere with the bootloader.

  Once paired, Windows exposes the HC-05 as a virtual COM port - point
  PlatformIO's Serial Monitor at that port instead of the USB one to
  watch telemetry and send target-speed commands wirelessly. Uploading
  new code over that same virtual port is possible in principle but
  unreliable (no DTR auto-reset over Bluetooth SPP, and the HC-05's
  default 9600 baud doesn't match the bootloader's expected upload
  rate) - keep using USB for uploads.
  Default HC-05 baud is 9600 unless you've reconfigured it via AT
  commands - matches Serial.begin() below.

  WHY THIS SETUP
  --------------
  Left-side motors (front+rear) are paralleled onto Sabertooth M1,
  right-side motors onto M2 - they get the same PWM command. But the
  two motors on a side are mechanically independent: if one wheel
  loses traction it will spin faster than its paired motor while the
  other stays grounded. Reading all 4 encoders lets us:
    1. Detect that speed mismatch (wheelspin/skid) per side.
    2. Use the AVERAGE of front+rear speed on a side as a more robust
       feedback signal for the PID loop than either encoder alone.

  A Uno only has 2 true hardware-interrupt pins (2, 3), so the rear
  encoders use the ATmega328's pin-change interrupt feature instead
  (covers digital pins 8-13, handled manually via PCICR/PCMSK0 below -
  no external library needed).

  TUNING
  ------
  Start with Kp only (Ki=Kd=0), raise until it tracks target speed
  without overshoot, then add small Ki to kill steady-state error,
  then a little Kd if it oscillates.
*/

#include <Arduino.h>
#include <Servo.h>

// ---------------- USER CONFIG ----------------

const float ENCODER_CPR       = 400.0;   // counts per motor-output-shaft rev
const float WHEEL_DIAMETER_MM = 120.0;   // confirm with calipers
const float WHEEL_CIRCUMF_MM  = 3.14159265 * WHEEL_DIAMETER_MM;

// Target crawl speed, mm/s. Positive = forward, negative = reverse.
// NOTE: the original design value here was 30.0, but calibration found
// this drivetrain can't reliably hold speeds that low - see
// MIN_RELIABLE_PWM_OFFSET. 500 is validated (src/stages/stage4_full_dual_pid.cpp)
// to track with bounded, safe PWM output. Expect some oscillation around
// the target (tens up to ~1000mm/s) rather than a perfectly smooth hold -
// tried raising Kp/Ki and loosening the slew-rate limit to tighten this,
// none helped (loosening the slew limit made it worse - it turned out to
// be usefully damping the oscillation, not just holding back convergence).
// Tightening further would need sensor filtering, not just gain tuning.
float targetSpeedMMs = 500.0;

// PID gains - the original 0.9/0.4/0.02 here were an untested guess and
// caused the instability documented above. Reset to the Kp-only, low
// starting point validated in src/stages/stage3_single_side_pid.cpp and
// stage4_full_dual_pid.cpp; raise Kp first, then add small Ki/Kd per the
// TUNING note below, watching for the slew-limited Lpwm/Rpwm staying
// bounded rather than banging between PWM_MIN/PWM_MAX.
float Kp = 0.3;
float Ki = 0.0;
float Kd = 0.0;

// Sabertooth PWM output limits (microseconds), 1500 = stop
const int PWM_STOP = 1500;
const int PWM_MIN  = 1000;
const int PWM_MAX  = 2000;

// Minimum "kick" offset (us) to overcome stiction when starting from rest
const int STICTION_KICK = 40;

// Calibrated by bisecting open-loop PWM offsets on the left side (see
// src/stages/stage2_open_loop_drive.cpp). Below ~100us from PWM_STOP, this
// drivetrain sits in a STOCHASTIC dead zone: the same commanded PWM
// sometimes stalls (gear backlash twitch only) and sometimes breaks free
// into a full, steady spin (~1500mm/s) on different trials - static
// friction, not something further PID tuning can fix. 1600us (100us
// offset) reliably produced sustained, repeatable rotation across
// multiple tests. Treat this as the practical floor for continuous PWM
// drive; a genuinely slower crawl would need PWM dithering (short pulses
// at/above this floor, separated by stops) rather than a lower duty
// cycle held continuously.
const int MIN_RELIABLE_PWM_OFFSET = 100;

// Max change in written PWM per control cycle (us). Caps how fast the
// output can swing regardless of what the PID computes - see
// STABILITY FIXES #3 above.
const int MAX_PWM_STEP_PER_CYCLE = 20;

// If front/rear speed on a side differ by more than this, flag slip
const float SLIP_THRESHOLD_MMS = 15.0;

// --- Slip response (traction control) ---
// This backs off torque on a side once it's slipping past SLIP_THRESHOLD_MMS.
// Both are set to have NO EFFECT until you deliberately turn them on:
//   slipResponseEnabled = false  -> correction code never runs, PID output passes through untouched
//   slipCorrectionGain  = 0.0    -> even if enabled, gain of 0 means zero correction
// To start testing: first watch [LEFT SLIP]/[RIGHT SLIP] in Serial and confirm
// SLIP_THRESHOLD_MMS is set sensibly for your robot, THEN set
// slipResponseEnabled = true and bring slipCorrectionGain up from a small
// value (e.g. 2.0) - it scales "mm/s of slip beyond threshold" into
// "microseconds pulled back toward stop".
bool  slipResponseEnabled = false;
float slipCorrectionGain  = 0.0;

// How often the control loop runs
const unsigned long CONTROL_PERIOD_MS = 50; // 20 Hz

// Right side reads negative for forward rotation (mirror-mounted
// relative to left) - see STABILITY FIXES #2 above.
const int RIGHT_SIGN = -1;

// ---------------- PIN CONFIG ----------------

const int LF_A = 2,  LF_B = 4;   // left front  (hardware interrupt)
const int RF_A = 3,  RF_B = 5;   // right front (hardware interrupt)
const int LR_A = 11, LR_B = 6;   // left rear   (pin-change interrupt)
const int RR_A = 12, RR_B = 7;   // right rear  (pin-change interrupt)

// ---------------- INTERNAL STATE ----------------

Servo saberLeft;
Servo saberRight;

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

struct PID {
  float integral = 0;
  float lastError = 0;
};
PID leftPID, rightPID;

unsigned long lastControlTime = 0;
int lastAppliedLeftPWM = PWM_STOP;
int lastAppliedRightPWM = PWM_STOP;

// ---------------- FUNCTION PROTOTYPES ----------------
// Declared here so they're known before use in loop(), rather than
// relying on the Arduino build step to auto-generate prototypes.

float ticksToMMs(long ticks, float dt);
int computePID(PID &state, float target, float measured, float dt);
int applySlipCorrection(int pwm, float frontMMs, float rearMMs);
int slewLimit(int rawPWM, int &lastApplied);

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(9600); // shared USB + HC-05 (see WIRING notes above)

  pinMode(LF_A, INPUT_PULLUP); pinMode(LF_B, INPUT_PULLUP);
  pinMode(RF_A, INPUT_PULLUP); pinMode(RF_B, INPUT_PULLUP);
  pinMode(LR_A, INPUT_PULLUP); pinMode(LR_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LF_A), lfISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RF_A), rfISR, RISING);

  // Enable pin-change interrupts for pins 11 (PCINT3) and 12 (PCINT4)
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT3) | (1 << PCINT4);
  lastPINB = PINB;

  saberLeft.attach(10);
  saberRight.attach(9);
  saberLeft.writeMicroseconds(PWM_STOP);
  saberRight.writeMicroseconds(PWM_STOP);
  delay(500);

  Serial.println("Ready. Send a number to set target speed in mm/s (negative = reverse).");
  lastControlTime = millis();
}

// ---------------- CONTROL LOOP ----------------

void loop() {
  if (Serial.available()) {
    targetSpeedMMs = Serial.parseFloat();
    // Drain any trailing CR/LF left behind by the sender (some terminal
    // apps send \r\n rather than a single terminator) - otherwise the
    // leftover byte trips Serial.available() again next loop, and a
    // second parseFloat() call with nothing but whitespace to read
    // times out and silently overwrites targetSpeedMMs back to 0.
    while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }
    Serial.println("New target speed (mm/s): " + String(targetSpeedMMs));
  }

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

    float leftFrontMMs  = ticksToMMs(lf, dt);
    float leftRearMMs   = ticksToMMs(lr, dt);
    float rightFrontMMs = ticksToMMs(rf, dt);
    float rightRearMMs  = ticksToMMs(rr, dt);

    // String line = " leftFrontMMs:" + String(leftFrontMMs) +
    //               " leftRearMMs:" + String(leftRearMMs) +
    //               " rightFrontMMs:" + String(rightFrontMMs) +
    //               " rightRearMMs:" + String(rightRearMMs);
    // Serial.println(line);

    // Use the average of front+rear per side as the PID feedback signal
    float leftSpeedMMs  = (leftFrontMMs + leftRearMMs) / 2.0;
    float rightSpeedMMs = (rightFrontMMs + rightRearMMs) / 2.0;

    // Slip detection: compare front vs rear on each side
    bool leftSlip  = fabs(leftFrontMMs - leftRearMMs) > SLIP_THRESHOLD_MMS;
    bool rightSlip = fabs(rightFrontMMs - rightRearMMs) > SLIP_THRESHOLD_MMS;

    int leftPWM  = computePID(leftPID,  targetSpeedMMs, leftSpeedMMs, dt);
    int rightPWM = computePID(rightPID, targetSpeedMMs, rightSpeedMMs, dt);

    leftPWM  = applySlipCorrection(leftPWM,  leftFrontMMs,  leftRearMMs);
    rightPWM = applySlipCorrection(rightPWM, rightFrontMMs, rightRearMMs);

    leftPWM  = slewLimit(leftPWM,  lastAppliedLeftPWM);
    rightPWM = slewLimit(rightPWM, lastAppliedRightPWM);

    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(rightPWM);

    String line = "tgt:" + String(targetSpeedMMs) +
                  " LF:" + String(leftFrontMMs) +
                  " LR:" + String(leftRearMMs) +
                  " RF:" + String(rightFrontMMs) +
                  " RR:" + String(rightRearMMs) +
                  " leftSpeedMMs:" + String(leftSpeedMMs) +
                  " rightSpeedMMs:" + String(rightSpeedMMs) +
                  " Lpwm:" + String(leftPWM) +
                  " Rpwm:" + String(rightPWM);
    if (leftSlip)  line += " [LEFT SLIP]";
    if (rightSlip) line += " [RIGHT SLIP]";
    Serial.println(line);
  }
}

// ---------------- HELPERS ----------------

float ticksToMMs(long ticks, float dt) {
  if (dt <= 0) return 0;
  float revs = ticks / ENCODER_CPR;
  return (revs * WHEEL_CIRCUMF_MM) / dt;
}

// Pulls PWM back toward PWM_STOP proportionally to slip severity.
// No-op unless slipResponseEnabled is true AND slipCorrectionGain > 0.
int applySlipCorrection(int pwm, float frontMMs, float rearMMs) {
  if (!slipResponseEnabled || slipCorrectionGain <= 0.0) return pwm;

  float diff = fabs(frontMMs - rearMMs);
  if (diff <= SLIP_THRESHOLD_MMS) return pwm;

  float excess = diff - SLIP_THRESHOLD_MMS;
  int reduction = (int)(excess * slipCorrectionGain);

  if (pwm > PWM_STOP) pwm = max(PWM_STOP, pwm - reduction);
  else if (pwm < PWM_STOP) pwm = min(PWM_STOP, pwm + reduction);

  return pwm;
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
    pwm += (output > 0) ? STICTION_KICK : -STICTION_KICK;
  }

  // Deadband compensation - only when we actually want motion (nonzero
  // target), otherwise a tiny residual-noise correction while trying to
  // sit at target=0 would get blown up into a full-strength drive
  // command. See STABILITY FIXES #4 above.
  int offset = pwm - PWM_STOP;
  if (fabs(target) > 0.1 && offset != 0 && abs(offset) < MIN_RELIABLE_PWM_OFFSET) {
    pwm = PWM_STOP + (offset > 0 ? MIN_RELIABLE_PWM_OFFSET : -MIN_RELIABLE_PWM_OFFSET);
  }

  return constrain(pwm, PWM_MIN, PWM_MAX);
}

// Caps how much the written PWM can change in one control cycle,
// regardless of what computePID/applySlipCorrection want - see
// STABILITY FIXES #3 above.
int slewLimit(int rawPWM, int &lastApplied) {
  int applied = constrain(rawPWM, lastApplied - MAX_PWM_STEP_PER_CYCLE,
                                    lastApplied + MAX_PWM_STEP_PER_CYCLE);
  applied = constrain(applied, PWM_MIN, PWM_MAX);
  lastApplied = applied;
  return applied;
}
