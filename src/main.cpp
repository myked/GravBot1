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

  HC-SR04 ultrasonic (front bumper/obstacle sensor):
  HC-SR04 VCC  -> Uno 5V
  HC-SR04 GND  -> Uno GND
  HC-SR04 TRIG -> Uno pin 8  (output, 10us trigger pulse)
  HC-SR04 ECHO -> Uno pin A0 (input, pulseIn() measures echo width)

  POWER: the Sabertooth 2x12's onboard 5V BEC feeds the Uno and
  everything else on the 5V rail (HC-05, HC-SR04) - no separate
  regulator needed. Confirm the BEC jumper is set to supply 5V; if it's
  ever removed/disabled, powering the Uno via USB alone would leave the
  HC-05 and HC-SR04 without 5V.

  Sampled independently of the 20Hz motor control loop (see
  SENSOR_PERIOD_MS below) - pulseIn() blocks for up to ECHO_TIMEOUT_US,
  and running it inside the 50ms control cycle every time would eat a
  meaningful chunk of that budget. Currently informational only:
  frontDistanceMM is read and logged in telemetry, but nothing in the
  control loop reacts to it yet - it does not brake or stop the robot.

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

// Steering bias, mm/s. Positive = right turn (left side sped up relative
// to right), negative = left turn. leftTarget = targetSpeedMMs -
// turnRateMMs, rightTarget = targetSpeedMMs + turnRateMMs. Set via the
// second number on the serial command line, e.g. "500 100" - the turn
// value is optional, defaults to 0 (straight) if omitted.
float turnRateMMs = 0.0;

// Speed/turn magnitudes used for single-character RC app commands (see
// handleRcChar() below) - matches the "Bluetooth RC Controller" app's
// default F/B/L/R/G/H/I/J/S protocol. The app streams the same character
// continuously while a direction is held, so holding F/B (or a diagonal)
// ramps targetSpeedMMs up/down by RC_SPEED_STEP_MMS each time the
// character repeats, up to the RC_SPEED_MMS ceiling - rather than
// jumping straight there. L/R stay an instant pivot turn (targetSpeedMMs
// forced to 0, turnRateMMs set directly), a separate action from
// throttle. None of these three have been bench tuned yet - if the ramp
// feels too fast/slow or turns too sharp/soft, adjust them.
const float RC_SPEED_MMS      = 3000.0; // ceiling for the held-throttle ramp
const float RC_SPEED_STEP_MMS = 40.0;   // added/subtracted per repeat while held
// Lowered from 1000 - at 1000 a static turn pushes one side's target to
// 1500mm/s (500 base +/- 1000 turn), deep enough into the nonlinear zone
// above MIN_RELIABLE_PWM_OFFSET that computePID's raw output saturates
// almost every cycle. With the error that large Kp stops mattering and
// the slew limiter alone drives the output, walking +/-MAX_PWM_STEP_PER_CYCLE
// every single cycle with no convergence - that's the bench-observed
// jerkiness (Lpwm ping-ponging 1580/1600, Rpwm sawtoothing 1400-1600).
// 650 keeps pivot targets in a less saturated range.
const float RC_TURN_MMS       = 650.0;  // fixed turn bias, applied instantly

// PID gains - the original 0.9/0.4/0.02 here were an untested guess and
// caused the instability documented above. 0.3 was the next validated
// starting point (src/stages/stage3_single_side_pid.cpp,
// stage4_full_dual_pid.cpp), but bench testing of the static turn showed
// even 0.3 is too high: MAX_PWM_STEP_PER_CYCLE rate-limits the actuator,
// and at Kp=0.3 the computed output almost always demands a bigger PWM
// swing per cycle than the slew limit can deliver - a rate-limited
// actuator plus too much loop gain is a textbook recipe for a sustained
// limit cycle (Lpwm/Rpwm banging between extremes, wheels visibly
// reversing). Dropped to 0.08 as a lower starting point; use the live
// P<value> serial command to bench-tune upward from here without
// reflashing, watching for Lpwm/Rpwm settling near a value instead of
// oscillating.
float Kp = 0.08;
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

// Below this measured speed, the wheel is considered still "stalled" and
// the MIN_RELIABLE_PWM_OFFSET floor below applies (breaking static
// friction from a near-stop needs the guaranteed-spin kick). Above it,
// the wheel is already moving and small PID corrections are let through
// unmodified - kinetic friction is lower than static, so holding/nudging
// an already-spinning wheel doesn't need the same floor. Without this
// guard the floor was firing on EVERY cycle where target != 0 (most of
// them, since a small tracking error produces a small Kp*error output),
// turning ordinary fine corrections into full +/-100us commands and
// producing a relay-like oscillation that never settled regardless of Kp
// - see bench notes on Kp above.
const float DEADBAND_RECOVERY_SPEED_MMS = 50.0;

// Max change in written PWM per control cycle (us). Caps how fast the
// output can swing regardless of what the PID computes - see
// STABILITY FIXES #3 above.
const int MAX_PWM_STEP_PER_CYCLE = 20;

// If front/rear speed on a side differ by more than this, flag slip
const float SLIP_THRESHOLD_MMS = 15.0;

// --- Front ultrasonic sensor (HC-SR04) ---
// Informational only for now - see loop() and measureFrontDistanceMM().
const unsigned long SENSOR_PERIOD_MS = 100;   // 10Hz, independent of the 20Hz control loop
const unsigned long ECHO_TIMEOUT_US  = 25000; // bounds pulseIn()'s worst-case block (~4.3m range)
const float OBSTACLE_WARN_MM         = 300.0; // flagged in telemetry below this range, not acted on

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
bool  slipResponseEnabled = false;x
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

const int BUMPER_TRIG = 8;   // HC-SR04 trigger (output)
const int BUMPER_ECHO = A0;  // HC-SR04 echo (input)

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

unsigned long lastSensorTime = 0;
float frontDistanceMM = -1; // last HC-SR04 reading; -1 = no echo / out of range

// ---------------- FUNCTION PROTOTYPES ----------------
// Declared here so they're known before use in loop(), rather than
// relying on the Arduino build step to auto-generate prototypes.

float ticksToMMs(long ticks, float dt);
float measureFrontDistanceMM();
int computePID(PID &state, float target, float measured, float dt);
int applySlipCorrection(int pwm, float frontMMs, float rearMMs);
int slewLimit(int rawPWM, int &lastApplied);
bool handleRcChar(int c);

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(9600); // shared USB + HC-05 (see WIRING notes above)

  pinMode(LF_A, INPUT_PULLUP); pinMode(LF_B, INPUT_PULLUP);
  pinMode(RF_A, INPUT_PULLUP); pinMode(RF_B, INPUT_PULLUP);
  pinMode(LR_A, INPUT_PULLUP); pinMode(LR_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  pinMode(BUMPER_TRIG, OUTPUT);
  pinMode(BUMPER_ECHO, INPUT);
  digitalWrite(BUMPER_TRIG, LOW);

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

  Serial.println("Ready. Send speed in mm/s, optionally followed by a turn rate,");
  Serial.println("e.g. \"500\" for straight or \"500 100\" to bias right (negative = reverse/left).");
  Serial.println("Or drive with the Bluetooth RC Controller app (F/B/L/R/G/H/I/J/S).");
  Serial.println("P<value>=set Kp live, e.g. P0.3   K<value>=set Ki live, e.g. K0.1");
  Serial.println("Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
  Serial.println("Front ultrasonic (HC-SR04) logged in telemetry as front:<mm>, -1 = no echo.");
  lastControlTime = millis();
}

// ---------------- CONTROL LOOP ----------------

void loop() {
  if (Serial.available()) {
    int c = Serial.peek();
    // 'P'/'K' for live Kp/Ki tuning - not 'I' for Ki, since 'I' is already
    // the RC app's forward-right command. Neither letter collides with any
    // RC char (F/B/L/R/G/H/I/J/S/U/D), so this can't misfire from the app.
    bool isGainCmd = (c == 'P' || c == 'p' || c == 'K' || c == 'k');

    if (c == 'P' || c == 'p') {
      Serial.read();
      Kp = Serial.parseFloat();
      Serial.println("Kp = " + String(Kp));
    } else if (c == 'K' || c == 'k') {
      Serial.read();
      Ki = Serial.parseFloat();
      Serial.println("Ki = " + String(Ki));
    } else if (!handleRcChar(c)) {
      // Not an RC app character - fall back to the typed "speed [turn]"
      // format used by a plain terminal app.
      targetSpeedMMs = Serial.parseFloat();

      // Turn rate is optional - a bare "500" should set turnRateMMs=0
      // without a long stall. parseFloat() blocks up to Serial's timeout
      // (default 1000ms) waiting for a digit when the buffer's empty, so
      // shorten it here: a turn value sent right after the speed value is
      // already buffered and reads back near-instantly, but nothing to
      // read isn't worth waiting a full second for.
      unsigned long defaultTimeout = Serial.getTimeout();
      Serial.setTimeout(50);
      turnRateMMs = Serial.parseFloat();
      Serial.setTimeout(defaultTimeout);
    }

    // Drain any trailing CR/LF left behind by the sender (some terminal
    // apps send \r\n rather than a single terminator) - otherwise the
    // leftover byte trips Serial.available() again next loop, and a
    // stray re-parse silently overwrites these values back to 0.
    while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }
    if (!isGainCmd) {
      Serial.println("New target speed (mm/s): " + String(targetSpeedMMs) +
                      "  turn rate (mm/s): " + String(turnRateMMs));
    }
  }

  unsigned long now = millis();

  // Sampled on its own cadence, independent of the 20Hz control loop
  // below - see the HC-SR04 note in the header comment.
  if (now - lastSensorTime >= SENSOR_PERIOD_MS) {
    lastSensorTime = now;
    frontDistanceMM = measureFrontDistanceMM();
  }

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

    // Use the average of front+rear per side as the PID feedback signal.
    // (Tried EMA-filtering this to fight the oscillation below - made it
    // worse: this loop's actuator is rate-limited (MAX_PWM_STEP_PER_CYCLE)
    // and Kp was demanding bigger swings than the slew limit could deliver
    // in one cycle, which is a rate-limit-induced limit cycle, not sensor
    // noise. Adding filter lag to an already-oscillating rate-limited loop
    // just widens the oscillation instead of damping it - see Kp note
    // below for the actual fix.)
    float leftSpeedMMs  = (leftFrontMMs + leftRearMMs) / 2.0;
    float rightSpeedMMs = (rightFrontMMs + rightRearMMs) / 2.0;

    // Slip detection: compare front vs rear on each side
    bool leftSlip  = fabs(leftFrontMMs - leftRearMMs) > SLIP_THRESHOLD_MMS;
    bool rightSlip = fabs(rightFrontMMs - rightRearMMs) > SLIP_THRESHOLD_MMS;

    float leftTargetMMs  = targetSpeedMMs - turnRateMMs;
    float rightTargetMMs = targetSpeedMMs + turnRateMMs;

    int leftPWM  = computePID(leftPID,  leftTargetMMs,  leftSpeedMMs, dt);
    int rightPWM = computePID(rightPID, rightTargetMMs, rightSpeedMMs, dt);

    leftPWM  = applySlipCorrection(leftPWM,  leftFrontMMs,  leftRearMMs);
    rightPWM = applySlipCorrection(rightPWM, rightFrontMMs, rightRearMMs);

    leftPWM  = slewLimit(leftPWM,  lastAppliedLeftPWM);
    rightPWM = slewLimit(rightPWM, lastAppliedRightPWM);

    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(rightPWM);

    String line = "tgt:" + String(targetSpeedMMs) +
                  " turn:" + String(turnRateMMs) +
                  " leftTgt:" + String(leftTargetMMs) +
                  " rightTgt:" + String(rightTargetMMs) +
                  " LF:" + String(leftFrontMMs) +
                  " LR:" + String(leftRearMMs) +
                  " RF:" + String(rightFrontMMs) +
                  " RR:" + String(rightRearMMs) +
                  " leftSpeedMMs:" + String(leftSpeedMMs) +
                  " rightSpeedMMs:" + String(rightSpeedMMs) +
                  " Lpwm:" + String(leftPWM) +
                  " Rpwm:" + String(rightPWM) +
                  " front:" + String(frontDistanceMM);
    if (leftSlip)  line += " [LEFT SLIP]";
    if (rightSlip) line += " [RIGHT SLIP]";
    if (frontDistanceMM > 0 && frontDistanceMM < OBSTACLE_WARN_MM) line += " [OBSTACLE]";
    Serial.println(line);
  }
}

// ---------------- HELPERS ----------------

// Maps the "Bluetooth RC Controller" app's default single-character
// protocol (streamed continuously while a direction is held) onto
// targetSpeedMMs/turnRateMMs. Returns true and consumes the character
// if it matched an RC command, false (without consuming anything) if
// it didn't - so the caller can fall back to numeric parsing.
bool handleRcChar(int c) {
  switch (c) {
    case 'F': case 'f':
      targetSpeedMMs = constrain(targetSpeedMMs + RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = 0;
      break;
    case 'B': case 'b':
      targetSpeedMMs = constrain(targetSpeedMMs - RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = 0;
      break;
    case 'L': case 'l':
      // Curve, don't stop-and-pivot - leave whatever speed is already
      // set alone, only bias the turn. Pressing F/B resets turnRateMMs
      // back to 0, which doubles as a "go straight" action.
      turnRateMMs = -RC_TURN_MMS;
      break;
    case 'R': case 'r':
      turnRateMMs = RC_TURN_MMS;
      break;
    case 'G': case 'g': // forward-left
      targetSpeedMMs = constrain(targetSpeedMMs + RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = -RC_TURN_MMS;
      break;
    case 'I': case 'i': // forward-right
      targetSpeedMMs = constrain(targetSpeedMMs + RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = RC_TURN_MMS;
      break;
    case 'H': case 'h': // backward-left
      targetSpeedMMs = constrain(targetSpeedMMs - RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = -RC_TURN_MMS;
      break;
    case 'J': case 'j': // backward-right
      targetSpeedMMs = constrain(targetSpeedMMs - RC_SPEED_STEP_MMS, -RC_SPEED_MMS, RC_SPEED_MMS);
      turnRateMMs = RC_TURN_MMS;
      break;
    case 'S': case 's':
      targetSpeedMMs = 0; turnRateMMs = 0; // deliberate release - stop promptly, don't coast
      break;
    case 'U': case 'u': // full speed forward - jumps straight to the ceiling, no ramp needed
      targetSpeedMMs = RC_SPEED_MMS; turnRateMMs = 0;
      break;
    case 'D': case 'd': // full speed backward
      targetSpeedMMs = -RC_SPEED_MMS; turnRateMMs = 0;
      break;
    default:
      return false;
  }
  Serial.read(); // consume the command character
  return true;
}

float ticksToMMs(long ticks, float dt) {
  if (dt <= 0) return 0;
  float revs = ticks / ENCODER_CPR;
  return (revs * WHEEL_CIRCUMF_MM) / dt;
}

// Reads the HC-SR04: 10us trigger pulse out, then times the echo's HIGH
// duration and converts to millimeters using the speed of sound
// (~343 m/s => 0.1715mm per microsecond of round-trip time). Returns -1
// if no echo comes back within ECHO_TIMEOUT_US (out of range, or nothing
// reflecting back to the sensor).
float measureFrontDistanceMM() {
  digitalWrite(BUMPER_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(BUMPER_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(BUMPER_TRIG, LOW);

  unsigned long duration = pulseIn(BUMPER_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;
  return duration * 0.1715;
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
  // target) AND the wheel is still near-stalled (see
  // DEADBAND_RECOVERY_SPEED_MMS above). Otherwise a tiny residual-noise
  // correction while trying to sit at target=0 would get blown up into a
  // full-strength drive command, and - the bug this guard fixes - an
  // ordinary small correction while already moving would too. See
  // STABILITY FIXES #4 above.
  int offset = pwm - PWM_STOP;
  if (fabs(target) > 0.1 && fabs(measured) < DEADBAND_RECOVERY_SPEED_MMS &&
      offset != 0 && abs(offset) < MIN_RELIABLE_PWM_OFFSET) {
    pwm = PWM_STOP + (offset > 0 ? MIN_RELIABLE_PWM_OFFSET : -MIN_RELIABLE_PWM_OFFSET);
  }

  return constrain(pwm, PWM_MIN, PWM_MAX);
}

// Caps how much the written PWM can change in one control cycle,
// regardless of what computePID/applySlipCorrection want - see
// STABILITY FIXES #3 above.
//
// Exception: if we were genuinely AT REST (lastApplied exactly at
// PWM_STOP, not just "somewhere near the dead zone") and the target has
// already jumped past the dead zone (via computePID()'s deadband
// compensation), jump straight to the FULL computed target on this one
// cycle instead of creeping through it a MAX_PWM_STEP_PER_CYCLE at a
// time - slewing gradually through 0-100us offset just spends several
// cycles producing the unpredictable twitching that zone is known for
// (see MIN_RELIABLE_PWM_OFFSET) before reaching reliable torque. Using
// the full target rather than capping at the dead zone's bare-minimum
// edge matters in practice: a pivot turn on the ground needs more
// torque to break real static/scrub friction than bench testing (which
// calibrated MIN_RELIABLE_PWM_OFFSET) accounted for, and computePID()
// already reflects that in its raw output once the commanded turn/speed
// is large enough - landing only at the 100us floor and then crawling
// the rest of the way at 20us/cycle was both sticky (undershoots what's
// needed to break loose) and slow (many cycles to get there). Safe to
// jump straight to rawPWM because computePID() already clamps it to
// PWM_MIN/PWM_MAX, and because of the constraint below, this can still
// only ever fire once per stop, not repeatedly.
//
// IMPORTANT: this must require exact equality to PWM_STOP, not "close
// to it" - an earlier version used abs(lastOffset) < MIN_RELIABLE_PWM_OFFSET,
// which could also fire while lastApplied was mid-transition near the
// zone's edge with a noisy/still-decelerating raw target on the
// OPPOSITE side, causing repeated ~180us instant jumps back and forth
// across the zone every cycle - a self-sustaining oscillation that
// bypassed the slew limit entirely and didn't stop even at target=0
// (computePID doesn't deadband-compensate a zero target, so residual
// momentum/noise could still swing the raw PID output wildly). Exact
// equality means this can only fire once, on the very first cycle
// moving away from a true stop.
int slewLimit(int rawPWM, int &lastApplied) {
  int targetOffset = rawPWM - PWM_STOP;
  if (lastApplied == PWM_STOP && abs(targetOffset) >= MIN_RELIABLE_PWM_OFFSET) {
    // REVERTED from jumping straight to the full rawPWM target - that
    // made on-ground pivot turns worse, not better, likely because a
    // bigger single-cycle torque kick exceeded the wheel's maximum
    // static grip and caused it to skid/spin in place instead of
    // actually turning the chassis (kinetic friction while slipping is
    // generally lower than static, so "more torque" doesn't reliably
    // mean "turns faster" once you're past that limit) - or induced the
    // same kind of noise the slew limiter exists to prevent. Back to
    // capping the one-time jump at the dead zone's edge.
    lastApplied = PWM_STOP + (targetOffset > 0 ? MIN_RELIABLE_PWM_OFFSET : -MIN_RELIABLE_PWM_OFFSET);
    return constrain(lastApplied, PWM_MIN, PWM_MAX);
  }

  int applied = constrain(rawPWM, lastApplied - MAX_PWM_STEP_PER_CYCLE,
                                    lastApplied + MAX_PWM_STEP_PER_CYCLE);
  applied = constrain(applied, PWM_MIN, PWM_MAX);
  lastApplied = applied;
  return applied;
}
