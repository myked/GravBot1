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
  Sabertooth S1 (left side)  -> pin 9  (PWM out from Arduino)
  Sabertooth S2 (right side) -> pin 10 (PWM out from Arduino)
  Arduino GND -> Sabertooth 0V

  HC-05 Bluetooth (wireless tuning from Android):
  HC-05 VCC -> Uno 5V
  HC-05 GND -> Uno GND
  HC-05 TX  -> Uno pin 8   (direct - 3.3V out is fine into Uno RX)
  HC-05 RX  -> Uno pin 13  (THROUGH a 1k/2k voltage divider - Uno's 5V
                            TX would over-drive the HC-05's 3.3V RX pin)
  Uses SoftwareSerial on pins 8/13 so the hardware Serial port (0/1)
  stays free for USB - upload and USB debugging are unaffected, and
  tuning commands/telemetry work over either USB or Bluetooth.
  Default HC-05 baud is 9600 unless you've reconfigured it via AT
  commands - matches BT_BAUD below.

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
#include <SoftwareSerial.h>

// Set to 1 once the HC-05 module is wired up and ready to use.
// While 0, all Bluetooth code is skipped - tuning/telemetry only over USB.
#define ENABLE_BLUETOOTH 0

// ---------------- USER CONFIG ----------------

const float ENCODER_CPR       = 400.0;   // counts per motor-output-shaft rev
const float WHEEL_DIAMETER_MM = 120.0;   // confirm with calipers
const float WHEEL_CIRCUMF_MM  = 3.14159265 * WHEEL_DIAMETER_MM;

// Target crawl speed, mm/s. Positive = forward, negative = reverse.
float targetSpeedMMs = 30.0;

// PID gains - start conservative, tune on your actual robot
float Kp = 0.9;
float Ki = 0.4;
float Kd = 0.02;

// Sabertooth PWM output limits (microseconds), 1500 = stop
const int PWM_STOP = 1500;
const int PWM_MIN  = 1000;
const int PWM_MAX  = 2000;

// Minimum "kick" offset (us) to overcome stiction when starting from rest
const int STICTION_KICK = 40;

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

// ---------------- PIN CONFIG ----------------

const int LF_A = 2,  LF_B = 4;   // left front  (hardware interrupt)
const int RF_A = 3,  RF_B = 5;   // right front (hardware interrupt)
const int LR_A = 11, LR_B = 6;   // left rear   (pin-change interrupt)
const int RR_A = 12, RR_B = 7;   // right rear  (pin-change interrupt)

// HC-05 Bluetooth module (wireless tuning), on SoftwareSerial so USB
// Serial (pins 0/1) stays free for uploads and USB debugging.
const int BT_RX_PIN = 8;   // to HC-05 TX
const int BT_TX_PIN = 13;  // to HC-05 RX (via voltage divider)
const long BT_BAUD = 9600; // HC-05 default; update if you've reconfigured it

// ---------------- INTERNAL STATE ----------------

Servo saberLeft;
Servo saberRight;

#if ENABLE_BLUETOOTH
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);
#endif

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

// ---------------- FUNCTION PROTOTYPES ----------------
// Declared here so they're known before use in loop(), rather than
// relying on the Arduino build step to auto-generate prototypes.

float ticksToMMs(long ticks, float dt);
int computePID(PID &state, float target, float measured, float dt);
int applySlipCorrection(int pwm, float frontMMs, float rearMMs);
void printBoth(const String &line);

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(9600);
#if ENABLE_BLUETOOTH
  bluetooth.begin(BT_BAUD);
#endif

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

  saberLeft.attach(9);
  saberRight.attach(10);
  saberLeft.writeMicroseconds(PWM_STOP);
  saberRight.writeMicroseconds(PWM_STOP);
  delay(500);

  printBoth("Ready. Send a number to set target speed in mm/s (negative = reverse).");
  lastControlTime = millis();
}

// ---------------- CONTROL LOOP ----------------

void loop() {
  if (Serial.available()) {
    targetSpeedMMs = Serial.parseFloat();
    printBoth("New target speed (mm/s): " + String(targetSpeedMMs));
  }
#if ENABLE_BLUETOOTH
  if (bluetooth.available()) {
    targetSpeedMMs = bluetooth.parseFloat();
    printBoth("New target speed (mm/s): " + String(targetSpeedMMs));
  }
#endif

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    noInterrupts();
    long lf = lfTicks, rf = rfTicks, lr = lrTicks, rr = rrTicks;
    lfTicks = rfTicks = lrTicks = rrTicks = 0;
    interrupts();

    float leftFrontMMs  = ticksToMMs(lf, dt);
    float leftRearMMs   = ticksToMMs(lr, dt);
    float rightFrontMMs = ticksToMMs(rf, dt);
    float rightRearMMs  = ticksToMMs(rr, dt);

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

    saberLeft.writeMicroseconds(leftPWM);
    saberRight.writeMicroseconds(rightPWM);

    String line = "tgt:" + String(targetSpeedMMs) +
                  " LF:" + String(leftFrontMMs) +
                  " LR:" + String(leftRearMMs) +
                  " RF:" + String(rightFrontMMs) +
                  " RR:" + String(rightRearMMs) +
                  " Lpwm:" + String(leftPWM) +
                  " Rpwm:" + String(rightPWM);
    if (leftSlip)  line += " [LEFT SLIP]";
    if (rightSlip) line += " [RIGHT SLIP]";
    printBoth(line);
  }
}

// ---------------- HELPERS ----------------

void printBoth(const String &line) {
  Serial.println(line);
#if ENABLE_BLUETOOTH
  bluetooth.println(line);
#endif
}

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

  return constrain(pwm, PWM_MIN, PWM_MAX);
}
