/*
  TEST -- HC-SR04 ultrasonic sensor only. No encoders, no motor/Sabertooth
  code touched. Confirms the sensor is wired correctly and returning
  sane distances before relying on it from the main sketch.

  GOAL
  ----
  Open the serial monitor at 9600 baud. With nothing in front of the
  sensor, you should see a steady stream of distance readings that
  increase as you back away and decrease as you approach, roughly
  matching a tape measure. "no echo" means out of range (or nothing
  reflecting straight back) - not a fault by itself unless it happens
  with a target directly in front within ~1m.

  WIRING (unchanged from the full sketch)
  ------
  HC-SR04 VCC  -> Uno 5V
  HC-SR04 GND  -> Uno GND
  HC-SR04 TRIG -> Uno pin 8  (output, 10us trigger pulse)
  HC-SR04 ECHO -> Uno pin A0 (input, pulseIn() measures echo width)
*/

#include <Arduino.h>

const int TRIG_PIN = 8;
const int ECHO_PIN = A0;

const unsigned long SAMPLE_PERIOD_MS = 200;   // slower, easier to read by eye
const unsigned long ECHO_TIMEOUT_US  = 25000; // ~4.3m max range

unsigned long lastSampleTime = 0;

// Reads the HC-SR04: 10us trigger pulse out, then times the echo's HIGH
// pulse width. Returns distance in mm, or -1 if no echo comes back within
// ECHO_TIMEOUT_US (out of range, or nothing to reflect off).
float readDistanceMM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;

  // speed of sound ~343 m/s at room temp -> 0.343 mm/us, round trip so /2
  return (duration * 0.343f) / 2.0f;
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.println("TEST: HC-SR04 ultrasonic sensor only.");
  Serial.println("Readings in mm, -1 = no echo / out of range.");
  lastSampleTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_PERIOD_MS) {
    lastSampleTime = now;

    float mm = readDistanceMM();
    if (mm < 0) {
      Serial.println("front: no echo");
    } else {
      Serial.println("front:" + String(mm, 0) + "mm");
    }
  }
}
