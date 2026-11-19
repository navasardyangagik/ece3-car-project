// ECE 3 — Line Follower (sensor fusion + PD steering)
//
// How it works, every loop:
//   1. Read the 8 IR sensors (raw ~600-2500).
//   2. Normalize each to 0-1000 using our calibration (min / max below).
//   3. Fuse all 8 into ONE error value with the (8-4-2-1)/4 weights.
//      negative = line toward S1 side, positive = line toward S8 side.
//   4. Steer using error and its change since the previous loop (PD).
//   5. If the line is lost, briefly turn toward its last seen side, then stop.
//
// Calibration constants come from sensor_fusion.xlsx; normalization and
// steering are computed on the car.

#include <ECE3.h>

// ---------------- Pin Assignments ----------------
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ---------------- Calibration (from sensor_fusion.xlsx) ----------------
const int sMin[8] = {818, 721, 612, 635, 644, 702, 703, 837};    // B24:I24, "Minimum"/Users/gagik/Desktop/ece3-car-project/ECE3_NOENCODER_ORIGINAL_LOOKATTHISON
const int sMax[8] = {1682, 1779, 1888, 1180, 1404, 1276, 1797, 1663}; // K24:R24, max after min subtracted

// Fusion weights: (8-4-2-1)/4
const int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
const int weight_div = 4;

// ---------------- Tuning (change these while testing) ----------------
const int   base_speed = 70;      // PWM 0-255. Start slow.
const float Kp = 0.05;           // error 2000 -> 100 PWM proportional correction
const float Kd = 0.035;
const int   error_offset = 290;   // AC13: fused value at the sheet's "0" position
const int   lost_threshold = 300; // if NO sensor is above this, the line is lost
const int   rebound_speed = 70;   // equal forward/reverse PWM during recovery
const unsigned long rebound_ms = 250; // stop if the line is still missing

// If the car turns AWAY from the line instead of toward it, change this to -1.
const int steer_dir = 1;

uint16_t sensorValues[8];
int normValues[8];
int prev_error = 0;
unsigned long last_line_time = 0;
bool line_was_lost = true;

void setMotors(int left_spd, int right_spd) {
  if (left_spd < 0) {
    digitalWrite(left_dir_pin, HIGH);
    left_spd = -1 * left_spd;
  } else {
    digitalWrite(left_dir_pin, LOW);
  }
  if (right_spd < 0) {
    digitalWrite(right_dir_pin, HIGH);
    right_spd = -1 * right_spd;
  } else {
    digitalWrite(right_dir_pin, LOW);
  }
  left_spd  = constrain(left_spd, 0, 255);
  right_spd = constrain(right_spd, 0, 255);
  analogWrite(left_pwm_pin, left_spd);
  analogWrite(right_pwm_pin, right_spd);
}

void stopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void normalizeSensors() {
  for (int i = 0; i < 8; i++) {
    int n = ((int)sensorValues[i] - sMin[i]) * 1000 / sMax[i];
    normValues[i] = constrain(n, 0, 1000);
  }
}

bool lineDetected() {
  for (int i = 0; i < 8; i++) {
    if (normValues[i] > lost_threshold) return true;
  }
  return false;
}

int fuseSensors() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += (long)weights[i] * normValues[i];
  }
  return (int)(sum / weight_div) - error_offset;
}

// ---------------- Setup ----------------
void setup() {
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);
  stopMotors();

  Serial.begin(9600);
  delay(2000);   // 2 s to set the car on the line
}

// ---------------- Main Loop ----------------
void loop() {
  ECE3_read_IR(sensorValues);
  normalizeSensors();

  if (!lineDetected()) {
    line_was_lost = true;
    if (prev_error != 0 && millis() - last_line_time < rebound_ms) {
      // Keep turning toward the last seen line, around the car's middle.
      int turn = steer_dir * (prev_error > 0 ? rebound_speed : -rebound_speed);
      setMotors(-turn, turn);
    } else {
      stopMotors();
      prev_error = 0;
    }
    return;
  }

  int error = fuseSensors();
  last_line_time = millis();
  // Start fresh after recovery so the derivative does not jerk the wheels.
  if (line_was_lost) prev_error = error;
  line_was_lost = false;

  int correction = steer_dir * (Kp * error + Kd * (error - prev_error));
  prev_error = error;

  // Positive correction slows/reverses the left wheel and speeds up the right.
  // This keeps the existing steering direction; steer_dir flips it if needed.
  int left_spd  = base_speed - correction;
  int right_spd = base_speed + correction;
  setMotors(left_spd, right_spd);
}
