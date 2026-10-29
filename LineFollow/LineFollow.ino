// ECE 3 — Line Follower (sensor fusion + proportional steering)
//
// How it works, every loop:
//   1. Read the 8 IR sensors (raw ~600-2500).
//   2. Normalize each to 0-1000 using our calibration (min / max below).
//   3. Fuse all 8 into ONE error value with the (8-4-2-1)/4 weights.
//      negative = line toward S1 side, positive = line toward S8 side.
//   4. Steer: one wheel speeds up, the other slows down, in proportion to error.
//   5. If no sensor sees black (line lost), stop.
//
// All calibration values are hardcoded from sensor_fusion.xlsx — nothing is
// computed on the car.

#include <ECE3.h>

// ---------------- Pin Assignments ----------------
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ---------------- Calibration (from sensor_fusion.xlsx) ----------------
const int sMin[8] = {818, 721, 612, 635, 644, 702, 703, 837};    // "Minimum" row
const int sMax[8] = {1682, 1779, 1888, 1180, 1404, 1276, 1797, 1663}; // "Maximum" row (after min subtracted)

// Fusion weights: (8-4-2-1)/4
const int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
const int weight_div = 4;

// ---------------- Tuning (change these while testing) ----------------
const int   base_speed = 40;      // PWM 0-255. Start slow.
const float Kp = 0.015;           // steering strength. error ~2000 -> 30 PWM of correction
const float Kd = 0.0;             // leave 0 until P alone works, then try ~0.05
const int   error_offset = 0;     // fused value when car is centered on the line
                                  // (our data reads ~ +290 at the "0" position)
const int   lost_threshold = 300; // if NO sensor is above this, the line is lost -> stop

// If the car turns AWAY from the line instead of toward it, change this to -1.
const int steer_dir = 1;

const bool debug = false;         // true = print values over Serial (slows the loop)

// ---------------- Globals ----------------
uint16_t sensorValues[8];
int normValues[8];
int prev_error = 0;

// ---------------- Motor Helpers ----------------
void setMotors(int left_spd, int right_spd) {
  // forward only, clamp to valid PWM range
  left_spd  = constrain(left_spd, 0, 255);
  right_spd = constrain(right_spd, 0, 255);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  analogWrite(left_pwm_pin, left_spd);
  analogWrite(right_pwm_pin, right_spd);
}

void stopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

// ---------------- Sensor Processing ----------------
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
    stopMotors();
    prev_error = 0;
    if (debug) Serial.println("LINE LOST - stopped");
    return;
  }

  int error = fuseSensors();
  int correction = steer_dir * (Kp * error + Kd * (error - prev_error));
  prev_error = error;

  // Line toward S8 side (positive error) -> turn toward S8 side.
  // If S8 is on the car's right, that means: left wheel faster, right wheel slower.
  int left_spd  = base_speed + correction;
  int right_spd = base_speed - correction;
  setMotors(left_spd, right_spd);

  if (debug) {
    Serial.print("err: ");  Serial.print(error);
    Serial.print("  L: ");  Serial.print(left_spd);
    Serial.print("  R: ");  Serial.println(right_spd);
  }
}
