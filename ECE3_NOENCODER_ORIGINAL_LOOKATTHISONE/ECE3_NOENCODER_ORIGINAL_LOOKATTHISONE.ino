#include <ECE3.h>

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int status_led_pin = 41;

const int sMin[8] = {818, 721, 612, 635, 644, 702, 703, 837};
const int sMax[8] = {1682, 1779, 1888, 1180, 1404, 1276, 1797, 1663}; // spans after subtracting sMin
const int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

// PD tuning
const int base_speed = 80;
const float Kp = 0.09f;
const float Kd = 0.48f;
const int error_offset = 290;
const int steer_dir = 1;

const int lost_threshold = 300;

// U-turn: check these on our car.
const int turn_speed = 240;
const unsigned long turn_ms = 270;
const int bar_level = 700;
const int bar_confirm = 3;
const unsigned long bar_ignore_ms = 500;

uint16_t sensorValues[8];
int normValues[8];
int prev_error = 0;
bool reset_pd = true;
bool returning = false;
bool finished = false;
bool bar_armed = false;
int bar_samples = 0;
int clear_samples = 0;
unsigned long last_bar_time = 0;

void setMotors(int left_spd, int right_spd) {
  digitalWrite(left_dir_pin, left_spd < 0 ? HIGH : LOW);
  digitalWrite(right_dir_pin, right_spd < 0 ? HIGH : LOW);
  analogWrite(left_pwm_pin, constrain(abs(left_spd), 0, 255));
  analogWrite(right_pwm_pin, constrain(abs(right_spd), 0, 255));
}

void stopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void normalizeSensors() {
  for (int i = 0; i < 8; i++) {
    long n = ((long)sensorValues[i] - sMin[i]) * 1000L / sMax[i];
    normValues[i] = (int)constrain(n, 0L, 1000L);
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
  for (int i = 0; i < 8; i++) sum += (long)weights[i] * normValues[i];
  return (int)(sum / 4) - error_offset;
}

bool crossbarDetected() {
  int dark_sensors = 0;
  for (int i = 0; i < 8; i++) {
    if (normValues[i] > bar_level) dark_sensors++;
  }

  // Arm only after leaving the starting/turnaround bar.
  if (!bar_armed) {
    if (dark_sensors < 8 && lineDetected() &&
        millis() - last_bar_time >= bar_ignore_ms) {
      clear_samples++;
    } else {
      clear_samples = 0;
    }
    if (clear_samples >= bar_confirm) bar_armed = true;
    return false;
  }

  if (dark_sensors == 8) bar_samples++;
  else bar_samples = 0;
  return bar_samples >= bar_confirm;
}

void makeUTurn() {
  stopMotors();
  delay(40);
  setMotors(-turn_speed, turn_speed);
  delay(turn_ms);
  stopMotors();
  returning = true;
  prev_error = 0;
  reset_pd = true;
}

void finishRun() {
  stopMotors();
  digitalWrite(left_nslp_pin, LOW);
  digitalWrite(right_nslp_pin, LOW);
  digitalWrite(status_led_pin, HIGH);
  finished = true;
}

void setup() {
  ECE3_Init();
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(status_led_pin, OUTPUT);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(status_led_pin, LOW);
  stopMotors();
  delay(2000);
  last_bar_time = millis();
}

void loop() {
  if (finished) return;

  ECE3_read_IR(sensorValues);
  normalizeSensors();

  if (crossbarDetected()) {
    bar_armed = false;
    bar_samples = 0;
    clear_samples = 0;
    if (returning) finishRun();
    else makeUTurn();
    last_bar_time = millis();
    return;
  }
  if (bar_samples > 0) return;

  if (!lineDetected()) {
    stopMotors();
    reset_pd = true;
    return;
  }

  int error = fuseSensors();
  if (reset_pd) prev_error = error;
  reset_pd = false;

  int correction = (int)(steer_dir * (Kp * error + Kd * (error - prev_error)));
  prev_error = error;
  setMotors(base_speed - correction, base_speed + correction);
}