// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

const int marker_frequencies[] = {
    3,
    4,
    5,
    6,
};
const int marker_pins[] = {
    24,
    52,
    13,
    56,
};
const size_t marker_count = 4;
const int ground_pins[] = {
    53,
    25,
    55,
};
const size_t ground_pin_count = 3;

/*
const uint16_t marker_frequencies[] = {
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
};
const int marker_pins[] = {
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
};
const size_t marker_count = 13;
const int ground_pins[] = {
};
const size_t ground_pin_count = 0;
*/

uint8_t marker_states[marker_count];
uint16_t marker_times[marker_count];

void setup() {
  for (size_t i = 0; i < marker_count; i++) {
    pinMode(marker_pins[i], OUTPUT);
    digitalWrite(marker_pins[i], HIGH);
  }
  for (size_t i = 0; i < ground_pin_count; i++) {
    pinMode(ground_pins[i], OUTPUT);
    digitalWrite(ground_pins[i], LOW);
  }
}

void loop() {
  /*uint32_t t = micros();
  //bool pwm = (((t >> 10) & 1) == 0);
  bool pwm = 0;
  for (size_t i = 0; i < marker_count; i++) {
    int p = marker_pins[i];
    int v = ((t * marker_frequencies[i] * 2 / (1000 * 1000)) & 1);
    v |= pwm;
    digitalWrite(p, v);
  }*/

  /*uint16_t t = millis();
  static uint16_t lt = 0;
  uint16_t dt = t - lt;
  if(dt > 100) dt = 100;
  lt = t;

  for (size_t i = 0; i < marker_count; i++) {
    marker_times[i] += dt * marker_frequencies[i];
    if(marker_times[i] > (uint16_t)500) {
      marker_times[i] = 0;
      marker_states[i] = !marker_states;
      digitalWrite(marker_pins[i], marker_states[i]);
    }
  }*/

  static uint8_t pwm = 0;
  pwm = ((pwm + 1) & 3);
  uint32_t t = millis();
  for (size_t i = 0; i < marker_count; i++) {
    digitalWrite(marker_pins[i], ((t * marker_frequencies[i]) / 500) & 1 || !pwm);
  }
}





