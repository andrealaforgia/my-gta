#include "interpolation.h"

#include <math.h>

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

float clampf(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

float move_towards(float value, float target, float max_step) {
  float gap = target - value;
  if (fabsf(gap) <= max_step) return target;
  return value + (gap > 0 ? max_step : -max_step);
}

float damp_towards(float value, float target, float rate, float dt) {
  return lerpf(value, target, 1.0f - expf(-rate * dt));
}

float signf(float value) {
  return value > 0 ? 1.0f : (value < 0 ? -1.0f : 0.0f);
}
