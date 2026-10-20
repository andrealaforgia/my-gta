#include "input.h"

#include <math.h>

#include "interpolation.h"

driving_input_t driving_input_from_keys(const unsigned char* keys) {
  driving_input_t input = {0, 0, 0, false};
  if (keys[INPUT_SCANCODE_UP] || keys[INPUT_SCANCODE_W]) input.throttle = 1;
  if (keys[INPUT_SCANCODE_DOWN] || keys[INPUT_SCANCODE_S]) input.brake = 1;
  if (keys[INPUT_SCANCODE_LEFT] || keys[INPUT_SCANCODE_A]) input.steering -= 1;
  if (keys[INPUT_SCANCODE_RIGHT] || keys[INPUT_SCANCODE_D]) input.steering += 1;
  input.handbrake = keys[INPUT_SCANCODE_SPACE] != 0;
  return input;
}

static float larger_magnitude(float a, float b) {
  return fabsf(a) >= fabsf(b) ? a : b;
}

driving_input_t driving_input_merge(driving_input_t a, driving_input_t b) {
  driving_input_t merged;
  merged.throttle = larger_magnitude(a.throttle, b.throttle);
  merged.brake = larger_magnitude(a.brake, b.brake);
  merged.steering = larger_magnitude(a.steering, b.steering);
  merged.handbrake = a.handbrake || b.handbrake;
  return merged;
}

float apply_dead_zone(float value, float dead_zone) {
  float magnitude = fabsf(value);
  if (magnitude <= dead_zone) return 0;
  float rescaled = (magnitude - dead_zone) / (1.0f - dead_zone);
  return clampf(rescaled, 0, 1) * signf(value);
}

steering_filter_t steering_filter_create(float rate, float half_lock_speed) {
  steering_filter_t filter = {rate, half_lock_speed, 0};
  return filter;
}

float steering_filter_update(steering_filter_t* filter, float demand,
                             float speed_mps, float dt) {
  /* Full lock at rest, half at half_lock_speed, a quarter at twice it. */
  float ratio = speed_mps / filter->half_lock_speed;
  float speed_factor = 1.0f / (1.0f + ratio * ratio);
  float target = clampf(demand, -1, 1) * speed_factor;
  filter->value = damp_towards(filter->value, target, filter->rate, dt);
  return filter->value;
}
