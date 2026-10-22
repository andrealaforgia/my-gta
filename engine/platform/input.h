/**
 * @file input.h
 * @brief Driving actions from keyboard and controller. The mapping and
 * filtering are plain functions so they are tested without a window; the
 * SDL polling lives in window.c.
 */
#ifndef ENGINE_PLATFORM_INPUT_H_
#define ENGINE_PLATFORM_INPUT_H_

#include <stdbool.h>

/* SDL scancodes, repeated here so the mapping has no SDL dependency. */
enum {
  INPUT_SCANCODE_SPACE = 44,
  INPUT_SCANCODE_LCTRL = 224,
  INPUT_SCANCODE_LSHIFT = 225,
  INPUT_SCANCODE_RSHIFT = 229,
  INPUT_SCANCODE_RIGHT = 79,
  INPUT_SCANCODE_LEFT = 80,
  INPUT_SCANCODE_DOWN = 81,
  INPUT_SCANCODE_UP = 82,
  INPUT_SCANCODE_W = 26,
  INPUT_SCANCODE_A = 4,
  INPUT_SCANCODE_S = 22,
  INPUT_SCANCODE_D = 7,
};

typedef struct {
  float throttle; /* 0..1 */
  float brake;    /* 0..1 */
  float steering; /* -1 left .. 1 right */
  bool handbrake;
  bool lower; /* Ctrl: takes power off an aircraft, brakes it on the ground */
} driving_input_t;

driving_input_t driving_input_from_keys(const unsigned char* keys);
/* Per-channel: the larger magnitude wins, so pad and keyboard coexist. */
driving_input_t driving_input_merge(driving_input_t a, driving_input_t b);
float apply_dead_zone(float value, float dead_zone);

/* Smooths a -1..1 steering demand towards its target at a fixed rate and
 * scales the reachable angle down with speed. */
typedef struct {
  float rate;            /* approach rate per second */
  float half_lock_speed; /* m/s at which the reachable lock has halved */
  float value;
} steering_filter_t;

steering_filter_t steering_filter_create(float rate, float half_lock_speed);
float steering_filter_update(steering_filter_t* filter, float demand,
                             float speed_mps, float dt);

#endif  // ENGINE_PLATFORM_INPUT_H_
