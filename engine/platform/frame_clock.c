#include "frame_clock.h"

frame_clock_t frame_clock_create(double step_seconds,
                                 double max_catch_up_seconds) {
  frame_clock_t clock = {step_seconds, max_catch_up_seconds, 0.0, 0.0};
  return clock;
}

int frame_clock_advance(frame_clock_t* clock, double frame_seconds) {
  if (frame_seconds < 0) frame_seconds = 0;
  if (frame_seconds > clock->max_catch_up) frame_seconds = clock->max_catch_up;
  clock->accumulator += frame_seconds;
  int steps = 0;
  while (clock->accumulator + 1e-9 >= clock->step) {
    clock->accumulator -= clock->step;
    if (clock->accumulator < 0) clock->accumulator = 0;
    steps++;
  }
  clock->alpha = clock->accumulator / clock->step;
  return steps;
}
