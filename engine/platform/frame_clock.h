/**
 * @file frame_clock.h
 * @brief Fixed-step scheduling. Rendering runs whenever it can; the
 * simulation advances in steps of a fixed length and catch-up after a
 * stall is capped so a long frame cannot spiral into longer ones.
 */
#ifndef ENGINE_PLATFORM_FRAME_CLOCK_H_
#define ENGINE_PLATFORM_FRAME_CLOCK_H_

typedef struct {
  double step;
  double max_catch_up;
  double accumulator;
  double alpha; /* fraction of a step elapsed since the last simulated one */
} frame_clock_t;

frame_clock_t frame_clock_create(double step_seconds, double max_catch_up_seconds);
/* Returns how many fixed steps to simulate for this frame. */
int frame_clock_advance(frame_clock_t* clock, double frame_seconds);

#endif  // ENGINE_PLATFORM_FRAME_CLOCK_H_
