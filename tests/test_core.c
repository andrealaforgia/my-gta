#include <assert.h>
#include <string.h>

#include "config.h"
#include "frame_clock.h"
#include "input.h"
#include "test_groups.h"
#include "test_support.h"

/* ---- frame clock ---- */

static void one_frame_at_sixty_yields_two_steps_at_one_twenty(void) {
  frame_clock_t clock = frame_clock_create(1.0 / 120.0, 0.25);
  int steps = frame_clock_advance(&clock, 1.0 / 60.0);
  assert(steps == 2);
  assert(clock.alpha >= 0 && clock.alpha < 0.01);
}

static void leftover_time_carries_into_the_next_frame(void) {
  frame_clock_t clock = frame_clock_create(1.0 / 120.0, 0.25);
  assert(frame_clock_advance(&clock, 0.005) == 0);
  assert(clock.alpha > 0.59 && clock.alpha < 0.61);
  assert(frame_clock_advance(&clock, 0.005) == 1);
}

static void a_stall_is_capped_so_catch_up_cannot_spiral(void) {
  frame_clock_t clock = frame_clock_create(1.0 / 120.0, 0.25);
  int steps = frame_clock_advance(&clock, 5.0);
  assert(steps == 30);
  assert(frame_clock_advance(&clock, 0.0) == 0);
}

/* ---- config ---- */

static void config_reads_keys_and_ignores_comments_and_blanks(void) {
  const char* text =
      "# vehicle\n"
      "mass = 1200\n"
      "\n"
      "name=hatchback   \n"
      "  wheel_radius = 0.33 # metres\n"
      "front_drive = true\n";
  config_t config = config_parse(text);
  assert(config_count(&config) == 4);
  assert(near(config_get_float(&config, "mass", 0), 1200));
  assert(near(config_get_float(&config, "wheel_radius", 0), 0.33));
  assert(strcmp(config_get_string(&config, "name", ""), "hatchback") == 0);
  assert(config_get_bool(&config, "front_drive", false));
  assert(config_has(&config, "mass"));
  assert(!config_has(&config, "colour"));
  config_free(&config);
}

static void config_falls_back_to_defaults_and_records_it(void) {
  config_t config = config_parse("mass = 1200\n");
  assert(near(config_get_float(&config, "drag", 0.4f), 0.4f));
  assert(config_get_int(&config, "gears", 5) == 5);
  assert(config_defaulted_count(&config) == 2);
  config_free(&config);
}

static void config_reports_malformed_lines(void) {
  config_t config = config_parse("this line has no equals\nmass = 1200\n");
  assert(config_count(&config) == 1);
  assert(config_malformed_count(&config) == 1);
  config_free(&config);
}

static void config_survives_a_missing_file(void) {
  config_t config;
  assert(!config_load(&config, "tests/fixtures/does-not-exist.cfg"));
  config_free(&config);
}

/* ---- input ---- */

static void dead_zone_removes_stick_drift_and_rescales(void) {
  assert(near(apply_dead_zone(0.05f, 0.15f), 0));
  assert(near(apply_dead_zone(1.0f, 0.15f), 1.0f));
  assert(near(apply_dead_zone(-1.0f, 0.15f), -1.0f));
  float half = apply_dead_zone(0.575f, 0.15f);
  assert(half > 0.49f && half < 0.51f);
}

static void keyboard_maps_arrows_to_driving_actions(void) {
  unsigned char keys[512] = {0};
  keys[INPUT_SCANCODE_UP] = 1;
  keys[INPUT_SCANCODE_LEFT] = 1;
  keys[INPUT_SCANCODE_SPACE] = 1;
  driving_input_t input = driving_input_from_keys(keys);
  assert(near(input.throttle, 1) && near(input.brake, 0));
  assert(near(input.steering, -1));
  assert(input.handbrake);
  keys[INPUT_SCANCODE_LEFT] = 0;
  keys[INPUT_SCANCODE_RIGHT] = 1;
  keys[INPUT_SCANCODE_UP] = 0;
  keys[INPUT_SCANCODE_DOWN] = 1;
  input = driving_input_from_keys(keys);
  assert(near(input.steering, 1) && near(input.brake, 1) &&
         near(input.throttle, 0));
}

static void controller_axes_combine_with_keyboard_by_largest_magnitude(void) {
  driving_input_t keyboard = {0};
  keyboard.steering = -0.2f;
  driving_input_t pad = {0};
  pad.steering = 0.9f;
  pad.throttle = 0.5f;
  driving_input_t merged = driving_input_merge(keyboard, pad);
  assert(near(merged.steering, 0.9f) && near(merged.throttle, 0.5f));
}

static void keyboard_steering_is_smoothed_and_speed_sensitive(void) {
  steering_filter_t filter = steering_filter_create(8.0f, 28.0f);
  float angle = 0;
  for (int i = 0; i < 5; i++)
    angle = steering_filter_update(&filter, 1.0f, 0.0f, 1.0f / 60.0f);
  assert(angle > 0.1f && angle < 0.9f);
  for (int i = 0; i < 200; i++)
    angle = steering_filter_update(&filter, 1.0f, 0.0f, 1.0f / 60.0f);
  assert(near_to(angle, 1.0f, 1e-3));
  steering_filter_t fast = steering_filter_create(8.0f, 28.0f);
  float at_speed = 0;
  for (int i = 0; i < 200; i++)
    at_speed = steering_filter_update(&fast, 1.0f, 40.0f, 1.0f / 60.0f);
  assert(at_speed < 0.5f && at_speed > 0.05f);
}

void run_core_tests(void) {
  RUN(one_frame_at_sixty_yields_two_steps_at_one_twenty);
  RUN(leftover_time_carries_into_the_next_frame);
  RUN(a_stall_is_capped_so_catch_up_cannot_spiral);
  RUN(config_reads_keys_and_ignores_comments_and_blanks);
  RUN(config_falls_back_to_defaults_and_records_it);
  RUN(config_reports_malformed_lines);
  RUN(config_survives_a_missing_file);
  RUN(dead_zone_removes_stick_drift_and_rescales);
  RUN(keyboard_maps_arrows_to_driving_actions);
  RUN(controller_axes_combine_with_keyboard_by_largest_magnitude);
  RUN(keyboard_steering_is_smoothed_and_speed_sensitive);
}
