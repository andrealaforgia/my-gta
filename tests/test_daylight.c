#include <assert.h>
#include <math.h>

#include "daylight.h"
#include "test_groups.h"
#include "test_support.h"

static float luminance(vec3_t c) {
  return 0.2f * c.x + 0.7f * c.y + 0.1f * c.z;
}

static void noon_is_bright_with_the_sun_overhead(void) {
  render_environment_t e = daylight_environment(12.0f);
  assert(vec3_normalize(e.direction).y > 0.9f);
  assert(e.ambient > 0.25f);
  assert(e.star_level < 0.01f && e.sun_disc > 0.9f);
  assert(luminance(e.sky_color) > 0.5f);
  assert(daylight_lamp_glow(12.0f) < 0.01f);
}

static void midnight_is_dark_with_stars_and_lit_lamps(void) {
  render_environment_t e = daylight_environment(0.0f);
  assert(e.ambient < 0.14f && e.ambient > 0.02f);
  assert(e.star_level > 0.9f);
  assert(luminance(e.sky_color) < 0.1f);
  assert(luminance(e.zenith_color) < 0.06f);
  assert(daylight_lamp_glow(0.0f) > 0.9f);
  assert(e.sun_disc < 0.1f);
  /* The light still points down from above: the moon. */
  assert(vec3_normalize(e.direction).y > 0.3f);
}

static void the_sun_rises_in_the_east_and_sets_in_the_west(void) {
  assert(vec3_normalize(daylight_environment(8.0f).direction).x > 0.2f);
  assert(vec3_normalize(daylight_environment(16.0f).direction).x < -0.2f);
}

static void dawn_light_is_warm(void) {
  render_environment_t e = daylight_environment(6.6f);
  assert(e.color.x > e.color.z * 1.15f);
  assert(e.sky_color.x > e.sky_color.z * 0.9f);
}

static void the_day_has_no_jumps_and_wraps_round(void) {
  render_environment_t previous = daylight_environment(0.0f);
  for (int i = 1; i <= 24 * 60; i++) {
    render_environment_t now = daylight_environment((float)i / 60.0f);
    assert(fabsf(now.ambient - previous.ambient) < 0.01f);
    assert(fabsf(luminance(now.sky_color) - luminance(previous.sky_color)) <
           0.02f);
    assert(fabsf(luminance(now.color) - luminance(previous.color)) < 0.05f);
    assert(vec3_distance(vec3_normalize(now.direction),
                         vec3_normalize(previous.direction)) < 0.05f);
    previous = now;
  }
  render_environment_t a = daylight_environment(0.0f);
  render_environment_t b = daylight_environment(24.0f);
  assert(near_to(a.ambient, b.ambient, 1e-4f));
  assert(near_to(daylight_sun_height(30.0f), daylight_sun_height(6.0f), 1e-4f));
}

static void lamps_come_on_at_dusk_and_go_off_at_dawn(void) {
  assert(daylight_lamp_glow(18.0f) < 0.3f);
  assert(daylight_lamp_glow(20.5f) > 0.8f);
  assert(daylight_lamp_glow(5.0f) > 0.8f);
  assert(daylight_lamp_glow(8.0f) < 0.05f);
}

static void the_bell_strikes_the_hour_by_day_and_stays_quiet_at_night(void) {
  assert(daylight_bell_strikes(8.99f, 9.01f) == 9);
  assert(daylight_bell_strikes(11.99f, 12.01f) == 12);
  assert(daylight_bell_strikes(12.99f, 13.01f) == 1); /* one o'clock */
  assert(daylight_bell_strikes(18.99f, 19.01f) == 7);
  assert(daylight_bell_strikes(9.2f, 9.4f) == 0);    /* nothing mid-hour */
  assert(daylight_bell_strikes(23.99f, 0.01f) == 0); /* asleep at midnight */
  assert(daylight_bell_strikes(2.99f, 3.01f) == 0);
  assert(daylight_bell_strikes(6.99f, 7.01f) == 7); /* the morning bell */
  assert(daylight_bell_strikes(22.99f, 23.01f) == 0);
}

void run_daylight_tests(void) {
  RUN(the_bell_strikes_the_hour_by_day_and_stays_quiet_at_night);
  RUN(noon_is_bright_with_the_sun_overhead);
  RUN(midnight_is_dark_with_stars_and_lit_lamps);
  RUN(the_sun_rises_in_the_east_and_sets_in_the_west);
  RUN(dawn_light_is_warm);
  RUN(the_day_has_no_jumps_and_wraps_round);
  RUN(lamps_come_on_at_dusk_and_go_off_at_dawn);
}
