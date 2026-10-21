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
  assert(e.moon_disc < 0.01f);
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
  /* The moon is up where the sun was, and the sun is not. */
  assert(e.moon_disc > 0.9f);
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

static void rain_greys_the_day_closes_the_fog_and_hides_the_stars(void) {
  render_environment_t clear = daylight_environment(13.0f);
  render_environment_t rain = clear;
  daylight_apply_weather(&rain, 1.0f);
  /* No rain changes nothing. */
  render_environment_t none = clear;
  daylight_apply_weather(&none, 0.0f);
  assert(vec3_distance(none.sky_color, clear.sky_color) < 1e-6f);
  assert(near_to(none.fog_end, clear.fog_end, 1e-3f));
  /* Heavy rain: duller light, a grey sky with less blue in it, a nearer fog. */
  assert(luminance(rain.color) < luminance(clear.color) * 0.6f);
  assert(luminance(rain.sky_color) < luminance(clear.sky_color));
  assert(rain.sky_color.z - rain.sky_color.x <
         clear.sky_color.z - clear.sky_color.x);
  assert(rain.fog_end < clear.fog_end * 0.5f &&
         rain.fog_start < clear.fog_start);
  assert(rain.sun_disc < 0.05f);
  assert(rain.ambient >
         clear.ambient * 0.8f); /* overcast light is soft, not dark */
  /* At night the stars go behind the cloud. */
  render_environment_t night = daylight_environment(1.0f);
  assert(night.star_level > 0.9f);
  daylight_apply_weather(&night, 1.0f);
  assert(night.star_level < 0.05f);
  assert(night.moon_disc < 0.05f);
  /* In between is in between. */
  render_environment_t half = clear;
  daylight_apply_weather(&half, 0.5f);
  assert(half.fog_end < clear.fog_end && half.fog_end > rain.fog_end);
}

static void clouds_drift_and_thicken_with_rain(void) {
  render_environment_t clear = daylight_environment(11.0f);
  assert(clear.cloud_cover > 0.2f && clear.cloud_cover < 0.6f);
  render_environment_t rain = clear;
  daylight_apply_weather(&rain, 1.0f);
  assert(rain.cloud_cover > 0.95f);
  render_environment_t half = clear;
  daylight_apply_weather(&half, 0.5f);
  assert(half.cloud_cover > clear.cloud_cover &&
         half.cloud_cover < rain.cloud_cover);
  /* Clouds are as dark at night as the light that falls on them. */
  assert(daylight_environment(0.0f).cloud_cover > 0.2f);
}

void run_daylight_tests(void) {
  RUN(clouds_drift_and_thicken_with_rain);
  RUN(rain_greys_the_day_closes_the_fog_and_hides_the_stars);
  RUN(the_bell_strikes_the_hour_by_day_and_stays_quiet_at_night);
  RUN(noon_is_bright_with_the_sun_overhead);
  RUN(midnight_is_dark_with_stars_and_lit_lamps);
  RUN(the_sun_rises_in_the_east_and_sets_in_the_west);
  RUN(dawn_light_is_warm);
  RUN(the_day_has_no_jumps_and_wraps_round);
  RUN(lamps_come_on_at_dusk_and_go_off_at_dawn);
}
