#include "daylight.h"

#include <math.h>

static float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static float smooth(float edge0, float edge1, float x) {
  float t = clamp01((x - edge0) / (edge1 - edge0));
  return t * t * (3.0f - 2.0f * t);
}

static vec3_t mix(vec3_t a, vec3_t b, float t) {
  return vec3_add(vec3_scale(a, 1.0f - t), vec3_scale(b, t));
}

static float wrap_hour(float hour) {
  hour = fmodf(hour, 24.0f);
  return hour < 0 ? hour + 24.0f : hour;
}

/* The sun climbs from 6 to 18, sinusoidally; the moon takes the other half
 * of the circle. */
float daylight_sun_height(float hour) {
  return sinf((wrap_hour(hour) - 6.0f) / 12.0f * (float)M_PI);
}

float daylight_lamp_glow(float hour) {
  return 1.0f - smooth(-0.15f, 0.05f, daylight_sun_height(hour));
}

render_environment_t daylight_environment(float hour) {
  hour = wrap_hour(hour);
  float height = daylight_sun_height(hour);
  float day = smooth(-0.1f, 0.25f, height); /* 0 at night, 1 by mid-morning */
  float warm = 1.0f - smooth(0.0f, 0.4f, fabsf(height)); /* near the horizon */
  warm *= smooth(-0.25f, 0.0f, height);

  /* One continuous arc over the whole day: the sun climbs from east to
   * west, and the moon, the mirror of it, carries on back round to the
   * east, so the light never jumps at dusk or dawn. */
  float angle = (hour - 6.0f) / 12.0f * (float)M_PI;
  vec3_t direction = vec3_normalize(
      vec3(cosf(angle) * 0.9f, fabsf(sinf(angle)) + 0.05f, -0.35f));

  vec3_t day_light = vec3(1.15f, 1.08f, 0.95f);
  vec3_t dawn_light = vec3(1.25f, 0.8f, 0.5f);
  vec3_t moon_light = vec3(0.16f, 0.2f, 0.32f);
  vec3_t sun_colour = mix(day_light, dawn_light, warm);
  sun_colour = vec3_scale(sun_colour, fmaxf(0.25f, clamp01(height * 3.0f)));
  vec3_t light = mix(moon_light, sun_colour, day);

  vec3_t day_horizon = vec3(0.62f, 0.74f, 0.9f);
  vec3_t dusk_horizon = vec3(0.95f, 0.58f, 0.38f);
  vec3_t night_horizon = vec3(0.03f, 0.04f, 0.08f);
  vec3_t horizon = mix(day_horizon, dusk_horizon, warm * 0.85f);
  horizon = mix(night_horizon, horizon, day);

  vec3_t day_zenith = vec3(0.25f, 0.45f, 0.85f);
  vec3_t night_zenith = vec3(0.005f, 0.008f, 0.03f);
  vec3_t zenith = mix(night_zenith, day_zenith, day);
  zenith = mix(zenith, vec3(0.35f, 0.4f, 0.65f), warm * 0.4f * day);

  render_environment_t e;
  e.direction = direction;
  e.color = light;
  e.ambient = 0.06f + 0.24f * day;
  e.sky_color = horizon;
  e.fog_start = 150.0f;
  e.fog_end = 900.0f;
  e.shadow_radius = 110.0f;
  e.zenith_color = zenith;
  e.star_level = 1.0f - smooth(-0.15f, 0.05f, height);
  e.sun_disc = smooth(-0.08f, 0.02f, height);
  e.spot_position = vec3(0, 0, 0);
  e.spot_direction = vec3(0, 0, -1);
  e.spot_intensity = 0;
  return e;
}

int daylight_bell_strikes(float before, float now) {
  int was = (int)floorf(wrap_hour(before));
  int is = (int)floorf(wrap_hour(now));
  if (was == is) return 0;
  if (is < 7 || is > 22) return 0;
  int strikes = is % 12;
  return strikes == 0 ? 12 : strikes;
}
