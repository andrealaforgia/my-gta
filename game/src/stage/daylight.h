/**
 * @file daylight.h
 * @brief Time of day: the sun's path, the colour of light and sky, and
 * whether the street lamps are lit. Pure functions of the hour (0..24).
 */
#ifndef GAME_SRC_STAGE_DAYLIGHT_H_
#define GAME_SRC_STAGE_DAYLIGHT_H_

#include "renderer.h"

/* The environment at an hour; fog range and shadow radius are left for the
 * caller to fill. Wraps: hour 24 is hour 0. */
render_environment_t daylight_environment(float hour);
/* 0 in daylight, 1 once it is dark: the emissive level for lamps. */
float daylight_lamp_glow(float hour);
/* Sun height 1 at noon, 0 on the horizon, negative below it. */
float daylight_sun_height(float hour);

#endif  // GAME_SRC_STAGE_DAYLIGHT_H_
