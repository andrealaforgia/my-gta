/**
 * @file street_name.h
 * @brief The name of the street under the car, for the screen.
 */
#ifndef GAME_SRC_STAGE_STREET_NAME_H_
#define GAME_SRC_STAGE_STREET_NAME_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_package.h"

/* The name of the nearest named road or street whose surface and pavement
 * hold the point (x, z), in capitals with spaces, written to `out` (of
 * `size` bytes). False, with `out` empty, when the point is on no named
 * street. */
bool street_name_at(const city_route_t* routes, size_t count, float x, float z,
                    char* out, size_t size);

#endif  // GAME_SRC_STAGE_STREET_NAME_H_
