/**
 * @file city_map.h
 * @brief The city flattened to a square RGBA picture, north at the top,
 * for the pause-screen map. Built once from the cooked package: parks,
 * water, car parks, paving, roads and roofs painted from their meshes.
 */
#ifndef GAME_SRC_SCENES_CITY_MAP_H_
#define GAME_SRC_SCENES_CITY_MAP_H_

#include <stdbool.h>
#include <stdint.h>

#include "city_package.h"

typedef struct {
  int size;        /* pixels along each side; 0 when there is no map */
  uint8_t* pixels; /* size * size RGBA, row 0 is the north edge */
  float min_x, min_z;
  float extent; /* metres covered by the whole picture */
} city_map_t;

bool city_map_build(city_map_t* map, const city_package_t* package, int size);
void city_map_free(city_map_t* map);
/* False when the point lies outside the picture. */
bool city_map_to_pixel(const city_map_t* map, float x, float z, int* px,
                       int* py);

#endif  // GAME_SRC_SCENES_CITY_MAP_H_
