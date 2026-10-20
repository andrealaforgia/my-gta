/**
 * @file fleet.h
 * @brief The cars that are not the player's: four body styles built from
 * their own tuning files, and a palette of paints. Shared by parked and
 * moving cars.
 */
#ifndef GAME_SRC_TRAFFIC_FLEET_H_
#define GAME_SRC_TRAFFIC_FLEET_H_

#include <stdbool.h>

#include "car_visual.h"

#define FLEET_VARIANTS 4
#define FLEET_PAINTS 10

typedef struct {
  car_visual_t visuals[FLEET_VARIANTS];
} fleet_t;

bool fleet_create(fleet_t* fleet, renderer_t* renderer);
void fleet_destroy(fleet_t* fleet, renderer_t* renderer);
/* Any number picks a style and paint: styles weighted to mostly hatchbacks
 * and saloons. */
int fleet_variant_for(int pick);
vec3_t fleet_paint(int pick);

#endif  // GAME_SRC_TRAFFIC_FLEET_H_
