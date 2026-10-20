/**
 * @file parked_cars.h
 * @brief Cars standing at the kerb: placed on the ground at the parking
 * places the generator found, each with a solid box so the player can hit
 * it. Only those near the camera are drawn.
 */
#ifndef GAME_SRC_TRAFFIC_PARKED_CARS_H_
#define GAME_SRC_TRAFFIC_PARKED_CARS_H_

#include <stddef.h>

#include "city_package.h"
#include "fleet.h"
#include "physics_world.h"

typedef struct {
  vec3_t position; /* chassis centre */
  float yaw;
  int variant;
  int paint;
  collider_handle_t collider;
} parked_car_t;

typedef struct {
  parked_car_t* cars;
  size_t count;
} parked_cars_t;

/* Stands a car at every spot that has ground under it. */
bool parked_cars_create(parked_cars_t* parked, const city_parking_t* spots,
                        size_t spot_count, physics_world_t* physics,
                        const fleet_t* fleet);
void parked_cars_destroy(parked_cars_t* parked, physics_world_t* physics);
/* Draws those within `range` metres of the eye; returns how many. */
int parked_cars_draw(const parked_cars_t* parked, const fleet_t* fleet,
                     renderer_t* renderer, vec3_t eye, float range,
                     float lights_on);

#endif  // GAME_SRC_TRAFFIC_PARKED_CARS_H_
