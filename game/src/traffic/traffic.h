/**
 * @file traffic.h
 * @brief Cars that drive about the city on their own. They follow the
 * route centre lines in the right-hand lane, slow for bends and junctions,
 * choose a road at each junction, turn round at dead ends, keep a gap to
 * the car ahead and stop for the player or another car in their way. Each
 * has a box collider moved with it, so the player can hit them.
 */
#ifndef GAME_SRC_TRAFFIC_TRAFFIC_H_
#define GAME_SRC_TRAFFIC_TRAFFIC_H_

#include <stddef.h>

#include "city_package.h"
#include "fleet.h"
#include "physics_world.h"

typedef struct traffic traffic_t;

typedef struct {
  vec3_t position; /* chassis centre */
  float yaw;
  float speed;
  int route;
  int variant;
  int paint;
  bool braking;
  float wheel_turn;
} traffic_car_t;

/* Routes are borrowed: they must outlive the traffic. Puts `car_count` cars
 * on drivable routes, spread out, from a seed. Returns NULL if no road can
 * carry them. */
traffic_t* traffic_create(const city_route_t* routes, size_t route_count,
                          physics_world_t* physics, const fleet_t* fleet,
                          int car_count, uint32_t seed);
void traffic_destroy(traffic_t* traffic, physics_world_t* physics);
/* Moves every car on by dt. The player is a point and a velocity to stop for.
 */
void traffic_step(traffic_t* traffic, physics_world_t* physics,
                  vec3_t player_position, vec3_t player_velocity, float dt);
size_t traffic_car_count(const traffic_t* traffic);
traffic_car_t traffic_car(const traffic_t* traffic, size_t index);
/* Puts a car on a route going `direction` (+1 start to end, -1 back),
 * `distance` metres in. For setting up scenes and tests. */
void traffic_car_place(traffic_t* traffic, size_t index, int route,
                       int direction, float distance);
/* Draws the cars within `range` of the eye; returns how many. */
int traffic_draw(const traffic_t* traffic, const fleet_t* fleet,
                 renderer_t* renderer, vec3_t eye, float range,
                 float lights_on);

#endif  // GAME_SRC_TRAFFIC_TRAFFIC_H_
