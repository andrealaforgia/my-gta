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
#include "navigation.h"
#include "physics_world.h"

/* Cars are drawn within this many metres of the eye; a car is only ever
 * put on the road beyond it, so none appears out of thin air. */
#define TRAFFIC_VISIBLE_RANGE 450.0f

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
  int indicator; /* -1 signalling left, +1 right, 0 none */
  bool pursuing; /* a police car after the player */
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
/* Traffic signal heads, borrowed: a car heading the way a head controls
 * stops six metres short of it while its group shows red. */
void traffic_set_signals(traffic_t* traffic, const city_signal_t* signals,
                         size_t count);
/* The place traffic should be thick around, usually the player: a car more
 * than 1200 m from it is moved to a road just beyond sight (see
 * TRAFFIC_VISIBLE_RANGE) from it, so a big city
 * has cars where someone can see them. */
void traffic_set_focus(traffic_t* traffic, vec3_t focus);
/* Bus stops (x, z pairs, borrowed): a bus halts at each it passes for a
 * few seconds. */
void traffic_set_bus_stops(traffic_t* traffic, const float* xz, size_t count);
/* People in the road (crossing a street): cars stop for them. Borrowed. */
void traffic_set_walkers(traffic_t* traffic, const vec3_t* positions,
                         size_t count);
/* The time in seconds on the signals' cycle. */
void traffic_set_signal_time(traffic_t* traffic, float seconds);
/* Whether a group shows green at a time in seconds: 21 s of red, 3 of red
 * and amber, 21 of green, 3 of amber, repeating; group 1 runs half a cycle
 * behind group 0, so crossing streets take turns. */
bool traffic_signal_open(float seconds, int group);
/* The same for group 0. */
bool traffic_signals_open(float seconds);
/* Makes car `index` a given fleet variant, on a call or not. For setting up
 * scenes and tests. */
void traffic_car_set_kind(traffic_t* traffic, size_t index, int variant,
                          bool responding);
/* Distance to the nearest car that is sounding its horn at the player, who
 * has held it up for a while, or a huge number if none is. */
float traffic_nearest_honk(const traffic_t* t, vec3_t point);
/* Distance from a point to the nearest emergency vehicle that is on a call
 * (beacons flashing, siren sounding), 1e9 with none. */
float traffic_nearest_siren(const traffic_t* traffic, vec3_t point);
/* Whether car `index` is on a call. */
bool traffic_car_responding(const traffic_t* traffic, size_t index);
/* Distance from a point to the nearest moving car, 1e9 with none. */
float traffic_nearest_distance(const traffic_t* traffic, vec3_t point);
/* Draws the cars within `range` of the eye; returns how many. */
int traffic_draw(const traffic_t* traffic, const fleet_t* fleet,
                 renderer_t* renderer, vec3_t eye, float range,
                 float lights_on);

/* The road graph pursuers use to find their way to the player; borrowed,
 * optional (without it they choose junctions at random). */
void traffic_set_nav(traffic_t* traffic, nav_t* nav);
/* How many police cars should be after the player, 0 for none: the traffic
 * has a few police cars, and sends them from out of sight, siren sounding,
 * faster than anyone, ignoring signals. Call them off with 0. */
void traffic_set_pursuit(traffic_t* traffic, int pursuers);
int traffic_pursuer_count(const traffic_t* traffic);
/* Distance from a point to the nearest pursuing police car, 1e9 with none. */
float traffic_nearest_pursuer(const traffic_t* traffic, vec3_t point);

/* Takes the nearest car within `radius` metres of a point (flat distance),
 * which is what a player does to steal one: the car is moved to a road out
 * of sight, and what it was is returned. False if none is near. */
bool traffic_take_car(traffic_t* traffic, vec3_t near, float radius,
                      int* variant, vec3_t* position, float* yaw);

#endif  // GAME_SRC_TRAFFIC_TRAFFIC_H_
