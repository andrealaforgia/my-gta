/**
 * @file route_path.h
 * @brief A route measured and levelled: distance along it, ground height at
 * each point, and sampling of position and heading at a distance in either
 * direction. Shared by traffic and pedestrians.
 */
#ifndef GAME_SRC_TRAFFIC_ROUTE_PATH_H_
#define GAME_SRC_TRAFFIC_ROUTE_PATH_H_

#include <stdbool.h>

#include "city_package.h"
#include "physics_world.h"

typedef struct {
  const city_route_t* route;
  float length;
  float* cumulative; /* distance to each point */
  float* ground;     /* ground height at each point */
} route_path_t;

bool route_path_init(route_path_t* path, const city_route_t* route,
                     physics_world_t* physics);
void route_path_free(route_path_t* path);
/* Position, unit heading and ground height `distance` metres along the route
 * in the direction of travel (+1 start to end, -1 end to start). */
void route_path_sample(const route_path_t* path, float distance, int direction,
                       float* x, float* z, float* dx, float* dz, float* ground);

#endif  // GAME_SRC_TRAFFIC_ROUTE_PATH_H_
