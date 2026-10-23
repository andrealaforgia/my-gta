/**
 * @file navigation.h
 * @brief Finding a way by road between two places in the city: the drivable
 * routes are the edges of a graph whose nodes are junctions, and the shortest
 * way is found with Dijkstra's algorithm, one-way streets respected. This is
 * what the satnav line on the minimap and the arrow follow.
 */
#ifndef GAME_SRC_STAGE_NAVIGATION_H_
#define GAME_SRC_STAGE_NAVIGATION_H_

#include <stdbool.h>
#include <stddef.h>

#include "city_package.h"

typedef struct nav nav_t;

/* Builds the graph from the routes, which must outlive it. NULL if there is
 * nothing for cars to drive on or memory runs out. */
nav_t* nav_create(const city_route_t* routes, size_t route_count);
void nav_destroy(nav_t* nav);

/* The way from (from_x, from_z) to (to_x, to_z) as x, z pairs, nearest road
 * to nearest road: returns how many points were written to `out`, at most
 * `max_points`, or 0 if no road joins them. `length` receives the metres of
 * driving when not NULL. */
size_t nav_path(nav_t* nav, float from_x, float from_z, float to_x, float to_z,
                float* out, size_t max_points, float* length);

/* For a vehicle at the end of `route` heading `forward` (1 along the route's
 * points, 0 against them): the road to take next to get to (to_x, to_z) by
 * the shortest way. False if the target's road is this one, or none joins. */
bool nav_next_hop(nav_t* nav, int route, int forward, float to_x, float to_z,
                  int* next_route, int* next_forward);

#endif  // GAME_SRC_STAGE_NAVIGATION_H_
