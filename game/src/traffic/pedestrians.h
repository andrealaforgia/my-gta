/**
 * @file pedestrians.h
 * @brief People walking the pavements. They follow both sides of every
 * road that has a pavement, and the middle of pedestrian streets and
 * paths, pick a way at each junction, turn back at dead ends, and stop
 * when the player's car is close. They are drawn as simple figures with
 * swinging arms and legs; they are not solid.
 */
#ifndef GAME_SRC_TRAFFIC_PEDESTRIANS_H_
#define GAME_SRC_TRAFFIC_PEDESTRIANS_H_

#include <stddef.h>

#include "city_package.h"
#include "physics_world.h"
#include "renderer.h"

typedef struct pedestrians pedestrians_t;

typedef struct {
  vec3_t position; /* feet */
  float yaw;
  float speed;
  float phase; /* walking cycle in radians */
  int outfit;
  int route;
} pedestrian_t;

typedef struct {
  mesh_handle_t torso, head, leg, arm;
} pedestrian_meshes_t;

pedestrians_t* pedestrians_create(const city_route_t* routes,
                                  size_t route_count, physics_world_t* physics,
                                  int count, uint32_t seed);
void pedestrians_destroy(pedestrians_t* pedestrians);
void pedestrians_step(pedestrians_t* pedestrians, vec3_t player_position,
                      float dt);
size_t pedestrians_count(const pedestrians_t* pedestrians);
pedestrian_t pedestrians_get(const pedestrians_t* pedestrians, size_t index);
/* Stands someone on a route: side +1 is the right of its direction. */
void pedestrians_place(pedestrians_t* pedestrians, size_t index, int route,
                       int side, int direction, float distance);

pedestrian_meshes_t pedestrian_meshes_create(renderer_t* renderer);
void pedestrian_meshes_destroy(pedestrian_meshes_t* meshes,
                               renderer_t* renderer);
int pedestrians_draw(const pedestrians_t* pedestrians,
                     const pedestrian_meshes_t* meshes, renderer_t* renderer,
                     vec3_t eye, float range);

#endif  // GAME_SRC_TRAFFIC_PEDESTRIANS_H_
