/**
 * @file scene.h
 * @brief What a scene gives the driving stage: the entities and
 * colliders are added to the world and physics world, and the stage
 * gets a spawn and an environment back.
 */
#ifndef GAME_SRC_SCENES_SCENE_H_
#define GAME_SRC_SCENES_SCENE_H_

#include <stdbool.h>

#include "city_map.h"
#include "physics_world.h"
#include "renderer.h"
#include "textures.h"
#include "world.h"

#define SCENE_MAX_MESHES 1024
#define POLE_MAX_ENTITIES 5

typedef struct {
  renderer_t* renderer;
  physics_world_t* physics;
  world_t* world;
  const texture_set_t* textures;
} scene_context_t;

typedef enum { POLE_LAMP, POLE_SIGNAL, POLE_SIGN } pole_kind_t;
typedef enum {
  POLE_STANDING,
  POLE_FALLING,
  POLE_DOWN,
  POLE_GONE /* its chunk was unloaded */
} pole_state_t;

/* A lamp post, signal or road sign the car can knock over: its parts are
 * entities of their own, and its solid part a collider that goes when it
 * is hit. Its meshes are in world coordinates, so falling turns them about
 * the foot. */
typedef struct {
  pole_kind_t kind;
  pole_state_t state;
  vec3_t foot;
  float height;
  collider_handle_t collider;
  uint32_t entities[POLE_MAX_ENTITIES];
  int entity_count;
  float fall;    /* seconds since it was hit */
  vec3_t pushed; /* horizontal unit vector it falls along */
  int chunk;     /* the package chunk it belongs to */
} pole_t;

/* What a chunk of the city has in the world while it is loaded: a big city
 * is streamed, only the chunks near the car have meshes, entities and
 * solid buildings. Ground colliders stay everywhere so cars far away still
 * have a road to drive on. */
typedef struct {
  bool loaded;
  uint32_t* entities;
  size_t entity_count, entity_capacity;
  mesh_handle_t* meshes;
  size_t mesh_count, mesh_capacity;
  collider_handle_t* colliders;
  size_t collider_count, collider_capacity;
} chunk_state_t;

typedef struct {
  vec3_t spawn_position;
  quat_t spawn_rotation;
  render_environment_t environment;
  mesh_handle_t meshes[SCENE_MAX_MESHES];
  int mesh_count;
  char name[64];
  city_label_t labels[CITY_PACKAGE_MAX_LABELS]; /* street names for the map */
  size_t label_count;
  city_route_t* routes; /* owned: the roads as cars and people use them */
  size_t route_count;
  city_signal_t* signals; /* owned: the traffic signal heads */
  size_t signal_count;
  float* crossings; /* owned: x, z of zebra crossings */
  size_t crossing_count;
  float* bus_stops; /* owned: x, z of bus stops */
  size_t bus_stop_count;
  city_parking_t* parked; /* owned: places where cars stand at the kerb */
  size_t parked_count;
  pole_t* poles; /* owned: everything that can be knocked down */
  size_t pole_count;
  size_t poles_gone;
  city_package_t* package; /* owned: the city, kept for streaming */
  chunk_state_t* chunk_states;
  float stream_load_radius, stream_unload_radius, stream_near_radius;
  scene_context_t context;
  city_map_t map; /* the pause-screen map; size 0 on the test track */
} scene_t;

bool scene_build_test_track(scene_t* scene, scene_context_t context);
/* Keep a mesh so the scene can release it later. */
void scene_track_mesh(scene_t* scene, mesh_handle_t mesh);
void scene_release(scene_t* scene, renderer_t* renderer);

/* Loads the chunks of a streamed city within about 800 m of `centre` and
 * unloads those beyond 1000 m. At most `budget` chunks are loaded per call,
 * except that anything within 250 m is loaded at once. Nothing for scenes
 * that are not streamed. */
void scene_stream(scene_t* scene, vec3_t centre, int budget);
/* Releases what streamed chunks hold; part of scene_release. */
void scene_release_streamed(scene_t* scene, renderer_t* renderer);

#endif  // GAME_SRC_SCENES_SCENE_H_
