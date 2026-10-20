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

#define SCENE_MAX_MESHES 4096

typedef struct {
  vec3_t spawn_position;
  quat_t spawn_rotation;
  render_environment_t environment;
  mesh_handle_t meshes[SCENE_MAX_MESHES];
  int mesh_count;
  char name[64];
  city_label_t labels[CITY_PACKAGE_MAX_LABELS]; /* street names for the map */
  size_t label_count;
  city_map_t map; /* the pause-screen map; size 0 on the test track */
} scene_t;

typedef struct {
  renderer_t* renderer;
  physics_world_t* physics;
  world_t* world;
  const texture_set_t* textures;
} scene_context_t;

bool scene_build_test_track(scene_t* scene, scene_context_t context);
/* Keep a mesh so the scene can release it later. */
void scene_track_mesh(scene_t* scene, mesh_handle_t mesh);
void scene_release(scene_t* scene, renderer_t* renderer);

#endif  // GAME_SRC_SCENES_SCENE_H_
