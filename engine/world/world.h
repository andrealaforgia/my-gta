/**
 * @file world.h
 * @brief Renderable entities with stable ids, transforms and bounds. A
 * flat list with frustum culling; a spatial structure can replace the
 * scan behind world_for_each_visible when profiling asks for it.
 */
#ifndef ENGINE_WORLD_WORLD_H_
#define ENGINE_WORLD_WORLD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aabb.h"
#include "frustum.h"
#include "renderer.h"
#include "transform.h"

#define ENTITY_LABEL_LENGTH 32

typedef struct {
  uint32_t id;
  transform_t transform;
  mesh_handle_t mesh;
  material_t material;
  aabb_t local_bounds;
  aabb_t world_bounds;
  char label[ENTITY_LABEL_LENGTH];
  bool used;
} entity_t;

typedef struct {
  entity_t* entities;
  size_t capacity;
  size_t count;
  uint32_t next_id;
} world_t;

typedef int (*entity_visitor_t)(const entity_t* entity, void* context);

world_t world_create(void);
void world_destroy(world_t* world);
uint32_t world_add(world_t* world, transform_t transform, mesh_handle_t mesh,
                   material_t material, aabb_t local_bounds, const char* label);
void world_remove(world_t* world, uint32_t id);
entity_t* world_entity(world_t* world, uint32_t id);
void world_set_transform(world_t* world, uint32_t id, transform_t transform);
size_t world_count(const world_t* world);
/* Visits entities whose bounds meet the frustum; the visitor returns 0 to stop.
 */
void world_for_each_visible(const world_t* world, frustum_t frustum,
                            entity_visitor_t visit, void* context);
/* Draws the visible entities and returns how many were drawn. */
int world_render(const world_t* world, renderer_t* renderer);

#endif  // ENGINE_WORLD_WORLD_H_
