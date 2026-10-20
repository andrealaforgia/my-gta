#include "world.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

world_t world_create(void) {
  world_t world = {NULL, 0, 0, 1};
  return world;
}

void world_destroy(world_t* world) {
  free(world->entities);
  world->entities = NULL;
  world->capacity = world->count = 0;
}

static entity_t* free_slot(world_t* world) {
  for (size_t i = 0; i < world->capacity; i++)
    if (!world->entities[i].used) return &world->entities[i];
  size_t capacity = world->capacity ? world->capacity * 2 : 64;
  entity_t* grown = realloc(world->entities, capacity * sizeof(entity_t));
  if (!grown) return NULL;
  memset(grown + world->capacity, 0,
         (capacity - world->capacity) * sizeof(entity_t));
  entity_t* slot = &grown[world->capacity];
  world->entities = grown;
  world->capacity = capacity;
  return slot;
}

uint32_t world_add(world_t* world, transform_t transform, mesh_handle_t mesh,
                   material_t material, aabb_t local_bounds,
                   const char* label) {
  entity_t* entity = free_slot(world);
  if (!entity) return 0;
  memset(entity, 0, sizeof(*entity));
  entity->id = world->next_id++;
  entity->transform = transform;
  entity->mesh = mesh;
  entity->material = material;
  entity->local_bounds = local_bounds;
  entity->world_bounds = aabb_transform(local_bounds, transform);
  snprintf(entity->label, sizeof(entity->label), "%s", label ? label : "");
  entity->used = true;
  world->count++;
  return entity->id;
}

entity_t* world_entity(world_t* world, uint32_t id) {
  for (size_t i = 0; i < world->capacity; i++)
    if (world->entities[i].used && world->entities[i].id == id)
      return &world->entities[i];
  return NULL;
}

void world_remove(world_t* world, uint32_t id) {
  entity_t* entity = world_entity(world, id);
  if (!entity) return;
  entity->used = false;
  world->count--;
}

void world_set_transform(world_t* world, uint32_t id, transform_t transform) {
  entity_t* entity = world_entity(world, id);
  if (!entity) return;
  entity->transform = transform;
  entity->world_bounds = aabb_transform(entity->local_bounds, transform);
}

size_t world_count(const world_t* world) { return world->count; }

void world_for_each_visible(const world_t* world, frustum_t frustum,
                            entity_visitor_t visit, void* context) {
  for (size_t i = 0; i < world->capacity; i++) {
    const entity_t* entity = &world->entities[i];
    if (!entity->used ||
        !frustum_intersects_aabb(frustum, entity->world_bounds))
      continue;
    if (!visit(entity, context)) return;
  }
}

typedef struct {
  renderer_t* renderer;
  int drawn;
} draw_context_t;

static int draw_entity(const entity_t* entity, void* context) {
  draw_context_t* draw = context;
  renderer_draw_mesh(draw->renderer, entity->mesh, entity->material,
                     transform_to_matrix(entity->transform));
  draw->drawn++;
  return 1;
}

int world_render(const world_t* world, renderer_t* renderer) {
  draw_context_t context = {renderer, 0};
  frustum_t frustum = frustum_from_matrix(renderer_view_projection(renderer));
  world_for_each_visible(world, frustum, draw_entity, &context);
  return context.drawn;
}
