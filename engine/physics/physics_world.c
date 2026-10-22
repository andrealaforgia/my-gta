#include "physics_world.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

#define MAX_BODIES 16
#define MAX_COLLIDERS 65536
#define MAX_CONTACTS 256
#define SOLVER_ITERATIONS 6
#define CONTACT_MARGIN \
  0.03f /* corners this close to a surface count as touching */
#define PENETRATION_SLOP 0.005f
#define PENETRATION_CORRECTION \
  0.3f /* fraction of the penetration recovered per step */

/* The static colliders are kept in a hash grid of square cells on the ground
 * plane, so that a ray or a body looks only at what is near it. A collider is
 * listed in every cell its bounds overlap; one so large that it would take
 * more than MAX_CELLS_PER_COLLIDER goes on a short list that is always
 * looked at. */
#define CELL_SIZE 24.0f
#define MAX_CELLS_PER_COLLIDER 256
#define MAX_CELLS_PER_QUERY 1500

typedef struct {
  int32_t cx, cz;
  uint32_t* items;
  uint32_t count, capacity;
  bool used;
} grid_cell_t;

struct physics_world {
  vec3_t gravity;
  rigid_body_t bodies[MAX_BODIES];
  static_collider_t* colliders;
  size_t collider_count;
  contact_event_t contacts[MAX_CONTACTS];
  size_t contact_count;
  /* Free collider slots, the most recently freed last. */
  uint32_t* free_slots;
  size_t free_count;
  /* The grid. */
  grid_cell_t* cells;
  size_t cell_capacity, cell_count;
  uint32_t* oversize;
  size_t oversize_count, oversize_capacity;
  /* Scratch for queries: the candidates found, and a stamp per collider so
   * each is listed once. */
  uint32_t* candidates;
  size_t candidate_count, candidate_capacity;
  uint32_t* stamps;
  uint32_t stamp;
};

physics_world_t* physics_world_create(void) {
  physics_world_t* world = calloc(1, sizeof(physics_world_t));
  if (!world) return NULL;
  world->colliders = calloc(MAX_COLLIDERS, sizeof(static_collider_t));
  world->free_slots = malloc(MAX_COLLIDERS * sizeof(uint32_t));
  world->stamps = calloc(MAX_COLLIDERS, sizeof(uint32_t));
  world->cell_capacity = 1024;
  world->cells = calloc(world->cell_capacity, sizeof(grid_cell_t));
  if (!world->colliders || !world->free_slots || !world->stamps ||
      !world->cells) {
    free(world->colliders);
    free(world->free_slots);
    free(world->stamps);
    free(world->cells);
    free(world);
    return NULL;
  }
  world->gravity = vec3(0, -9.81f, 0);
  return world;
}

void physics_world_destroy(physics_world_t* world) {
  if (!world) return;
  for (size_t i = 0; i < world->collider_count; i++) {
    free(world->colliders[i].vertices);
    free(world->colliders[i].indices);
  }
  for (size_t i = 0; i < world->cell_capacity; i++) free(world->cells[i].items);
  free(world->cells);
  free(world->oversize);
  free(world->candidates);
  free(world->stamps);
  free(world->free_slots);
  free(world->colliders);
  free(world);
}

void physics_world_set_gravity(physics_world_t* world, vec3_t gravity) {
  world->gravity = gravity;
}
vec3_t physics_world_gravity(const physics_world_t* world) {
  return world->gravity;
}

/* ---- the grid ---- */

static size_t cell_hash(int32_t cx, int32_t cz, size_t capacity) {
  uint32_t h = (uint32_t)cx * 73856093u ^ (uint32_t)cz * 19349663u;
  return h & (capacity - 1);
}

static grid_cell_t* find_cell(physics_world_t* world, int32_t cx, int32_t cz,
                              bool create);

static bool grow_cells(physics_world_t* world) {
  size_t capacity = world->cell_capacity * 2;
  grid_cell_t* cells = calloc(capacity, sizeof(grid_cell_t));
  if (!cells) return false;
  for (size_t i = 0; i < world->cell_capacity; i++) {
    grid_cell_t* old = &world->cells[i];
    if (!old->used) continue;
    size_t h = cell_hash(old->cx, old->cz, capacity);
    while (cells[h].used) h = (h + 1) & (capacity - 1);
    cells[h] = *old;
  }
  free(world->cells);
  world->cells = cells;
  world->cell_capacity = capacity;
  return true;
}

static grid_cell_t* find_cell(physics_world_t* world, int32_t cx, int32_t cz,
                              bool create) {
  size_t h = cell_hash(cx, cz, world->cell_capacity);
  while (world->cells[h].used) {
    if (world->cells[h].cx == cx && world->cells[h].cz == cz)
      return &world->cells[h];
    h = (h + 1) & (world->cell_capacity - 1);
  }
  if (!create) return NULL;
  if ((world->cell_count + 1) * 2 > world->cell_capacity) {
    if (!grow_cells(world)) return NULL;
    return find_cell(world, cx, cz, true);
  }
  grid_cell_t* cell = &world->cells[h];
  cell->used = true;
  cell->cx = cx;
  cell->cz = cz;
  world->cell_count++;
  return cell;
}

static void cell_range(aabb_t bounds, int32_t* x0, int32_t* z0, int32_t* x1,
                       int32_t* z1) {
  *x0 = (int32_t)floorf(bounds.min.x / CELL_SIZE);
  *x1 = (int32_t)floorf(bounds.max.x / CELL_SIZE);
  *z0 = (int32_t)floorf(bounds.min.z / CELL_SIZE);
  *z1 = (int32_t)floorf(bounds.max.z / CELL_SIZE);
}

static bool list_add(uint32_t** items, uint32_t* count, uint32_t* capacity,
                     uint32_t value) {
  if (*count == *capacity) {
    uint32_t grown = *capacity ? *capacity * 2 : 4;
    uint32_t* bigger = realloc(*items, grown * sizeof(uint32_t));
    if (!bigger) return false;
    *items = bigger;
    *capacity = grown;
  }
  (*items)[(*count)++] = value;
  return true;
}

static void grid_insert(physics_world_t* world, uint32_t index) {
  aabb_t bounds = world->colliders[index].bounds;
  int32_t x0, z0, x1, z1;
  cell_range(bounds, &x0, &z0, &x1, &z1);
  int64_t cells = (int64_t)(x1 - x0 + 1) * (int64_t)(z1 - z0 + 1);
  if (cells > MAX_CELLS_PER_COLLIDER) {
    uint32_t count = (uint32_t)world->oversize_count,
             capacity = (uint32_t)world->oversize_capacity;
    list_add(&world->oversize, &count, &capacity, index);
    world->oversize_count = count;
    world->oversize_capacity = capacity;
    return;
  }
  for (int32_t cz = z0; cz <= z1; cz++)
    for (int32_t cx = x0; cx <= x1; cx++) {
      grid_cell_t* cell = find_cell(world, cx, cz, true);
      if (cell) list_add(&cell->items, &cell->count, &cell->capacity, index);
    }
}

static void list_remove(uint32_t* items, uint32_t* count, uint32_t value) {
  for (uint32_t i = 0; i < *count; i++)
    if (items[i] == value) {
      items[i] = items[--(*count)];
      return;
    }
}

static void grid_remove(physics_world_t* world, uint32_t index, aabb_t bounds) {
  int32_t x0, z0, x1, z1;
  cell_range(bounds, &x0, &z0, &x1, &z1);
  int64_t cells = (int64_t)(x1 - x0 + 1) * (int64_t)(z1 - z0 + 1);
  if (cells > MAX_CELLS_PER_COLLIDER) {
    uint32_t count = (uint32_t)world->oversize_count;
    list_remove(world->oversize, &count, index);
    world->oversize_count = count;
    return;
  }
  for (int32_t cz = z0; cz <= z1; cz++)
    for (int32_t cx = x0; cx <= x1; cx++) {
      grid_cell_t* cell = find_cell(world, cx, cz, false);
      if (cell) list_remove(cell->items, &cell->count, index);
    }
}

static int compare_indices(const void* a, const void* b) {
  uint32_t x = *(const uint32_t*)a, y = *(const uint32_t*)b;
  return x < y ? -1 : (x > y);
}

static void candidate_add(physics_world_t* world, uint32_t index) {
  if (world->stamps[index] == world->stamp) return;
  world->stamps[index] = world->stamp;
  uint32_t count = (uint32_t)world->candidate_count,
           capacity = (uint32_t)world->candidate_capacity;
  list_add(&world->candidates, &count, &capacity, index);
  world->candidate_count = count;
  world->candidate_capacity = capacity;
}

/* Lists, in the order of their slots, the colliders whose bounds may overlap
 * the box (its footprint is what the grid sees), in world->candidates. */
static void gather(physics_world_t* world, aabb_t box) {
  world->candidate_count = 0;
  if (++world->stamp == 0) {
    memset(world->stamps, 0, MAX_COLLIDERS * sizeof(uint32_t));
    world->stamp = 1;
  }
  int32_t x0, z0, x1, z1;
  cell_range(box, &x0, &z0, &x1, &z1);
  int64_t cells = (int64_t)(x1 - x0 + 1) * (int64_t)(z1 - z0 + 1);
  if (cells > MAX_CELLS_PER_QUERY) {
    /* Too wide to be worth walking: everything is a candidate. */
    for (uint32_t i = 0; i < world->collider_count; i++)
      if (world->colliders[i].used) candidate_add(world, i);
    return;
  }
  for (int32_t cz = z0; cz <= z1; cz++)
    for (int32_t cx = x0; cx <= x1; cx++) {
      grid_cell_t* cell = find_cell(world, cx, cz, false);
      if (!cell) continue;
      for (uint32_t k = 0; k < cell->count; k++)
        candidate_add(world, cell->items[k]);
    }
  for (size_t k = 0; k < world->oversize_count; k++)
    candidate_add(world, world->oversize[k]);
  qsort(world->candidates, world->candidate_count, sizeof(uint32_t),
        compare_indices);
}

/* ---- colliders ---- */

static static_collider_t* new_collider(physics_world_t* world,
                                       collider_handle_t* handle) {
  if (world->free_count > 0) {
    uint32_t slot = world->free_slots[--world->free_count];
    handle->id = slot + 1;
    return &world->colliders[slot];
  }
  if (world->collider_count >= MAX_COLLIDERS) {
    LOG_ERROR("physics", "collider table full (%d)", MAX_COLLIDERS);
    handle->id = 0;
    return NULL;
  }
  handle->id = (uint32_t)world->collider_count + 1;
  return &world->colliders[world->collider_count++];
}

collider_handle_t physics_add_static_box(physics_world_t* world,
                                         transform_t transform,
                                         vec3_t half_extents,
                                         collision_layer_t layer,
                                         float friction, uint64_t source_id) {
  collider_handle_t handle;
  static_collider_t* collider = new_collider(world, &handle);
  if (!collider) return handle;
  memset(collider, 0, sizeof(*collider));
  collider->shape = COLLIDER_SHAPE_BOX;
  collider->transform = transform;
  collider->half_extents = half_extents;
  collider->bounds =
      aabb_transform(aabb_from_center(vec3_zero(), half_extents), transform);
  collider->layer = layer;
  collider->friction = friction;
  collider->source_id = source_id;
  collider->used = true;
  grid_insert(world, handle.id - 1);
  return handle;
}

void physics_set_static_box_transform(physics_world_t* world,
                                      collider_handle_t handle,
                                      transform_t transform) {
  if (handle.id == 0 || handle.id > world->collider_count) return;
  static_collider_t* collider = &world->colliders[handle.id - 1];
  if (!collider->used || collider->shape != COLLIDER_SHAPE_BOX) return;
  grid_remove(world, handle.id - 1, collider->bounds);
  collider->transform = transform;
  collider->bounds = aabb_transform(
      aabb_from_center(vec3_zero(), collider->half_extents), transform);
  grid_insert(world, handle.id - 1);
}

collider_handle_t physics_add_static_mesh(
    physics_world_t* world, const vec3_t* vertices, size_t vertex_count,
    const uint32_t* indices, size_t index_count, collision_layer_t layer,
    float friction, uint64_t source_id) {
  collider_handle_t none = {0};
  if (vertex_count == 0 || index_count < 3) return none;
  vec3_t* own_vertices = malloc(vertex_count * sizeof(vec3_t));
  uint32_t* own_indices = malloc(index_count * sizeof(uint32_t));
  if (!own_vertices || !own_indices) {
    free(own_vertices);
    free(own_indices);
    return none;
  }
  collider_handle_t handle;
  static_collider_t* collider = new_collider(world, &handle);
  if (!collider) {
    free(own_vertices);
    free(own_indices);
    return handle;
  }
  memcpy(own_vertices, vertices, vertex_count * sizeof(vec3_t));
  memcpy(own_indices, indices, index_count * sizeof(uint32_t));
  memset(collider, 0, sizeof(*collider));
  collider->shape = COLLIDER_SHAPE_MESH;
  collider->transform = transform_identity();
  collider->vertices = own_vertices;
  collider->vertex_count = vertex_count;
  collider->indices = own_indices;
  collider->index_count = index_count - index_count % 3;
  collider->bounds = aabb_empty();
  for (size_t i = 0; i < vertex_count; i++)
    collider->bounds = aabb_include(collider->bounds, vertices[i]);
  collider->layer = layer;
  collider->friction = friction;
  collider->source_id = source_id;
  collider->used = true;
  grid_insert(world, handle.id - 1);
  return handle;
}

void physics_remove_collider(physics_world_t* world, collider_handle_t handle) {
  if (handle.id == 0 || handle.id > world->collider_count) return;
  static_collider_t* collider = &world->colliders[handle.id - 1];
  if (!collider->used) return;
  grid_remove(world, handle.id - 1, collider->bounds);
  free(collider->vertices);
  free(collider->indices);
  memset(collider, 0, sizeof(*collider));
  world->free_slots[world->free_count++] = handle.id - 1;
}

size_t physics_collider_count(const physics_world_t* world) {
  return world->collider_count;
}

const static_collider_t* physics_collider_at(const physics_world_t* world,
                                             size_t index) {
  return index < world->collider_count ? &world->colliders[index] : NULL;
}

/* ---- bodies ---- */

body_handle_t physics_add_body(physics_world_t* world, vec3_t position,
                               quat_t rotation, float mass,
                               vec3_t half_extents) {
  body_handle_t none = {0};
  for (uint32_t i = 0; i < MAX_BODIES; i++) {
    rigid_body_t* body = &world->bodies[i];
    if (body->used) continue;
    memset(body, 0, sizeof(*body));
    body->position = position;
    body->rotation = rotation;
    body->mass = mass;
    body->inverse_mass = mass > 0 ? 1.0f / mass : 0;
    body->half_extents = half_extents;
    float w = half_extents.x * 2, h = half_extents.y * 2,
          d = half_extents.z * 2;
    vec3_t inertia =
        vec3(mass / 12.0f * (h * h + d * d), mass / 12.0f * (w * w + d * d),
             mass / 12.0f * (w * w + h * h));
    body->inverse_inertia_local =
        vec3(1.0f / inertia.x, 1.0f / inertia.y, 1.0f / inertia.z);
    body->linear_damping = 0.02f;
    body->angular_damping = 0.1f;
    body->restitution = 0.05f;
    body->previous = transform_at(position, rotation);
    body->used = true;
    body_handle_t handle = {i + 1};
    return handle;
  }
  LOG_ERROR("physics", "body table full (%d)", MAX_BODIES);
  return none;
}

void physics_remove_body(physics_world_t* world, body_handle_t handle) {
  if (handle.id == 0 || handle.id > MAX_BODIES) return;
  memset(&world->bodies[handle.id - 1], 0, sizeof(rigid_body_t));
}

rigid_body_t* physics_body(physics_world_t* world, body_handle_t handle) {
  if (handle.id == 0 || handle.id > MAX_BODIES ||
      !world->bodies[handle.id - 1].used)
    return NULL;
  return &world->bodies[handle.id - 1];
}

void physics_body_teleport(physics_world_t* world, body_handle_t handle,
                           vec3_t position, quat_t rotation) {
  rigid_body_t* body = physics_body(world, handle);
  if (!body) return;
  body->position = position;
  body->rotation = rotation;
  body->velocity = vec3_zero();
  body->angular_velocity = vec3_zero();
  body->force = vec3_zero();
  body->torque = vec3_zero();
  body->previous = transform_at(position, rotation);
}

void physics_apply_force(physics_world_t* world, body_handle_t handle,
                         vec3_t force) {
  rigid_body_t* body = physics_body(world, handle);
  if (body) body->force = vec3_add(body->force, force);
}

void physics_apply_force_at_point(physics_world_t* world, body_handle_t handle,
                                  vec3_t force, vec3_t local_point) {
  rigid_body_t* body = physics_body(world, handle);
  if (!body) return;
  vec3_t arm = quat_rotate(body->rotation, local_point);
  body->force = vec3_add(body->force, force);
  body->torque = vec3_add(body->torque, vec3_cross(arm, force));
}

void physics_apply_force_at_world_point(physics_world_t* world,
                                        body_handle_t handle, vec3_t force,
                                        vec3_t world_point) {
  rigid_body_t* body = physics_body(world, handle);
  if (!body) return;
  body->force = vec3_add(body->force, force);
  body->torque = vec3_add(
      body->torque, vec3_cross(vec3_sub(world_point, body->position), force));
}

static vec3_t inverse_inertia_apply(const rigid_body_t* body, vec3_t v) {
  vec3_t local = quat_rotate(quat_conjugate(body->rotation), v);
  local = vec3_multiply(local, body->inverse_inertia_local);
  return quat_rotate(body->rotation, local);
}

static void apply_impulse(rigid_body_t* body, vec3_t impulse,
                          vec3_t world_point) {
  body->velocity = vec3_add_scaled(body->velocity, impulse, body->inverse_mass);
  vec3_t arm = vec3_sub(world_point, body->position);
  body->angular_velocity =
      vec3_add(body->angular_velocity,
               inverse_inertia_apply(body, vec3_cross(arm, impulse)));
}

void physics_apply_impulse_at_point(physics_world_t* world,
                                    body_handle_t handle, vec3_t impulse,
                                    vec3_t world_point) {
  rigid_body_t* body = physics_body(world, handle);
  if (body) apply_impulse(body, impulse, world_point);
}

vec3_t physics_body_velocity_at_point(const rigid_body_t* body,
                                      vec3_t world_point) {
  vec3_t arm = vec3_sub(world_point, body->position);
  return vec3_add(body->velocity, vec3_cross(body->angular_velocity, arm));
}

transform_t physics_body_interpolated(const rigid_body_t* body, float alpha) {
  return transform_lerp(body->previous,
                        transform_at(body->position, body->rotation), alpha);
}

/* ---- queries ---- */

static bool ray_collider(const static_collider_t* collider, ray_t ray,
                         float max_distance, ray_hit_t* best) {
  float t_min, t_max;
  if (!ray_aabb(ray, collider->bounds, &t_min, &t_max) || t_min > max_distance)
    return false;
  bool found = false;
  if (collider->shape == COLLIDER_SHAPE_BOX) {
    /* Cast in the box frame against the axis-aligned box. */
    ray_t local;
    local.origin = transform_inverse_point(collider->transform, ray.origin);
    local.direction = quat_rotate(quat_conjugate(collider->transform.rotation),
                                  ray.direction);
    aabb_t box = aabb_from_center(vec3_zero(), collider->half_extents);
    if (!ray_aabb(local, box, &t_min, &t_max)) return false;
    float t = t_min >= 0 ? t_min : t_max;
    if (t < 0 || t > max_distance || t >= best->distance) return false;
    vec3_t p = vec3_add_scaled(local.origin, local.direction, t);
    /* The normal is the axis of the face hit, found by the largest ratio. */
    vec3_t ratio = vec3(fabsf(p.x) / collider->half_extents.x,
                        fabsf(p.y) / collider->half_extents.y,
                        fabsf(p.z) / collider->half_extents.z);
    vec3_t normal;
    if (ratio.x >= ratio.y && ratio.x >= ratio.z)
      normal = vec3(p.x > 0 ? 1.0f : -1.0f, 0, 0);
    else if (ratio.y >= ratio.z)
      normal = vec3(0, p.y > 0 ? 1.0f : -1.0f, 0);
    else
      normal = vec3(0, 0, p.z > 0 ? 1.0f : -1.0f);
    best->distance = t;
    best->point = vec3_add_scaled(ray.origin, ray.direction, t);
    best->normal = quat_rotate(collider->transform.rotation, normal);
    if (vec3_dot(best->normal, ray.direction) > 0)
      best->normal = vec3_negate(best->normal);
    return true;
  }
  for (size_t i = 0; i + 2 < collider->index_count; i += 3) {
    ray_hit_t hit;
    if (ray_triangle(ray, collider->vertices[collider->indices[i]],
                     collider->vertices[collider->indices[i + 1]],
                     collider->vertices[collider->indices[i + 2]], &hit) &&
        hit.distance <= max_distance && hit.distance < best->distance) {
      *best = hit;
      found = true;
    }
  }
  return found;
}

bool physics_ray_cast(const physics_world_t* const_world, ray_t ray,
                      float max_distance, uint32_t layer_mask,
                      ray_cast_result_t* result) {
  /* The grid's scratch lists are reused; they are not part of the world's
   * state as far as the caller can tell. */
  physics_world_t* world = (physics_world_t*)const_world;
  ray_hit_t best;
  best.distance = INFINITY;
  bool found = false;
  vec3_t end = vec3_add_scaled(ray.origin, ray.direction, max_distance);
  aabb_t swept = aabb_empty();
  swept = aabb_include(swept, ray.origin);
  swept = aabb_include(swept, end);
  gather(world, swept);
  for (size_t k = 0; k < world->candidate_count; k++) {
    size_t i = world->candidates[k];
    const static_collider_t* collider = &world->colliders[i];
    if (!collider->used || !(collider->layer & layer_mask)) continue;
    if (ray_collider(collider, ray, max_distance, &best)) {
      found = true;
      result->collider.id = (uint32_t)i + 1;
      result->layer = collider->layer;
      result->friction = collider->friction;
      result->source_id = collider->source_id;
    }
  }
  if (found) result->hit = best;
  return found;
}

/* ---- stepping ---- */

typedef struct {
  rigid_body_t* body;
  body_handle_t handle;
  vec3_t point;
  vec3_t normal;
  float penetration;
  float friction;
  collider_handle_t collider;
  float accumulated_impulse;
  vec3_t accumulated_friction;
} contact_t;

static void integrate(rigid_body_t* body, vec3_t gravity, float dt) {
  body->previous = transform_at(body->position, body->rotation);
  vec3_t acceleration =
      vec3_add(gravity, vec3_scale(body->force, body->inverse_mass));
  body->velocity = vec3_add_scaled(body->velocity, acceleration, dt);
  body->angular_velocity = vec3_add_scaled(
      body->angular_velocity, inverse_inertia_apply(body, body->torque), dt);
  body->velocity =
      vec3_scale(body->velocity, 1.0f / (1.0f + body->linear_damping * dt));
  body->angular_velocity = vec3_scale(
      body->angular_velocity, 1.0f / (1.0f + body->angular_damping * dt));
  body->position = vec3_add_scaled(body->position, body->velocity, dt);
  body->rotation = quat_integrate(body->rotation, body->angular_velocity, dt);
  body->force = vec3_zero();
  body->torque = vec3_zero();
}

static size_t find_contacts(physics_world_t* world, rigid_body_t* body,
                            body_handle_t handle, contact_t* contacts,
                            size_t capacity) {
  size_t count = 0;
  aabb_t body_bounds =
      aabb_transform(aabb_from_center(vec3_zero(), body->half_extents),
                     transform_at(body->position, body->rotation));
  gather(world, body_bounds);
  for (int corner = 0; corner < 8 && count < capacity; corner++) {
    vec3_t h = body->half_extents;
    vec3_t local = vec3((corner & 1) ? h.x : -h.x, (corner & 2) ? h.y : -h.y,
                        (corner & 4) ? h.z : -h.z);
    vec3_t world_corner =
        vec3_add(body->position, quat_rotate(body->rotation, local));
    vec3_t offset = vec3_sub(world_corner, body->position);
    float reach = vec3_length(offset);
    ray_t ray = {body->position, vec3_scale(offset, 1.0f / reach)};
    ray_hit_t best;
    best.distance = INFINITY;
    size_t hit_index = 0;
    bool found = false;
    for (size_t k = 0; k < world->candidate_count; k++) {
      size_t i = world->candidates[k];
      const static_collider_t* collider = &world->colliders[i];
      if (!collider->used || !aabb_overlaps(body_bounds, collider->bounds))
        continue;
      if (ray_collider(collider, ray, reach + CONTACT_MARGIN, &best)) {
        found = true;
        hit_index = i;
      }
    }
    if (!found) continue;
    contact_t* contact = &contacts[count++];
    contact->body = body;
    contact->handle = handle;
    contact->point = world_corner;
    contact->normal = best.normal;
    contact->penetration = reach - best.distance;
    contact->friction = world->colliders[hit_index].friction;
    contact->collider.id = (uint32_t)hit_index + 1;
    contact->accumulated_impulse = 0;
    contact->accumulated_friction = vec3_zero();
  }
  return count;
}

static void solve_contact(contact_t* contact) {
  rigid_body_t* body = contact->body;
  vec3_t arm = vec3_sub(contact->point, body->position);
  vec3_t velocity = physics_body_velocity_at_point(body, contact->point);
  float normal_velocity = vec3_dot(velocity, contact->normal);
  vec3_t angular_term = vec3_cross(
      inverse_inertia_apply(body, vec3_cross(arm, contact->normal)), arm);
  float effective_mass =
      body->inverse_mass + vec3_dot(angular_term, contact->normal);
  if (effective_mass <= 0) return;
  float restitution = normal_velocity < -1.0f ? body->restitution : 0.0f;
  float wanted = -(1.0f + restitution) * normal_velocity;
  float impulse = wanted / effective_mass;
  float previous = contact->accumulated_impulse;
  contact->accumulated_impulse = fmaxf(previous + impulse, 0.0f);
  impulse = contact->accumulated_impulse - previous;
  apply_impulse(body, vec3_scale(contact->normal, impulse), contact->point);

  /* Coulomb friction: the accumulated tangential impulse over all
   * iterations stays inside the friction cone of the accumulated normal
   * impulse. */
  velocity = physics_body_velocity_at_point(body, contact->point);
  vec3_t tangent_velocity = vec3_sub(
      velocity,
      vec3_scale(contact->normal, vec3_dot(velocity, contact->normal)));
  float tangent_speed = vec3_length(tangent_velocity);
  if (tangent_speed < 1e-6f || contact->accumulated_impulse <= 0) return;
  vec3_t tangent = vec3_scale(tangent_velocity, 1.0f / tangent_speed);
  vec3_t tangent_angular =
      vec3_cross(inverse_inertia_apply(body, vec3_cross(arm, tangent)), arm);
  float tangent_mass = body->inverse_mass + vec3_dot(tangent_angular, tangent);
  vec3_t old_friction = contact->accumulated_friction;
  vec3_t total =
      vec3_add_scaled(old_friction, tangent, -tangent_speed / tangent_mass);
  float limit = contact->friction * contact->accumulated_impulse;
  float magnitude = vec3_length(total);
  if (magnitude > limit) total = vec3_scale(total, limit / magnitude);
  contact->accumulated_friction = total;
  apply_impulse(body, vec3_sub(total, old_friction), contact->point);
}

/* Resting contacts sit within the slop; deeper ones are eased out over a
 * few steps without touching the velocity, so a resting body reports rest. */
static void correct_penetration(contact_t* contacts, size_t count) {
  for (size_t i = 0; i < count; i++) {
    float depth = contacts[i].penetration - PENETRATION_SLOP;
    if (depth <= 0) continue;
    rigid_body_t* body = contacts[i].body;
    body->position = vec3_add_scaled(body->position, contacts[i].normal,
                                     depth * PENETRATION_CORRECTION);
  }
}

void physics_world_step(physics_world_t* world, float dt) {
  world->contact_count = 0;
  for (uint32_t i = 0; i < MAX_BODIES; i++) {
    rigid_body_t* body = &world->bodies[i];
    if (!body->used) continue;
    body_handle_t handle = {i + 1};
    integrate(body, world->gravity, dt);
    contact_t contacts[8];
    size_t count = find_contacts(world, body, handle, contacts, 8);
    for (int iteration = 0; iteration < SOLVER_ITERATIONS; iteration++)
      for (size_t c = 0; c < count; c++) solve_contact(&contacts[c]);
    correct_penetration(contacts, count);
    for (size_t c = 0; c < count && world->contact_count < MAX_CONTACTS; c++) {
      if (contacts[c].accumulated_impulse <= 0) continue;
      contact_event_t* event = &world->contacts[world->contact_count++];
      event->body = handle;
      event->point = contacts[c].point;
      event->normal = contacts[c].normal;
      event->impulse = contacts[c].accumulated_impulse;
      event->collider = contacts[c].collider;
    }
  }
}

size_t physics_contact_count(const physics_world_t* world) {
  return world->contact_count;
}

const contact_event_t* physics_contact_at(const physics_world_t* world,
                                          size_t index) {
  return index < world->contact_count ? &world->contacts[index] : NULL;
}
