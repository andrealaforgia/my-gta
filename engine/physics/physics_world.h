/**
 * @file physics_world.h
 * @brief The narrow physics module of ADR-003: dynamic box bodies, static
 * box and triangle-mesh colliders, ray casts, contacts and a fixed step.
 * Contacts are found by casting from each body's centre to its corners,
 * so a body whose centre is inside static geometry will not be pushed out;
 * vehicles keep their chassis clear of the ground on their wheels.
 */
#ifndef ENGINE_PHYSICS_PHYSICS_WORLD_H_
#define ENGINE_PHYSICS_PHYSICS_WORLD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aabb.h"
#include "quat.h"
#include "ray.h"
#include "transform.h"
#include "vec3.h"

typedef enum {
  COLLISION_LAYER_GROUND = 1,
  COLLISION_LAYER_BUILDING = 2,
  COLLISION_LAYER_BARRIER = 4,
  COLLISION_LAYER_PROP = 8,
  COLLISION_LAYER_ALL = 0xffff,
} collision_layer_t;

typedef enum { COLLIDER_SHAPE_BOX, COLLIDER_SHAPE_MESH } collider_shape_t;

typedef struct {
  uint32_t id; /* 0 is "none" */
} body_handle_t;

typedef struct {
  uint32_t id; /* 0 is "none" */
} collider_handle_t;

typedef struct {
  collider_shape_t shape;
  transform_t transform;  /* boxes */
  vec3_t half_extents;    /* boxes */
  vec3_t* vertices;       /* meshes, world space */
  size_t vertex_count;
  uint32_t* indices;
  size_t index_count;
  aabb_t bounds;
  collision_layer_t layer;
  float friction;
  uint64_t source_id; /* for diagnostics: a feature or entity id */
  bool used;
} static_collider_t;

typedef struct {
  vec3_t position;
  quat_t rotation;
  vec3_t velocity;
  vec3_t angular_velocity;
  float mass;
  float inverse_mass;
  vec3_t inverse_inertia_local; /* diagonal, body frame */
  vec3_t half_extents;
  vec3_t force;
  vec3_t torque;
  float linear_damping;
  float angular_damping;
  float restitution;
  transform_t previous;
  bool used;
} rigid_body_t;

typedef struct {
  body_handle_t body;
  vec3_t point;
  vec3_t normal;
  float impulse;
  collider_handle_t collider;
} contact_event_t;

typedef struct {
  ray_hit_t hit;
  collider_handle_t collider;
  collision_layer_t layer;
  float friction;
  uint64_t source_id;
} ray_cast_result_t;

typedef struct physics_world physics_world_t;

physics_world_t* physics_world_create(void);
void physics_world_destroy(physics_world_t* world);
void physics_world_set_gravity(physics_world_t* world, vec3_t gravity);
vec3_t physics_world_gravity(const physics_world_t* world);

collider_handle_t physics_add_static_box(physics_world_t* world, transform_t transform,
                                         vec3_t half_extents, collision_layer_t layer,
                                         float friction, uint64_t source_id);
/* Vertices are copied; indices are triangles anticlockwise from the front. */
collider_handle_t physics_add_static_mesh(physics_world_t* world, const vec3_t* vertices,
                                          size_t vertex_count, const uint32_t* indices,
                                          size_t index_count, collision_layer_t layer,
                                          float friction, uint64_t source_id);
void physics_remove_collider(physics_world_t* world, collider_handle_t collider);
size_t physics_collider_count(const physics_world_t* world);
const static_collider_t* physics_collider_at(const physics_world_t* world, size_t index);

body_handle_t physics_add_body(physics_world_t* world, vec3_t position, quat_t rotation,
                               float mass, vec3_t half_extents);
void physics_remove_body(physics_world_t* world, body_handle_t body);
rigid_body_t* physics_body(physics_world_t* world, body_handle_t body);
void physics_body_teleport(physics_world_t* world, body_handle_t body, vec3_t position,
                           quat_t rotation);
void physics_apply_force(physics_world_t* world, body_handle_t body, vec3_t force);
/* World-space force at a point given in the body frame. */
void physics_apply_force_at_point(physics_world_t* world, body_handle_t body, vec3_t force,
                                  vec3_t local_point);
void physics_apply_force_at_world_point(physics_world_t* world, body_handle_t body,
                                        vec3_t force, vec3_t world_point);
void physics_apply_impulse_at_point(physics_world_t* world, body_handle_t body, vec3_t impulse,
                                    vec3_t world_point);
vec3_t physics_body_velocity_at_point(const rigid_body_t* body, vec3_t world_point);
transform_t physics_body_interpolated(const rigid_body_t* body, float alpha);

bool physics_ray_cast(const physics_world_t* world, ray_t ray, float max_distance,
                      uint32_t layer_mask, ray_cast_result_t* result);

void physics_world_step(physics_world_t* world, float dt);
size_t physics_contact_count(const physics_world_t* world);
const contact_event_t* physics_contact_at(const physics_world_t* world, size_t index);

#endif  // ENGINE_PHYSICS_PHYSICS_WORLD_H_
