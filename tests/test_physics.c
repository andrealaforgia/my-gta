#include <assert.h>
#include <math.h>

#include "physics_world.h"
#include "test_groups.h"
#include "test_support.h"

static const float step = 1.0f / 120.0f;

static physics_world_t* world_with_flat_ground(void) {
  physics_world_t* world = physics_world_create();
  /* Two triangles, anticlockwise seen from above, 100 m square. */
  vec3_t vertices[4] = {vec3(-50, 0, 50), vec3(50, 0, 50), vec3(50, 0, -50),
                        vec3(-50, 0, -50)};
  uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
  physics_add_static_mesh(world, vertices, 4, indices, 6,
                          COLLISION_LAYER_GROUND, 1.0f, 0);
  return world;
}

static void simulate(physics_world_t* world, float seconds) {
  int steps = (int)(seconds / step);
  for (int i = 0; i < steps; i++) physics_world_step(world, step);
}

static void a_dropped_box_lands_on_the_ground_and_rests(void) {
  physics_world_t* world = world_with_flat_ground();
  body_handle_t handle = physics_add_body(world, vec3(0, 3, 0), quat_identity(),
                                          100.0f, vec3(0.5f, 0.25f, 1.0f));
  simulate(world, 3.0f);
  const rigid_body_t* body = physics_body(world, handle);
  assert(near_to(body->position.y, 0.25f, 0.02f));
  assert(fabsf(body->velocity.y) < 0.05f);
  assert(vec3_length(body->angular_velocity) < 0.05f);
  assert(near_to(quat_up(body->rotation).y, 1.0f, 1e-3));
  physics_world_destroy(world);
}

static void landing_reports_a_contact_with_an_impulse(void) {
  physics_world_t* world = world_with_flat_ground();
  physics_add_body(world, vec3(0, 2, 0), quat_identity(), 100.0f,
                   vec3(0.5f, 0.25f, 1.0f));
  float largest = 0;
  for (int i = 0; i < 240; i++) {
    physics_world_step(world, step);
    for (size_t c = 0; c < physics_contact_count(world); c++) {
      const contact_event_t* contact = physics_contact_at(world, c);
      if (contact->impulse > largest) largest = contact->impulse;
      assert(contact->normal.y > 0.9f);
    }
  }
  /* Falling 1.75 m gives about 5.9 m/s; the impulse to stop 100 kg is
   * spread over the corners that touch first. */
  assert(largest > 50.0f && largest < 700.0f);
  physics_world_destroy(world);
}

static void a_sliding_box_is_stopped_by_a_wall(void) {
  physics_world_t* world = world_with_flat_ground();
  physics_add_static_box(world, transform_at(vec3(10, 1, 0), quat_identity()),
                         vec3(0.5f, 1, 5), COLLISION_LAYER_BARRIER, 0.6f, 0);
  body_handle_t handle =
      physics_add_body(world, vec3(0, 0.25f, 0), quat_identity(), 100.0f,
                       vec3(0.5f, 0.25f, 1.0f));
  rigid_body_t* body = physics_body(world, handle);
  body->velocity = vec3(15, 0, 0);
  simulate(world, 2.0f);
  assert(body->position.x < 9.5f);
  assert(body->position.x > 7.0f);
  assert(body->position.y > 0.1f);
  physics_world_destroy(world);
}

static void friction_brings_a_sliding_box_to_rest(void) {
  physics_world_t* world = world_with_flat_ground();
  body_handle_t handle =
      physics_add_body(world, vec3(0, 0.25f, 0), quat_identity(), 100.0f,
                       vec3(0.5f, 0.25f, 1.0f));
  rigid_body_t* body = physics_body(world, handle);
  body->velocity = vec3(5, 0, 0);
  simulate(world, 4.0f);
  assert(vec3_length(body->velocity) < 0.1f);
  assert(body->position.x > 0.5f);
  physics_world_destroy(world);
}

static void ray_casts_report_the_surface_its_friction_and_layer(void) {
  physics_world_t* world = world_with_flat_ground();
  physics_add_static_box(world, transform_at(vec3(0, 2, -5), quat_identity()),
                         vec3(1, 1, 1), COLLISION_LAYER_BUILDING, 0.5f, 4242);
  ray_t down = {vec3(3, 10, 3), vec3(0, -1, 0)};
  ray_cast_result_t result;
  assert(physics_ray_cast(world, down, 100, COLLISION_LAYER_ALL, &result));
  assert(near(result.hit.distance, 10));
  assert(near(result.friction, 1.0f));
  assert(result.layer == COLLISION_LAYER_GROUND);
  ray_t north = {vec3(0, 2, 0), vec3(0, 0, -1)};
  assert(physics_ray_cast(world, north, 100, COLLISION_LAYER_ALL, &result));
  assert(near(result.hit.distance, 4));
  assert(result.layer == COLLISION_LAYER_BUILDING);
  assert(result.source_id == 4242);
  assert(near_to(result.hit.normal.z, 1, 1e-5));
  assert(!physics_ray_cast(world, north, 100, COLLISION_LAYER_GROUND, &result));
  assert(!physics_ray_cast(world, north, 3, COLLISION_LAYER_ALL, &result));
  physics_world_destroy(world);
}

static void forces_at_a_point_turn_the_body(void) {
  physics_world_t* world = physics_world_create();
  physics_world_set_gravity(world, vec3(0, 0, 0));
  body_handle_t handle = physics_add_body(world, vec3(0, 0, 0), quat_identity(),
                                          100.0f, vec3(0.5f, 0.25f, 1.0f));
  for (int i = 0; i < 60; i++) {
    physics_apply_force_at_point(world, handle, vec3(100, 0, 0),
                                 vec3(0, 0, -1));
    physics_world_step(world, step);
  }
  const rigid_body_t* body = physics_body(world, handle);
  assert(body->velocity.x > 0);
  /* A push towards +X at the front (-Z) turns the nose to the right:
   * clockwise from above, so negative yaw rate. */
  assert(body->angular_velocity.y < 0);
  physics_world_destroy(world);
}

static void previous_state_is_kept_for_interpolation(void) {
  physics_world_t* world = world_with_flat_ground();
  body_handle_t handle = physics_add_body(world, vec3(0, 5, 0), quat_identity(),
                                          100.0f, vec3(0.5f, 0.25f, 1.0f));
  physics_world_step(world, step);
  const rigid_body_t* body = physics_body(world, handle);
  assert(near(body->previous.position.y, 5));
  assert(body->position.y < 5);
  transform_t halfway = physics_body_interpolated(body, 0.5f);
  assert(halfway.position.y < 5 && halfway.position.y > body->position.y);
  physics_world_destroy(world);
}

static void colliders_can_be_listed_for_debug_drawing(void) {
  physics_world_t* world = world_with_flat_ground();
  physics_add_static_box(world, transform_at(vec3(1, 2, 3), quat_identity()),
                         vec3(1, 1, 1), COLLISION_LAYER_PROP, 0.5f, 7);
  assert(physics_collider_count(world) == 2);
  const static_collider_t* box = physics_collider_at(world, 1);
  assert(box->shape == COLLIDER_SHAPE_BOX);
  assert(near(box->transform.position.z, 3));
  assert(aabb_contains(box->bounds, vec3(1.5f, 2.5f, 3.5f)));
  physics_world_destroy(world);
}

static void a_moved_box_collider_blocks_rays_where_it_now_stands(void) {
  physics_world_t* world = physics_world_create();
  collider_handle_t box = physics_add_static_box(
      world, transform_at(vec3(0, 1, 0), quat_identity()), vec3(1, 1, 2),
      COLLISION_LAYER_PROP, 0.5f, 7);
  ray_t ray = {vec3(10, 1, 0), vec3(-1, 0, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(world, ray, 50, COLLISION_LAYER_ALL, &hit));
  assert(near_to(hit.hit.point.x, 1.0f, 1e-3f));
  physics_set_static_box_transform(
      world, box, transform_at(vec3(0, 1, 20), quat_identity()));
  assert(!physics_ray_cast(world, ray, 50, COLLISION_LAYER_ALL, &hit));
  ray_t moved = {vec3(10, 1, 20), vec3(-1, 0, 0)};
  assert(physics_ray_cast(world, moved, 50, COLLISION_LAYER_ALL, &hit));
  assert(near_to(hit.hit.point.x, 1.0f, 1e-3f) && hit.source_id == 7);
  /* Turned a quarter, its long side now faces along x. */
  physics_set_static_box_transform(
      world, box,
      transform_at(vec3(0, 1, 20), quat_from_yaw((float)M_PI * 0.5f)));
  assert(physics_ray_cast(world, moved, 50, COLLISION_LAYER_ALL, &hit));
  assert(near_to(hit.hit.point.x, 2.0f, 1e-2f));
  physics_world_destroy(world);
}

static void the_collider_grid_finds_what_is_near_and_follows_moves_and_removals(
    void) {
  physics_world_t* world = physics_world_create();
  /* A 5 km ground, far bigger than a grid cell, and a lattice of boxes. */
  vec3_t ground[4] = {vec3(-2500, 0, 2500), vec3(2500, 0, 2500),
                      vec3(2500, 0, -2500), vec3(-2500, 0, -2500)};
  uint32_t ground_indices[6] = {0, 1, 2, 0, 2, 3};
  physics_add_static_mesh(world, ground, 4, ground_indices, 6,
                          COLLISION_LAYER_GROUND, 1.0f, 0);
  collider_handle_t boxes[40 * 40];
  for (int j = 0; j < 40; j++)
    for (int i = 0; i < 40; i++)
      boxes[j * 40 + i] = physics_add_static_box(
          world,
          transform_at(vec3((float)(i * 10 - 200), 1.0f + 0.01f * (float)i,
                            (float)(j * 10 - 200)),
                       quat_identity()),
          vec3(1, 1, 1), COLLISION_LAYER_PROP, 0.5f, (uint64_t)(j * 40 + i));
  ray_cast_result_t hit;
  /* Down onto a box top, and onto bare ground between the boxes. */
  ray_t on_box = {vec3(70, 50, -30), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, on_box, 100.0f, COLLISION_LAYER_ALL, &hit));
  assert(hit.layer == COLLISION_LAYER_PROP &&
         fabsf(hit.hit.point.y - 2.27f) < 0.001f);
  assert(hit.source_id == (uint64_t)(17 * 40 + 27));
  ray_t between = {vec3(75, 50, -25), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, between, 100.0f, COLLISION_LAYER_ALL, &hit));
  assert(hit.layer == COLLISION_LAYER_GROUND &&
         fabsf(hit.hit.point.y) < 0.001f);
  /* Sideways along a row, through a few cells. */
  ray_t along = {vec3(-400, 1, -30), vec3(1, 0, 0)};
  assert(physics_ray_cast(world, along, 1000.0f, COLLISION_LAYER_PROP, &hit));
  assert(fabsf(hit.hit.point.x + 201.0f) < 0.001f);
  /* Far from everything but the ground. */
  ray_t away = {vec3(2000, 5, 2000), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, away, 20.0f, COLLISION_LAYER_PROP, &hit) ==
         false);
  assert(physics_ray_cast(world, away, 20.0f, COLLISION_LAYER_GROUND, &hit));
  /* A box moved across many cells is found where it is now, not where it was.
   */
  physics_set_static_box_transform(
      world, boxes[17 * 40 + 27],
      transform_at(vec3(1500, 3, 1500), quat_identity()));
  assert(physics_ray_cast(world, on_box, 100.0f, COLLISION_LAYER_PROP, &hit) ==
         false);
  ray_t there = {vec3(1500, 50, 1500), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, there, 100.0f, COLLISION_LAYER_PROP, &hit));
  assert(fabsf(hit.hit.point.y - 4.0f) < 0.001f);
  /* Removed, it is gone; its slot is used again by the next one. */
  physics_remove_collider(world, boxes[17 * 40 + 27]);
  assert(physics_ray_cast(world, there, 100.0f, COLLISION_LAYER_PROP, &hit) ==
         false);
  size_t before = physics_collider_count(world);
  collider_handle_t again = physics_add_static_box(
      world, transform_at(vec3(-1800, 1, 900), quat_identity()), vec3(1, 1, 1),
      COLLISION_LAYER_PROP, 0.5f, 99999);
  assert(again.id == boxes[17 * 40 + 27].id &&
         physics_collider_count(world) == before);
  ray_t reused = {vec3(-1800, 50, 900), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, reused, 100.0f, COLLISION_LAYER_PROP, &hit));
  assert(hit.source_id == 99999);
  /* A body dropped among them still lands on a box and rests. */
  body_handle_t body =
      physics_add_body(world, vec3(-200, 6, -200), quat_identity(), 100.0f,
                       vec3(0.4f, 0.25f, 0.4f));
  simulate(world, 3.0f);
  assert(physics_body(world, body)->position.y > 1.9f);
  physics_world_destroy(world);
}

void run_physics_tests(void) {
  RUN(a_moved_box_collider_blocks_rays_where_it_now_stands);
  RUN(a_dropped_box_lands_on_the_ground_and_rests);
  RUN(landing_reports_a_contact_with_an_impulse);
  RUN(a_sliding_box_is_stopped_by_a_wall);
  RUN(friction_brings_a_sliding_box_to_rest);
  RUN(ray_casts_report_the_surface_its_friction_and_layer);
  RUN(forces_at_a_point_turn_the_body);
  RUN(previous_state_is_kept_for_interpolation);
  RUN(colliders_can_be_listed_for_debug_drawing);
  RUN(the_collider_grid_finds_what_is_near_and_follows_moves_and_removals);
}
