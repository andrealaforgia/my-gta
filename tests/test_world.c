#include <assert.h>
#include <string.h>

#include "test_groups.h"
#include "test_support.h"
#include "world.h"

static int count_visible(const entity_t* entity, void* context) {
  (void)entity;
  int* count = context;
  (*count)++;
  return 1;
}

static void entities_get_stable_ids_and_world_bounds(void) {
  world_t world = world_create();
  mesh_handle_t mesh = {1};
  aabb_t local = aabb_from_center(vec3_zero(), vec3(1, 1, 1));
  uint32_t first =
      world_add(&world, transform_at(vec3(10, 0, 0), quat_identity()), mesh,
                material_plain(vec3(1, 0, 0)), local, "first");
  uint32_t second =
      world_add(&world, transform_at(vec3(-10, 0, 0), quat_identity()), mesh,
                material_plain(vec3(0, 1, 0)), local, "second");
  assert(first != second);
  assert(world_count(&world) == 2);
  entity_t* entity = world_entity(&world, second);
  assert(entity && strcmp(entity->label, "second") == 0);
  assert(near(entity->world_bounds.min.x, -11) &&
         near(entity->world_bounds.max.x, -9));
  world_remove(&world, first);
  assert(world_count(&world) == 1);
  assert(world_entity(&world, first) == NULL);
  world_destroy(&world);
}

static void only_entities_inside_the_frustum_are_visited(void) {
  world_t world = world_create();
  mesh_handle_t mesh = {1};
  aabb_t local = aabb_from_center(vec3_zero(), vec3(1, 1, 1));
  world_add(&world, transform_at(vec3(0, 0, -20), quat_identity()), mesh,
            material_plain(vec3(1, 1, 1)), local, "ahead");
  world_add(&world, transform_at(vec3(0, 0, 20), quat_identity()), mesh,
            material_plain(vec3(1, 1, 1)), local, "behind");
  world_add(&world, transform_at(vec3(200, 0, -20), quat_identity()), mesh,
            material_plain(vec3(1, 1, 1)), local, "far right");
  mat4_t view = mat4_look_at(vec3(0, 1, 0), vec3(0, 1, -10), vec3(0, 1, 0));
  mat4_t projection = mat4_perspective(1.0f, 1.5f, 0.1f, 500.0f);
  frustum_t frustum = frustum_from_matrix(mat4_multiply(projection, view));
  int visible = 0;
  world_for_each_visible(&world, frustum, count_visible, &visible);
  assert(visible == 1);
  world_destroy(&world);
}

static void moving_an_entity_updates_its_bounds(void) {
  world_t world = world_create();
  mesh_handle_t mesh = {1};
  aabb_t local = aabb_from_center(vec3_zero(), vec3(1, 1, 1));
  uint32_t id = world_add(&world, transform_identity(), mesh,
                          material_plain(vec3(1, 1, 1)), local, "mover");
  world_set_transform(&world, id, transform_at(vec3(5, 5, 5), quat_identity()));
  assert(near(world_entity(&world, id)->world_bounds.min.y, 4));
  world_destroy(&world);
}

static void rendering_draws_only_what_the_camera_sees(void) {
  world_t world = world_create();
  renderer_t* renderer = renderer_create(1280, 720);
  mesh_data_t cube = mesh_data_create();
  mesh_data_add_box(&cube, vec3_zero(), vec3(1, 1, 1));
  mesh_handle_t mesh = renderer_upload_mesh(renderer, &cube);
  aabb_t local = mesh_data_bounds(&cube);
  world_add(&world, transform_at(vec3(0, 0, -20), quat_identity()), mesh,
            material_plain(vec3(1, 1, 1)), local, "ahead");
  world_add(&world, transform_at(vec3(0, 0, 20), quat_identity()), mesh,
            material_plain(vec3(1, 1, 1)), local, "behind");
  render_camera_t camera = {vec3(0, 1, 0), vec3(0, 1, -10), vec3(0, 1, 0), 1.0f,
                            0.1f,          500.0f};
  render_environment_t environment = {
      {0, 1, 0},  {1, 1, 1}, 0.3f, {0.5f, 0.6f, 0.8f},
      100,        400,       0,    {0.3f, 0.5f, 0.8f},
      0,          0,         0,    {0, 0, 0},
      {0, 0, -1}, 0,         0.3f, 0};
  renderer_begin_frame(renderer, camera, environment);
  assert(world_render(&world, renderer) == 1);
  assert(renderer_stats(renderer).draw_calls == 1);
  renderer_end_frame(renderer);
  mesh_data_free(&cube);
  renderer_destroy(renderer);
  world_destroy(&world);
}

void run_world_tests(void) {
  RUN(entities_get_stable_ids_and_world_bounds);
  RUN(rendering_draws_only_what_the_camera_sees);
  RUN(only_entities_inside_the_frustum_are_visited);
  RUN(moving_an_entity_updates_its_bounds);
}
