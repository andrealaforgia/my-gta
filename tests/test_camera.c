#include <assert.h>
#include <math.h>

#include "chase_camera.h"
#include "physics_world.h"
#include "test_groups.h"
#include "test_support.h"

static const float step = 1.0f / 60.0f;

static void the_camera_settles_behind_and_above_the_car(void) {
  chase_camera_t camera = chase_camera_create(chase_camera_default_params());
  transform_t car = transform_at(vec3(10, 0.5f, -20), quat_from_yaw(0.3f));
  chase_camera_reset(&camera, car);
  for (int i = 0; i < 300; i++) chase_camera_update(&camera, car, vec3_zero(), NULL, step);
  render_camera_t view = chase_camera_render(&camera);
  vec3_t to_camera = vec3_sub(view.eye, car.position);
  vec3_t forward = quat_forward(car.rotation);
  assert(vec3_dot(vec3_normalize(to_camera), forward) < -0.8f);
  assert(view.eye.y > car.position.y + 1.0f);
  assert(near_to(vec3_length(vec3(to_camera.x, 0, to_camera.z)), camera.params.distance, 0.05f));
  assert(vec3_dot(vec3_normalize(vec3_sub(view.target, view.eye)), forward) > 0.5f);
}

static void the_camera_follows_a_moving_car_without_running_away(void) {
  chase_camera_t camera = chase_camera_create(chase_camera_default_params());
  transform_t car = transform_at(vec3(0, 0.5f, 0), quat_identity());
  chase_camera_reset(&camera, car);
  vec3_t velocity = vec3(0, 0, -25);
  for (int i = 0; i < 300; i++) {
    car.position = vec3_add_scaled(car.position, velocity, step);
    chase_camera_update(&camera, car, velocity, NULL, step);
  }
  render_camera_t view = chase_camera_render(&camera);
  float gap = vec3_distance(view.eye, car.position);
  assert(gap < camera.params.distance * 1.8f);
  assert(view.eye.z > car.position.z);
  assert(view.target.z < car.position.z); /* looks ahead at speed */
  assert(!isnan(view.eye.x) && !isnan(view.target.x));
}

static void a_wall_behind_the_car_pulls_the_camera_in(void) {
  physics_world_t* world = physics_world_create();
  physics_add_static_box(world, transform_at(vec3(0, 2, 3), quat_identity()), vec3(5, 2, 0.25f),
                         COLLISION_LAYER_BUILDING, 0.5f, 0);
  chase_camera_t camera = chase_camera_create(chase_camera_default_params());
  transform_t car = transform_at(vec3(0, 0.5f, 0), quat_identity());
  chase_camera_reset(&camera, car);
  for (int i = 0; i < 120; i++) chase_camera_update(&camera, car, vec3_zero(), world, step);
  render_camera_t view = chase_camera_render(&camera);
  assert(view.eye.z < 2.8f);
  assert(view.eye.z > 0.5f);
  physics_world_destroy(world);
}

static void reversing_keeps_the_camera_behind_the_nose(void) {
  chase_camera_t camera = chase_camera_create(chase_camera_default_params());
  transform_t car = transform_at(vec3(0, 0.5f, 0), quat_identity());
  chase_camera_reset(&camera, car);
  vec3_t velocity = vec3(0, 0, 5);
  for (int i = 0; i < 180; i++) {
    car.position = vec3_add_scaled(car.position, velocity, step);
    chase_camera_update(&camera, car, velocity, NULL, step);
  }
  render_camera_t view = chase_camera_render(&camera);
  assert(view.eye.z > car.position.z);
}

void run_camera_tests(void) {
  RUN(the_camera_settles_behind_and_above_the_car);
  RUN(the_camera_follows_a_moving_car_without_running_away);
  RUN(a_wall_behind_the_car_pulls_the_camera_in);
  RUN(reversing_keeps_the_camera_behind_the_nose);
}
