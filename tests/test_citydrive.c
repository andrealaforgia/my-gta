/* Drives the car through the city generated from the fixture, using the
 * stub renderer: the road surfaces, junction patches and building walls
 * must hold the car up and stop it, with no gaps and no NaNs. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "city_generator.h"
#include "city_import.h"
#include "city_package.h"
#include "driving_stage.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"

static const char* package_directory = "/tmp/gta-like-test-citydrive";

static void cook_fixture_city(void) {
  osm_data_t osm;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(),
                     "tests/fixtures/district.osm"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "district", &stats));
  assert(city_package_write(&package, package_directory));
  city_package_free(&package);
  city_model_free(&model);
}

typedef struct {
  window_t window;
  renderer_t* renderer;
  driving_stage_t stage;
} city_run_t;

static void start(city_run_t* run) {
  memset(&run->window, 0, sizeof(run->window));
  run->window.drawable_width = 1280;
  run->window.drawable_height = 720;
  run->renderer = renderer_create(1280, 720);
  stage_options_t options = {SCENE_CITY, package_directory, NULL, false, true};
  assert(driving_stage_init(&run->stage, &run->window, run->renderer, NULL,
                            options));
}

static void stop(city_run_t* run) {
  driving_stage_destroy(&run->stage);
  renderer_destroy(run->renderer);
}

static const rigid_body_t* car(city_run_t* run) {
  return physics_body(run->stage.physics, run->stage.vehicle.body);
}

static void drive(city_run_t* run, driving_input_t input, float seconds) {
  int steps = (int)(seconds * 120);
  for (int i = 0; i < steps; i++) {
    driving_stage_step(&run->stage, input, 1.0f / 120.0f);
    const rigid_body_t* body = car(run);
    assert(isfinite(body->position.x) && isfinite(body->position.y) &&
           isfinite(body->position.z));
    assert(body->position.y > -0.5f); /* never through the ground */
    assert(body->position.y < 5.0f);  /* never launched */
  }
}

static void the_city_loads_with_roads_buildings_and_a_spawn_on_a_road(void) {
  city_run_t run;
  start(&run);
  assert(world_count(&run.stage.world) > 20);
  assert(physics_collider_count(run.stage.physics) > 40);
  ray_t down = {vec3_add(run.stage.scene.spawn_position, vec3(0, 5, 0)),
                vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(
      physics_ray_cast(run.stage.physics, down, 20, COLLISION_LAYER_ALL, &hit));
  assert(near_to(hit.hit.point.y, 0.06f,
                 0.01f)); /* the road surface, not the grass */
  assert(near(hit.friction, 1.0f));
  stop(&run);
}

static void the_car_drives_along_the_spawn_road_and_through_a_junction(void) {
  city_run_t run;
  start(&run);
  driving_input_t idle = {0, 0, 0, false};
  drive(&run, idle, 1.0f);
  driving_input_t throttle = {1, 0, 0, false};
  vec3_t start_position = car(&run)->position;
  drive(&run, throttle, 8.0f);
  const rigid_body_t* body = car(&run);
  assert(vec3_distance(body->position, start_position) > 60.0f);
  assert(quat_up(body->rotation).y > 0.95f);
  bool grounded = true;
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++)
    grounded = grounded && run.stage.vehicle.wheels[i].grounded;
  assert(grounded);
  stop(&run);
}

static void buildings_stop_the_car(void) {
  city_run_t run;
  start(&run);
  /* Building 201 spans x -133..-94 and z 44..89 in the fixture's local
   * frame. Start 25 m north of its north wall, facing south (+Z). */
  vec3_t start_position = vec3(-114.0f, 0.9f, 20.0f);
  vehicle_set_spawn(&run.stage.vehicle, start_position,
                    quat_from_yaw((float)M_PI));
  vehicle_reset(&run.stage.vehicle, run.stage.physics);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&run, throttle, 8.0f);
  const rigid_body_t* body = car(&run);
  assert(body->position.z > 30.0f); /* it did reach the wall */
  assert(body->position.z < 44.5f); /* and did not pass through it */
  assert(fabsf(vehicle_forward_speed(&run.stage.vehicle)) < 2.0f);
  stop(&run);
}

void run_citydrive_tests(void) {
  cook_fixture_city();
  RUN(the_city_loads_with_roads_buildings_and_a_spawn_on_a_road);
  RUN(the_car_drives_along_the_spawn_road_and_through_a_junction);
  RUN(buildings_stop_the_car);
}
