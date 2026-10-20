/* Drives the car through the city generated from the fixture, using the
 * stub renderer: the road surfaces, junction patches and building walls
 * must hold the car up and stop it, with no gaps and no NaNs. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
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
static const char* hill_directory = "/tmp/gta-like-test-citydrive-hill";

/* Ground at 100 m on the west side rising 0.08 m per metre to the east. */
static float hill_height(float x) { return 100.0f + 0.08f * x; }

static void cook_fixture_city_into(const char* directory, bool hilly) {
  osm_data_t osm;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  city_model_t model;
  assert(city_import(&model, &osm, city_import_default_options(),
                     "tests/fixtures/district.osm"));
  osm_free(&osm);
  if (hilly) {
    float heights[4] = {hill_height(-1000), hill_height(1000),
                        hill_height(-1000), hill_height(1000)};
    model.terrain.min_x = -1000, model.terrain.min_z = -1000;
    model.terrain.dx = 2000, model.terrain.dz = 2000;
    model.terrain.cols = 2, model.terrain.rows = 2;
    model.terrain.heights = malloc(sizeof(heights));
    memcpy(model.terrain.heights, heights, sizeof(heights));
  }
  city_generator_options_t options = city_generator_default_options();
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "district", &stats));
  assert(city_package_write(&package, directory));
  city_package_free(&package);
  city_model_free(&model);
}

static void cook_fixture_city(void) {
  cook_fixture_city_into(package_directory, false);
  cook_fixture_city_into(hill_directory, true);
}

typedef struct {
  window_t window;
  renderer_t* renderer;
  driving_stage_t stage;
} city_run_t;

/* Bounds on the car's height: the flat city sits at 0, the hill 100 m up. */
static float drive_floor = -0.5f, drive_ceiling = 5.0f;

static int test_traffic_cars = 0, test_pedestrians = 0;

static void start_in(city_run_t* run, const char* directory) {
  bool hilly = directory == hill_directory;
  drive_floor = hilly ? 80.0f : -0.5f;
  drive_ceiling = hilly ? 125.0f : 5.0f;
  memset(&run->window, 0, sizeof(run->window));
  run->window.drawable_width = 1280;
  run->window.drawable_height = 720;
  run->renderer = renderer_create(1280, 720);
  stage_options_t options = {
      SCENE_CITY,        directory,        NULL, false, true, 10.0f, 0.0f,
      test_traffic_cars, test_pedestrians, 0.0f};
  assert(driving_stage_init(&run->stage, &run->window, run->renderer, NULL,
                            options));
}

static void start(city_run_t* run) { start_in(run, package_directory); }

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
    assert(body->position.y > drive_floor);   /* never through the ground */
    assert(body->position.y < drive_ceiling); /* never launched */
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

static void the_car_follows_the_hills(void) {
  city_run_t run;
  start_in(&run, hill_directory);
  /* The car spawns on the slope, not under it. */
  vec3_t spawn = run.stage.scene.spawn_position;
  assert(near_to(spawn.y, hill_height(spawn.x) + 0.06f + 0.9f, 0.2f));
  driving_input_t idle = {0, 0, 0, false};
  drive(&run, idle, 1.0f);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&run, throttle, 8.0f);
  const rigid_body_t* body = car(&run);
  /* Hills lift the drive: unlike on the flat the check in drive() that
   * the car stays below 5 m cannot hold, so look at how high it is above
   * the ground under it instead. */
  float ground = hill_height(body->position.x);
  assert(body->position.y > ground - 0.1f && body->position.y < ground + 1.5f);
  bool grounded = true;
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++)
    grounded = grounded && run.stage.vehicle.wheels[i].grounded;
  assert(grounded);
  assert(vec3_distance(body->position, spawn) > 40.0f);
  stop(&run);
}

static void the_town_sounds_follow_the_time_of_day(void) {
  city_run_t run;
  start(&run);
  run.stage.hour = 12.0f;
  audio_state_t noon = driving_stage_audio_state(&run.stage);
  run.stage.hour = 0.5f;
  audio_state_t midnight = driving_stage_audio_state(&run.stage);
  assert(noon.city > 0.9f && midnight.city > 0.9f);
  assert(noon.night < 0.05f && midnight.night > 0.9f);
  stop(&run);
}

static void the_driver_sees_from_inside_and_from_the_bonnet(void) {
  city_run_t run;
  start(&run);
  driving_input_t idle = {0, 0, 0, false};
  drive(&run, idle, 0.5f);
  vec3_t chassis = car(&run)->position;
  vec3_t forward = quat_rotate(car(&run)->rotation, vec3(0, 0, -1));
  run.stage.view_mode = 2;
  render_camera_t cockpit = driving_stage_camera(&run.stage);
  assert(vec3_distance(cockpit.eye, chassis) < 0.8f); /* inside the car */
  vec3_t gaze = vec3_normalize(vec3_sub(cockpit.target, cockpit.eye));
  assert(vec3_dot(gaze, forward) > 0.99f);
  run.stage.view_mode = 1;
  render_camera_t bonnet = driving_stage_camera(&run.stage);
  assert(vec3_dot(vec3_sub(bonnet.eye, chassis), forward) >
         0.5f); /* ahead of the driver */
  run.stage.view_mode = 0;
  render_camera_t chase = driving_stage_camera(&run.stage);
  assert(vec3_dot(vec3_sub(chase.eye, chassis), forward) < -2.0f); /* behind */
  stop(&run);
}

static void v_swaps_the_car_for_the_next_one_where_it_stands(void) {
  city_run_t run;
  start(&run);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&run, throttle, 2.0f);
  vec3_t where = car(&run)->position;
  float mass = car(&run)->mass;
  char first[64];
  snprintf(first, sizeof(first), "%s", run.stage.vehicle.params.name);
  driving_stage_next_vehicle(&run.stage);
  assert(strcmp(run.stage.vehicle.params.name, first) != 0);
  assert(vec3_distance(car(&run)->position, where) < 1.0f);
  assert(car(&run)->mass != mass);
  /* The new car drives. */
  vec3_t before = car(&run)->position;
  drive(&run, throttle, 3.0f);
  assert(vec3_distance(car(&run)->position, before) > 4.0f);
  /* All four in turn, and back to the first. */
  for (int i = 0; i < 3; i++) driving_stage_next_vehicle(&run.stage);
  assert(strcmp(run.stage.vehicle.params.name, first) == 0);
  stop(&run);
}

static void a_busy_city_runs_for_a_minute_without_trouble(void) {
  test_traffic_cars = 20;
  test_pedestrians = 40;
  city_run_t run;
  start(&run);
  assert(run.stage.traffic && run.stage.people);
  assert(run.stage.parked.count > 0 || run.stage.scene.parked_count == 0);
  assert(traffic_car_count(run.stage.traffic) == 20);
  driving_input_t throttle = {0.6f, 0, 0.1f, false};
  for (int second = 0; second < 60; second++) {
    drive(&run, throttle, 1.0f);
    /* Whatever the car runs into, nothing goes wrong. */
    for (size_t i = 0; i < traffic_car_count(run.stage.traffic); i++) {
      traffic_car_t c = traffic_car(run.stage.traffic, i);
      assert(isfinite(c.position.x) && isfinite(c.position.z) &&
             c.speed < 14.0f);
    }
    if (second % 15 == 14) {
      driving_input_t reverse = {0, 1, -0.3f, false};
      drive(&run, reverse, 2.0f);
    }
  }
  test_traffic_cars = 0;
  test_pedestrians = 0;
  stop(&run);
}

static void the_chase_view_widens_with_speed(void) {
  city_run_t run;
  start(&run);
  driving_input_t idle = {0, 0, 0, false};
  drive(&run, idle, 0.5f);
  float standing = driving_stage_camera(&run.stage).fov_y;
  driving_input_t throttle = {1, 0, 0, false};
  drive(&run, throttle, 8.0f);
  float moving = driving_stage_camera(&run.stage).fov_y;
  assert(moving > standing + 0.03f && moving < standing + 0.2f);
  stop(&run);
}

static void rain_comes_on_gradually_wets_the_roads_and_is_heard(void) {
  city_run_t run;
  start(&run);
  assert(run.stage.rain == 0.0f);
  driving_stage_render(&run.stage);
  float dry = 0;
  for (size_t i = 0; i < run.stage.world.capacity; i++) {
    entity_t* e = &run.stage.world.entities[i];
    if (e->used && strncmp(e->label, "road ", 5) == 0)
      dry = e->material.roughness;
  }
  assert(dry > 0.9f);
  driving_stage_set_rain(&run.stage, 1.0f);
  driving_input_t idle = {0, 0, 0, false};
  driving_stage_advance(&run.stage, idle, 3.0);
  assert(run.stage.rain > 0.3f && run.stage.rain < 0.7f); /* part way */
  driving_stage_advance(&run.stage, idle, 4.0);
  assert(run.stage.rain > 0.99f);
  assert(driving_stage_audio_state(&run.stage).rain > 0.99f);
  driving_stage_render(&run.stage);
  float wet = 1;
  for (size_t i = 0; i < run.stage.world.capacity; i++) {
    entity_t* e = &run.stage.world.entities[i];
    if (e->used && strncmp(e->label, "road ", 5) == 0)
      wet = e->material.roughness;
  }
  assert(wet < 0.3f);
  stop(&run);
}

static void the_map_and_the_pause_silence_the_engine(void) {
  city_run_t run;
  start(&run);
  driving_input_t throttle = {1, 0, 0, false};
  drive(&run, throttle, 1.0f);
  assert(driving_stage_audio_state(&run.stage).engine_running);
  run.stage.map_open = true;
  audio_state_t on_the_map = driving_stage_audio_state(&run.stage);
  assert(!on_the_map.engine_running && on_the_map.skid == 0);
  run.stage.map_open = false;
  run.stage.paused = true;
  assert(!driving_stage_audio_state(&run.stage).engine_running);
  stop(&run);
}

void run_citydrive_tests(void) {
  cook_fixture_city();
  RUN(the_city_loads_with_roads_buildings_and_a_spawn_on_a_road);
  RUN(the_car_drives_along_the_spawn_road_and_through_a_junction);
  RUN(buildings_stop_the_car);
  RUN(the_car_follows_the_hills);
  RUN(the_town_sounds_follow_the_time_of_day);
  RUN(the_driver_sees_from_inside_and_from_the_bonnet);
  RUN(v_swaps_the_car_for_the_next_one_where_it_stands);
  RUN(a_busy_city_runs_for_a_minute_without_trouble);
  RUN(the_chase_view_widens_with_speed);
  RUN(rain_comes_on_gradually_wets_the_roads_and_is_heard);
  RUN(the_map_and_the_pause_silence_the_engine);
}
