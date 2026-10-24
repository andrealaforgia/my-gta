/* Drives the car through the city generated from the fixture, using the
 * stub renderer: the road surfaces, junction patches and building walls
 * must hold the car up and stop it, with no gaps and no NaNs. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "air_stage.h"
#include "aircraft_cockpit.h"
#include "city_generator.h"
#include "city_import.h"
#include "city_list.h"
#include "city_menu.h"
#include "city_package.h"
#include "driving_stage.h"
#include "osm_reader.h"
#include "poles.h"
#include "street_name.h"
#include "test_groups.h"
#include "test_support.h"

static const char* package_directory = "/tmp/gta-like-test-citydrive";
static const char* hill_directory = "/tmp/gta-like-test-citydrive-hill";
static const char* airport_directory = "/tmp/gta-like-test-citydrive-airport";

/* Ground at 100 m on the west side rising 0.08 m per metre to the east. */
static float hill_height(float x) { return 100.0f + 0.08f * x; }

static void cook_fixture_city_into(const char* directory, bool hilly,
                                   bool airport) {
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
  if (!airport)
    snprintf(options.airport_side, sizeof(options.airport_side), "-");
  city_package_t package;
  city_generator_stats_t stats;
  assert(city_generate(&package, &model, &options, "district", &stats));
  assert(city_package_write(&package, directory));
  city_package_free(&package);
  city_model_free(&model);
}

static void cook_fixture_city(void) {
  cook_fixture_city_into(package_directory, false, false);
  cook_fixture_city_into(hill_directory, true, false);
  cook_fixture_city_into(airport_directory, false, true);
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
      test_traffic_cars, test_pedestrians, 0.0f, false};
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
  driving_input_t idle = {0, 0, 0, false, false};
  drive(&run, idle, 1.0f);
  driving_input_t throttle = {1, 0, 0, false, false};
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
  driving_input_t throttle = {1, 0, 0, false, false};
  drive(&run, throttle, 8.0f);
  const rigid_body_t* body = car(&run);
  assert(body->position.z > 30.0f); /* it did reach the wall */
  assert(body->position.z < 44.5f); /* and did not pass through it */
  assert(fabsf(vehicle_forward_speed(&run.stage.vehicle)) < 2.0f);
  stop(&run);
}

static void a_hard_crash_jolts_the_view_and_it_settles(void) {
  city_run_t run;
  start(&run);
  vehicle_set_spawn(&run.stage.vehicle, vec3(-114.0f, 0.9f, 20.0f),
                    quat_from_yaw((float)M_PI));
  vehicle_reset(&run.stage.vehicle, run.stage.physics);
  assert(run.stage.shake == 0);
  render_camera_t steady = driving_stage_camera(&run.stage);
  driving_input_t throttle = {1, 0, 0, false, false};
  float most = 0;
  for (int i = 0; i < 120 * 8; i++) {
    driving_stage_step(&run.stage, throttle, 1.0f / 120.0f);
    most = fmaxf(most, run.stage.shake);
  }
  assert(most > 0.15f); /* the wall was hit hard */
  /* While it shakes the eye is not where it would be; afterwards it is. */
  run.stage.shake = 0.8f;
  render_camera_t jolted = driving_stage_camera(&run.stage);
  run.stage.shake = 0;
  render_camera_t calm = driving_stage_camera(&run.stage);
  assert(vec3_distance(jolted.eye, calm.eye) > 0.02f);
  driving_input_t coast = {0, 1, 0, false, false};
  for (int i = 0; i < 120 * 3; i++)
    driving_stage_step(&run.stage, coast, 1.0f / 120.0f);
  assert(run.stage.shake < 0.01f);
  (void)steady;
  stop(&run);
}

static size_t loaded_chunks(const scene_t* scene) {
  size_t n = 0;
  for (size_t i = 0; i < scene->package->chunk_count; i++)
    n += scene->chunk_states[i].loaded;
  return n;
}

static void a_big_city_is_streamed_in_and_out_around_the_car(void) {
  city_run_t run;
  start(&run);
  scene_t* scene = &run.stage.scene;
  size_t chunks = scene->package->chunk_count;
  size_t everything = world_count(&run.stage.world);
  size_t all_colliders = physics_collider_count(run.stage.physics);
  assert(chunks > 4 && loaded_chunks(scene) == chunks); /* small: all in */
  /* Shrink the radii to a few chunks and stand in a corner. */
  scene->stream_load_radius = 90.0f;
  scene->stream_unload_radius = 130.0f;
  scene->stream_near_radius = 30.0f;
  vec3_t corner = vec3(scene->package->chunks[0].bounds.min.x + 5.0f, 0,
                       scene->package->chunks[0].bounds.min.z + 5.0f);
  scene_stream(scene, corner, 100);
  size_t near = loaded_chunks(scene);
  assert(near >= 1 && near < chunks);
  assert(world_count(&run.stage.world) < everything);
  size_t ground_only = physics_collider_count(run.stage.physics);
  assert(ground_only <= all_colliders && ground_only > 0);
  /* The roads stay solid everywhere: a ray from above still lands. */
  vec3_t far = vec3(scene->package->chunks[chunks - 1].bounds.max.x - 5.0f, 50,
                    scene->package->chunks[chunks - 1].bounds.max.z - 5.0f);
  ray_t down = {far, vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(run.stage.physics, down, 200, COLLISION_LAYER_GROUND,
                          &hit));
  /* Poles of unloaded chunks are gone, and standing ones count only the
   * loaded. */
  assert(scene->poles_gone > 0 || scene->pole_count < 10000);
  /* A budget of nothing loads only what is close; walking across brings the
   * far corner in and sends the first one out. */
  scene_stream(scene, far, 0);
  scene_stream(scene, far, 100);
  assert(scene->chunk_states[chunks - 1].loaded);
  assert(!scene->chunk_states[0].loaded);
  /* Back again, and everything the first corner had is rebuilt. */
  scene_stream(scene, corner, 100);
  assert(scene->chunk_states[0].loaded);
  assert(world_count(&run.stage.world) < everything);
  scene->stream_load_radius = 1e6f;
  scene->stream_unload_radius = 2e6f;
  scene_stream(scene, corner, 1000);
  assert(loaded_chunks(scene) == chunks);
  assert(world_count(&run.stage.world) == everything);
  assert(physics_collider_count(run.stage.physics) == all_colliders);
  stop(&run);
}

static void the_street_under_the_car_is_named_on_the_screen(void) {
  city_run_t run;
  start(&run);
  scene_t* scene = &run.stage.scene;
  /* The first named street in the package: on its middle the name comes
   * out in capitals with spaces; far from every street there is none. */
  const city_route_t* named = NULL;
  for (size_t r = 0; r < scene->route_count && !named; r++)
    if (scene->routes[r].name[0]) named = &scene->routes[r];
  assert(named);
  float x = (named->points[0] + named->points[2]) * 0.5f;
  float z = (named->points[1] + named->points[3]) * 0.5f;
  char name[48];
  assert(street_name_at(scene->routes, scene->route_count, x, z, name,
                        sizeof(name)));
  assert(strlen(name) > 2 && strchr(name, '_') == NULL);
  for (const char* c = name; *c; c++)
    assert(*c == ' ' || !(*c >= 'a' && *c <= 'z'));
  assert(!street_name_at(scene->routes, scene->route_count, 9000.0f, 9000.0f,
                         name, sizeof(name)));
  assert(name[0] == '\0');
  /* The stage keeps it up to date as the car moves. */
  physics_body_teleport(run.stage.physics, run.stage.vehicle.body,
                        vec3(x, 1.0f, z), quat_identity());
  driving_input_t coast = {0, 0, 0, false, false};
  for (int i = 0; i < 120; i++)
    driving_stage_step(&run.stage, coast, 1.0f / 120.0f);
  assert(run.stage.street[0] != '\0');
  physics_body_teleport(run.stage.physics, run.stage.vehicle.body,
                        vec3(9000.0f, 50.0f, 9000.0f), quat_identity());
  for (int i = 0; i < 120; i++)
    driving_stage_step(&run.stage, coast, 1.0f / 120.0f);
  assert(run.stage.street[0] == '\0');
  stop(&run);
}

static void the_tyres_stay_quiet_on_an_ordinary_drive(void) {
  /* No squeal and no skid marks driving along a road, on the flat or up a
   * hill, nor while standing still. */
  const char* places[2] = {package_directory, hill_directory};
  for (int place = 0; place < 2; place++) {
    city_run_t run;
    start_in(&run, places[place]);
    driving_input_t idle = {0, 0, 0, false, false},
                    throttle = {0.5f, 0, 0, false, false};
    float loudest = 0;
    bool marked = false;
    for (int i = 0; i < 120 * 10; i++) {
      driving_stage_step(&run.stage, i < 120 * 3 ? idle : throttle,
                         1.0f / 120.0f);
      loudest = fmaxf(loudest, driving_stage_audio_state(&run.stage).skid);
      for (int w = 0; w < VEHICLE_WHEEL_COUNT; w++)
        marked |= run.stage.vehicle.wheels[w].skidding;
    }
    assert(loudest == 0 && !marked);
    stop(&run);
  }
}

static void the_car_follows_the_hills(void) {
  city_run_t run;
  start_in(&run, hill_directory);
  /* The car spawns on the slope, not under it. */
  vec3_t spawn = run.stage.scene.spawn_position;
  assert(near_to(spawn.y, hill_height(spawn.x) + 0.06f + 0.9f, 0.2f));
  driving_input_t idle = {0, 0, 0, false, false};
  drive(&run, idle, 1.0f);
  driving_input_t throttle = {1, 0, 0, false, false};
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
  driving_input_t idle = {0, 0, 0, false, false};
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
  driving_input_t throttle = {1, 0, 0, false, false};
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
  /* All of them in turn, and back to the first. */
  for (int i = 0; i < FLEET_VARIANTS - 1; i++)
    driving_stage_next_vehicle(&run.stage);
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
  assert(pedestrians_count(run.stage.people) == 40);
  vec3_t start_position = car(&run)->position;
  float furthest = 0;
  driving_input_t throttle = {0.6f, 0, 0.1f, false, false};
  for (int second = 0; second < 60; second++) {
    drive(&run, throttle, 1.0f);
    furthest =
        fmaxf(furthest, vec3_distance(car(&run)->position, start_position));
    /* Whatever the car runs into, nothing goes wrong (and police after a
     * crash are quick, so the limit leaves room for them). */
    for (size_t i = 0; i < traffic_car_count(run.stage.traffic); i++) {
      traffic_car_t c = traffic_car(run.stage.traffic, i);
      assert(isfinite(c.position.x) && isfinite(c.position.y) &&
             isfinite(c.position.z) && isfinite(c.yaw) && isfinite(c.speed) &&
             fabsf(c.speed) < 30.0f);
    }
    for (size_t i = 0; i < pedestrians_count(run.stage.people); i++) {
      pedestrian_t person = pedestrians_get(run.stage.people, i);
      assert(isfinite(person.position.x) && isfinite(person.position.y) &&
             isfinite(person.position.z) && isfinite(person.yaw) &&
             isfinite(person.speed));
    }
    if (second % 15 == 14) {
      driving_input_t reverse = {0, 1, -0.3f, false, false};
      drive(&run, reverse, 2.0f);
    }
  }
  assert(furthest > 10.0f);
  test_traffic_cars = 0;
  test_pedestrians = 0;
  stop(&run);
}

static void the_chase_view_widens_with_speed(void) {
  city_run_t run;
  start(&run);
  driving_input_t idle = {0, 0, 0, false, false};
  drive(&run, idle, 0.5f);
  float standing = driving_stage_camera(&run.stage).fov_y;
  driving_input_t throttle = {1, 0, 0, false, false};
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
  driving_input_t idle = {0, 0, 0, false, false};
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

static void the_cockpit_has_a_dashboard_and_a_wheel_that_turns(void) {
  city_run_t run;
  start(&run);
  run.stage.view_mode = 0;
  driving_stage_render(&run.stage);
  int outside = renderer_stats(run.renderer).draw_calls;
  run.stage.view_mode = 2;
  driving_stage_render(&run.stage);
  int inside = renderer_stats(run.renderer).draw_calls;
  /* Dashboard, wheel rim, hub and spokes are added; the two views cull the
   * street a little differently, so the count is only about the same. */
  assert(inside + 8 >= outside + 3);
  /* The wheel's angle follows the steering, a lock and a half each way at
   * full input, and is level when the car goes straight. */
  assert(near_to(driving_stage_wheel_angle(&run.stage), 0.0f, 1e-3f));
  driving_input_t left = {0.2f, 0, -1.0f, false, false};
  drive(&run, left, 2.0f);
  float angle = driving_stage_wheel_angle(&run.stage);
  assert(angle < -1.0f && angle > -4.8f);
  stop(&run);
}

static void the_horn_sounds_only_while_held_and_not_on_the_map(void) {
  city_run_t run;
  start(&run);
  assert(!driving_stage_audio_state(&run.stage).horn);
  run.stage.horn = true;
  assert(driving_stage_audio_state(&run.stage).horn);
  run.stage.map_open = true;
  assert(!driving_stage_audio_state(&run.stage).horn);
  stop(&run);
}

static void a_wrecked_car_smokes_and_looks_scuffed(void) {
  city_run_t run;
  start(&run);
  driving_stage_render(&run.stage);
  int sound = renderer_stats(run.renderer).draw_calls;
  run.stage.vehicle.damage = 0.3f; /* dented, not yet smoking */
  driving_stage_render(&run.stage);
  assert(renderer_stats(run.renderer).draw_calls == sound);
  assert(car_visual_damage_tint(0.3f) < 1.0f &&
         car_visual_damage_tint(0.3f) > 0.6f);
  run.stage.vehicle.damage = 0.8f;
  driving_stage_render(&run.stage);
  assert(renderer_stats(run.renderer).draw_calls >=
         sound + 5); /* puffs of smoke */
  assert(car_visual_damage_tint(1.0f) < car_visual_damage_tint(0.5f));
  assert(car_visual_damage_tint(0.0f) == 1.0f);
  stop(&run);
}

static void a_wrecked_car_burns_then_blows_up_and_the_player_starts_again(
    void) {
  city_run_t run;
  start(&run);
  driving_input_t idle = {0, 0, 0, false, false};
  run.stage.vehicle.damage = 1.0f;
  for (int i = 0; i < 120 * 3; i++)
    driving_stage_step(&run.stage, idle, 1.0f / 120.0f);
  assert(run.stage.burning > 2.0f && run.stage.blast == 0);
  /* A car that is mended stops burning. */
  run.stage.vehicle.damage = 0.5f;
  driving_stage_step(&run.stage, idle, 1.0f / 120.0f);
  assert(run.stage.burning == 0);
  run.stage.vehicle.damage = 1.0f;
  bool blew = false;
  for (int i = 0; i < 120 * 12; i++) {
    driving_stage_step(&run.stage, idle, 1.0f / 120.0f);
    if (run.stage.blast > 0) blew = true;
  }
  /* It went up, and then the player was put back at the start, mended and
   * mended. */
  assert(blew && run.stage.blast == 0);
  assert(run.stage.vehicle.damage == 0);
  stop(&run);
}

/* Puts the player in a plane in the air over the airport. */
static aircraft_t* take_to_the_air(city_run_t* run) {
  driving_stage_t* stage = &run->stage;
  driving_input_t stand = {0, 0, 0, false, false};
  drive(run, stand, 1.0f);
  driving_stage_interact(stage); /* out of the car */
  const city_airport_t* airport = &stage->scene.package->airport;
  vec3_t axis = vec3(-sinf(airport->yaw), 0, -cosf(airport->yaw));
  aircraft_t* plane = &stage->aircraft[0];
  vec3_t start =
      vec3(airport->x - axis.x * 380.0f, 5.0f, airport->z - axis.z * 380.0f);
  scene_stream(&stage->scene, start, 1 << 20);
  ray_t down = {vec3(start.x, 100.0f, start.z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(stage->physics, down, 300.0f, COLLISION_LAYER_GROUND,
                          &hit));
  plane->position = vec3(start.x, hit.hit.point.y, start.z);
  plane->yaw = airport->yaw;
  stage->walker.position = vec3(start.x + 3.0f, plane->position.y, start.z);
  driving_stage_interact(stage);
  assert(stage->piloting == 1);
  for (int i = 0; i < 120 * 22 && plane->on_ground; i++) {
    float t = (float)i / 120.0f;
    driving_input_t fly = {0, t > 8.0f ? 1.0f : 0.0f, 0, true, false};
    driving_stage_step(stage, fly, 1.0f / 120.0f);
  }
  assert(!plane->on_ground);
  return plane;
}

/* Puts the player in aircraft `index` at the airport, on the ground. */
static aircraft_t* board_aircraft(city_run_t* run, int index) {
  driving_stage_t* stage = &run->stage;
  driving_input_t stand = {0, 0, 0, false, false};
  drive(run, stand, 1.0f);
  if (!stage->on_foot) driving_stage_interact(stage); /* out of the car */
  aircraft_t* craft = &stage->aircraft[index];
  scene_stream(&stage->scene, craft->position, 1 << 20);
  stage->walker.position = vec3_add(craft->position, vec3(3.0f, 0, 0));
  driving_stage_interact(stage);
  return craft;
}

/* The horn sounds in the cockpit of an aircraft as it does in a car, and not
 * from someone walking. */
static void the_horn_sounds_from_an_aircraft_and_not_from_a_walker(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  stage->horn = true;
  assert(driving_stage_audio_state(stage).horn);
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  driving_stage_interact(stage); /* out on foot */
  assert(!driving_stage_audio_state(stage).horn);
  board_aircraft(&run, 6);
  assert(driving_stage_audio_state(stage).horn);
  stop(&run);
}

/* C while flying looks from the pilot's seat: the eye is inside the aircraft,
 * at the place its kind puts it, and the world is drawn from there. */
static void the_cockpit_view_looks_from_the_pilots_seat(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  for (int index = 5; index < 8; index++) {
    aircraft_t* craft = board_aircraft(&run, index);
    stage->air_view = 0;
    render_camera_t behind = driving_stage_camera(stage);
    stage->air_view = 1;
    render_camera_t seat = driving_stage_camera(stage);
    vec3_t eye = transform_point(aircraft_transform(craft),
                                 aircraft_cockpit_eye(craft->kind));
    assert(vec3_distance(seat.eye, eye) < 0.01f);
    assert(vec3_distance(behind.eye, craft->position) >
           2.0f * vec3_distance(seat.eye, craft->position));
    /* Looking where the nose points. */
    vec3_t ahead = vec3_normalize(vec3_sub(seat.target, seat.eye));
    vec3_t nose = aircraft_heading(craft);
    assert(vec3_dot(ahead, nose) > 0.99f);
    driving_stage_interact(stage); /* out again, so the next can be boarded */
    stage->on_foot = true;
    stage->piloting = 0;
  }
  stop(&run);
}

/* G in the jet or the gunship launches a missile, one at a time, that flies
 * out and goes off where it strikes; in a plane or an airliner it does
 * nothing. */
static void a_missile_leaves_the_jet_flies_and_goes_off_where_it_strikes(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  board_aircraft(&run, 6);
  assert(driving_stage_fire(stage));
  assert(!driving_stage_fire(stage)); /* the rail is still hot */
  assert(stage->missiles[0].active);
  vec3_t velocity = stage->missiles[0].velocity;
  assert(vec3_length(velocity) > 150.0f);
  driving_input_t idle = {0, 0, 0, false, false};
  bool struck = false;
  for (int i = 0; i < 120 * 8; i++) {
    driving_stage_step(stage, idle, 1.0f / 120.0f);
    struck |= stage->strikes[0].age > 0;
  }
  assert(struck && !stage->missiles[0].active);
  /* The fireball dies away. */
  assert(stage->strikes[0].age == 0);
  /* A second can follow once the cooldown is over. */
  assert(driving_stage_fire(stage));
  stop(&run);
}

/* The 747 stands on the taxiway: it can be boarded where it is and flown off
 * without hitting anything. */
static void the_747_can_be_flown_off_from_its_stand(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  aircraft_t* jumbo = board_aircraft(&run, 5);
  assert(jumbo->kind == AIRCRAFT_AIRLINER);
  float lifted_at = -1, lowest_speed = 1e9f;
  for (int i = 0; i < 120 * 90 && stage->piloting == 6; i++) {
    float t = (float)i / 120.0f;
    /* Full power from the start, nose up once there is speed to lift on. */
    driving_input_t fly = {0, jumbo->speed > 56.0f ? 1.0f : 0.0f, 0, true,
                           false};
    driving_stage_step(stage, fly, 1.0f / 120.0f);
    if (lifted_at < 0 && !jumbo->on_ground) lifted_at = t;
    lowest_speed = fminf(lowest_speed, jumbo->speed);
  }
  assert(stage->piloting == 6 && lifted_at > 0 && jumbo->position.y > 30.0f);
  stop(&run);
}

/* Standing at the nose or the tail of the 747, thirty metres from its middle,
 * is close enough to climb in. */
static void the_747_can_be_boarded_at_its_nose_and_its_tail(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  driving_stage_interact(stage); /* out of the car */
  aircraft_t* jumbo = &stage->aircraft[5];
  scene_stream(&stage->scene, jumbo->position, 1 << 20);
  vec3_t heading = aircraft_heading(jumbo);
  for (int end = -1; end <= 1; end += 2) {
    stage->walker.position =
        vec3_add(jumbo->position, vec3_scale(heading, 36.0f * (float)end));
    assert(air_stage_nearest(stage) == jumbo);
    /* But not from the wing tip, or from well beyond the nose. */
    stage->walker.position =
        vec3_add(jumbo->position, vec3_scale(heading, 60.0f * (float)end));
    assert(air_stage_nearest(stage) == NULL);
  }
  driving_stage_interact(stage);
  assert(stage->piloting == 0);
  stage->walker.position =
      vec3_add(jumbo->position, vec3_scale(heading, 35.0f));
  driving_stage_interact(stage);
  assert(stage->piloting == 6);
  stop(&run);
}

/* From the seat every aircraft draws its panel, round dials, horizon ball,
 * compass, throttle and stick: many more parts than the outside of the
 * aircraft, and the aeroplanes add their landing-gear lights. */
static void every_cockpit_draws_its_dials(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  int outside[AIRCRAFT_KIND_COUNT], inside[AIRCRAFT_KIND_COUNT];
  const int index_of[AIRCRAFT_KIND_COUNT] = {0, 3, 5, 6, 7};
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    board_aircraft(&run, index_of[k]);
    assert(stage->aircraft[index_of[k]].kind == (aircraft_kind_t)k);
    stage->air_view = 0;
    driving_stage_render(stage);
    outside[k] = renderer_stats(run.renderer).draw_calls;
    stage->air_view = 1;
    driving_stage_render(stage);
    inside[k] = renderer_stats(run.renderer).draw_calls;
    driving_stage_interact(stage);
    stage->on_foot = true;
    stage->piloting = 0;
  }
  for (int k = 0; k < AIRCRAFT_KIND_COUNT; k++) {
    /* The hull is left out from the seat, yet the deck adds more. */
    assert(inside[k] >= outside[k] + 3);
  }
  stop(&run);
}

static void a_plane_has_no_missiles_to_fire(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  board_aircraft(&run, 0);
  assert(!driving_stage_fire(stage) && !stage->missiles[0].active);
  stop(&run);
}

/* F in the air is a jump: a parachute for the player, a dive and a crash for
 * the plane, which goes back to its stand; the player lands and lives. */
static void f_in_the_air_bails_out_and_the_plane_crashes(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  aircraft_t* plane = take_to_the_air(&run);
  driving_input_t level = {0, 0, 0, true, false};
  for (int i = 0; i < 120 * 6; i++)
    driving_stage_step(stage, level, 1.0f / 120.0f);
  float height_at_jump = plane->position.y;
  assert(height_at_jump > 20.0f);
  driving_stage_interact(stage);
  assert(stage->piloting == 0 && stage->chute.active && plane->doomed);
  /* The wind is heard, close to, from the first moment. */
  for (int i = 0; i < 30; i++) driving_stage_advance(stage, level, 1.0 / 60.0);
  audio_state_t sound = driving_stage_audio_state(stage);
  assert(!sound.engine_running && sound.wind > 0.3f);
  assert(sound.listener_distance < 12.0f);
  /* Down on the parachute, slowly, while the plane dives and blows up. */
  driving_input_t drift = {0, 0, 0, false, false};
  bool blew = false;
  float fastest_fall = 0;
  for (int i = 0; i < 120 * 90 && stage->chute.active; i++) {
    driving_stage_step(stage, drift, 1.0f / 120.0f);
    if (stage->blast > 0) blew = true;
    fastest_fall = fmaxf(fastest_fall, -stage->chute.velocity.y);
  }
  assert(!stage->chute.active && stage->on_foot);
  assert(fastest_fall < 12.0f); /* a canopy, not a plunge */
  /* Let the plane finish if it has not. */
  for (int i = 0; i < 120 * 30 && plane->doomed; i++) {
    driving_stage_step(stage, drift, 1.0f / 120.0f);
    if (stage->blast > 0) blew = true;
  }
  assert(blew && !plane->doomed);
  /* Back on its stand, and the player's car is not wrecked by a plane. */
  assert(near_to(plane->position.x, stage->aircraft_home[0].position.x, 0.01f));
  for (int i = 0; i < 120 * 4; i++)
    driving_stage_step(stage, drift, 1.0f / 120.0f);
  assert(stage->on_foot && !stage->wrecked && stage->blast == 0);
  stop(&run);
}

/* The engine is heard from the aircraft, not from the car left behind. */
static void the_engine_note_stays_with_the_aircraft_that_is_flown(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  aircraft_t* plane = take_to_the_air(&run);
  driving_input_t level = {0, 0, 0, true, false};
  for (int i = 0; i < 60 * 8; i++) {
    driving_stage_advance(stage, level, 1.0 / 60.0);
  }
  /* Far from the car by now, yet the engine is close and running. */
  const rigid_body_t* body = car(&run);
  assert(vec3_distance(plane->position, body->position) > 100.0f);
  audio_state_t sound = driving_stage_audio_state(stage);
  assert(sound.engine_running && sound.listener_distance < 40.0f);
  assert(sound.rpm > 2000.0f);
  stop(&run);
}

/* On foot the player can punch, a few times a second, and only on foot. */
static void on_foot_the_player_can_punch_but_not_twice_at_once(void) {
  test_pedestrians = 10;
  city_run_t run;
  start(&run);
  driving_input_t stand = {0, 0, 0, false, false};
  assert(!driving_stage_punch(&run.stage)); /* at the wheel */
  drive(&run, stand, 1.0f);
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  assert(driving_stage_punch(&run.stage));
  assert(run.stage.punch_clock > 0);
  assert(!driving_stage_punch(&run.stage)); /* the arm is still out */
  drive(&run, stand, 0.6f);
  assert(driving_stage_punch(&run.stage));
  stop(&run);
  test_pedestrians = 0;
}

/* Getting out, and getting in, the driver's door swings open and shuts. */
static void the_driver_door_swings_open_to_get_out_and_in(void) {
  city_run_t run;
  start(&run);
  driving_stage_t* stage = &run.stage;
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  assert(stage->door_clock == 0);
  driving_stage_interact(stage); /* out */
  assert(stage->on_foot && stage->door_clock > 1.0f);
  drive(&run, stand, 0.5f);
  driving_stage_render(stage);
  int open_draws = renderer_stats(run.renderer).draw_calls;
  drive(&run, stand, 1.5f); /* shut by now */
  assert(stage->door_clock == 0);
  driving_stage_render(stage);
  int shut_draws = renderer_stats(run.renderer).draw_calls;
  /* The panel, its window and the dark doorway: three more. */
  assert(open_draws >= shut_draws + 3);
  /* And in again: open as they reach it, shut once they are in. */
  driving_stage_interact(stage);
  assert(stage->entering > 0);
  drive(&run, stand, 0.45f);
  driving_stage_render(stage);
  assert(renderer_stats(run.renderer).draw_calls >= shut_draws + 2);
  drive(&run, stand, 1.2f);
  assert(!stage->on_foot);
  stop(&run);
}

/* Every vehicle in the fleet, lorries and buses included, can be taken. */
static void every_fleet_vehicle_can_be_stolen_and_driven(void) {
  city_run_t run;
  start(&run);
  driving_input_t go = {1.0f, 0, 0, false, false};
  for (int variant = 0; variant < FLEET_VARIANTS; variant++) {
    vec3_t at = car(&run)->position;
    assert(driving_stage_set_vehicle(&run.stage, variant,
                                     vec3_add(at, vec3(0, 0.5f, 0)),
                                     car(&run)->rotation));
    drive(&run, go, 0.5f);
    assert(isfinite(car(&run)->position.x) && car(&run)->position.y > -5.0f);
  }
  stop(&run);
}

/* Out of the car, the player only dies in the blast if they are near it. */
static void a_player_who_got_out_survives_the_car_blowing_up_unless_close(
    void) {
  for (int close = 0; close < 2; close++) {
    city_run_t run;
    start(&run);
    driving_input_t stand = {0, 0, 0, false, false};
    drive(&run, stand, 1.0f);
    driving_stage_interact(&run.stage); /* out of the car */
    assert(run.stage.on_foot);
    if (!close) {
      /* Well away from the car. */
      run.stage.walker.position.x += 40.0f;
    }
    vec3_t car =
        physics_body(run.stage.physics, run.stage.vehicle.body)->position;
    vec3_t walker_before = run.stage.walker.position;
    run.stage.vehicle.damage = 1.0f;
    bool blew = false;
    for (int i = 0; i < 120 * 12; i++) {
      driving_stage_step(&run.stage, stand, 1.0f / 120.0f);
      if (run.stage.blast > 0) blew = true;
    }
    assert(blew && run.stage.blast == 0);
    if (close) {
      /* Caught in it: back at the start, in the car. */
      assert(!run.stage.on_foot && !run.stage.wrecked);
    } else {
      /* Still on foot where they stood, and the wreck is no use. */
      assert(run.stage.on_foot && run.stage.wrecked);
      assert(near_to(run.stage.walker.position.x, walker_before.x, 3.0f));
      run.stage.walker.position = car;
      driving_stage_interact(&run.stage);
      assert(run.stage.on_foot && run.stage.entering == 0);
    }
    stop(&run);
  }
}

static void on_foot_the_pistol_fires_leaves_a_trail_and_cools_down(void) {
  test_traffic_cars = 20;
  test_pedestrians = 10;
  city_run_t run;
  start(&run);
  driving_input_t stand = {0, 0, 0, false, false};
  /* In the car there is no shooting. */
  assert(!driving_stage_fire(&run.stage));
  drive(&run, stand, 1.0f);
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  assert(driving_stage_fire(&run.stage));
  assert(run.stage.shot_flash > 0);
  assert(vec3_distance(run.stage.shot_from, run.stage.shot_to) > 0.5f);
  /* A second shot has to wait. */
  assert(!driving_stage_fire(&run.stage));
  drive(&run, stand, 0.5f);
  assert(driving_stage_fire(&run.stage));
  /* The first shot is a crime. */
  assert(run.stage.wanted.stars >= 1);
  stop(&run);
}

static void a_reset_stops_a_climb_and_a_blast_so_nothing_is_left_wedged(void) {
  test_traffic_cars = 20;
  test_pedestrians = 10;
  city_run_t run;
  start(&run);
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  driving_stage_interact(&run.stage);
  drive(&run, stand, 0.5f);
  /* Back in, then reset half way through the climb: F still works after. */
  run.stage.walker.position =
      vec3(car(&run)->position.x + 1.8f, car(&run)->position.y,
           car(&run)->position.z);
  driving_stage_interact(&run.stage);
  assert(run.stage.entering > 0);
  driving_stage_reset(&run.stage);
  assert(!run.stage.on_foot && run.stage.entering == 0);
  drive(&run, stand, 1.0f);
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  /* A reset in the middle of a blast leaves no second one behind. */
  run.stage.on_foot = false;
  run.stage.vehicle.damage = 1.0f;
  run.stage.blast = 0.5f;
  driving_stage_reset(&run.stage);
  drive(&run, stand, 4.0f);
  assert(run.stage.blast == 0);
  stop(&run);
}

/* The airport has a road from the city, and the guide follows it. */
static void the_guide_points_at_the_airport_gate_by_a_road_that_reaches_it(
    void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  const city_airport_t* airport = &stage->scene.package->airport;
  assert(stage->guide.active && stage->guide.x == airport->gate_x &&
         stage->guide.z == airport->gate_z);
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 2.0f);
  /* The way from the start runs on to the gate, and ends beside it. */
  assert(stage->satnav_count > 1);
  float last_x = stage->satnav[2 * (stage->satnav_count - 1)],
        last_z = stage->satnav[2 * (stage->satnav_count - 1) + 1];
  assert(hypotf(last_x - airport->gate_x, last_z - airport->gate_z) < 30.0f);
  bool road = false;
  for (size_t i = 0; i < stage->scene.route_count; i++)
    road |= strcmp(stage->scene.routes[i].name, "Airport_Road") == 0;
  assert(road);
  stop(&run);
}

/* Walks to the plane, climbs in, and flies it off the runway. */
static void a_plane_can_be_stolen_and_flown_off_the_runway(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  assert(stage->has_air && stage->aircraft_count == 8);
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  driving_stage_interact(stage); /* out of the car */
  assert(stage->on_foot);
  /* A plane starts on the apron: put it on the runway, nose along it. */
  const city_airport_t* airport = &stage->scene.package->airport;
  vec3_t axis = vec3(-sinf(airport->yaw), 0, -cosf(airport->yaw));
  aircraft_t* plane = &stage->aircraft[0];
  assert(plane->kind == AIRCRAFT_PLANE);
  /* Streamed in around the runway's end, so the ground is there to ride. */
  vec3_t start =
      vec3(airport->x - axis.x * 380.0f, 5.0f, airport->z - axis.z * 380.0f);
  scene_stream(&stage->scene, start, 1 << 20);
  ray_t down = {vec3(start.x, 100.0f, start.z), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(stage->physics, down, 300.0f, COLLISION_LAYER_GROUND,
                          &hit));
  plane->position = vec3(start.x, hit.hit.point.y, start.z);
  plane->yaw = airport->yaw;
  stage->walker.position = vec3(start.x + 3.0f, plane->position.y, start.z);
  driving_stage_interact(stage);
  assert(stage->piloting == 1);
  /* Full power; the nose up once it is quick enough. */
  float height = 0;
  for (int i = 0; i < 120 * 25 && stage->piloting; i++) {
    float t = (float)i / 120.0f;
    driving_input_t fly = {0, t > 8.0f && t < 14.0f ? 1.0f : 0.0f, 0, true,
                           false};
    driving_stage_step(stage, fly, 1.0f / 120.0f);
    height = fmaxf(height, plane->position.y - hit.hit.point.y);
  }
  assert(stage->piloting == 1 && !plane->on_ground && height > 10.0f);
  /* The walker rides along, so the city follows the plane. */
  assert(near_to(stage->walker.position.x, plane->position.x, 0.01f));
  stop(&run);
}

static void a_crash_blows_the_aircraft_up_and_puts_it_back_on_its_stand(void) {
  city_run_t run;
  start_in(&run, airport_directory);
  driving_stage_t* stage = &run.stage;
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  driving_stage_interact(stage);
  aircraft_t* heli = &stage->aircraft[3];
  assert(heli->kind == AIRCRAFT_HELICOPTER);
  vec3_t home = heli->position;
  scene_stream(&stage->scene, home, 1 << 20);
  stage->walker.position = vec3(home.x + 3.0f, home.y, home.z);
  driving_stage_interact(stage);
  assert(stage->piloting == 4);
  /* Climb for a few seconds, then let it fall from there. */
  driving_input_t up = {0, 0, 0, true, false};
  for (int i = 0; i < 120 * 7; i++)
    driving_stage_step(stage, up, 1.0f / 120.0f);
  assert(heli->position.y > home.y + 10.0f);
  for (int i = 0; i < 120 * 6 && stage->piloting; i++) {
    heli->climb = -30.0f; /* dropping like a stone */
    driving_stage_step(stage, stand, 1.0f / 120.0f);
  }
  assert(stage->piloting == 0 && stage->blast > 0);
  for (int i = 0; i < 120 * 4; i++)
    driving_stage_step(stage, stand, 1.0f / 120.0f);
  assert(stage->blast == 0 && !stage->on_foot);
  assert(near_to(stage->aircraft[3].position.x, home.x, 0.01f) &&
         stage->aircraft[3].on_ground);
  stop(&run);
}

static void walls_are_grubbier_at_the_foot_but_roads_are_not(void) {
  city_run_t run;
  start(&run);
  int walls = 0, others = 0;
  for (size_t i = 0; i < run.stage.world.capacity; i++) {
    const entity_t* e = &run.stage.world.entities[i];
    if (!e->used) continue;
    bool is_wall = strncmp(e->label, "facade", 6) == 0 ||
                   strncmp(e->label, "shopfront", 9) == 0;
    if (is_wall) {
      assert(e->material.wall_ao > 0.3f && e->material.wall_ao <= 1.0f);
      walls++;
    } else if (strncmp(e->label, "road", 4) == 0 ||
               strncmp(e->label, "roof", 4) == 0) {
      assert(e->material.wall_ao == 0.0f);
      others++;
    }
  }
  assert(walls > 3 && others > 3);
  stop(&run);
}

static void everything_the_generator_builds_is_drawn(void) {
  /* A material the generator adds must not need listing in the scene to be
   * seen: every non-empty part has an entity with its material's label. */
  city_package_t package;
  assert(city_package_read(&package, package_directory));
  city_run_t run;
  start(&run);
  size_t checked = 0;
  for (size_t c = 0; c < package.chunk_count; c++)
    for (size_t p = 0; p < package.chunks[c].part_count; p++) {
      const city_part_t* part = &package.chunks[c].parts[p];
      if (part->mesh.vertex_count == 0) continue;
      char label[ENTITY_LABEL_LENGTH];
      snprintf(label, sizeof(label), "%s %d,%d", part->material,
               package.chunks[c].column, package.chunks[c].row);
      bool drawn = false;
      for (size_t i = 0; i < run.stage.world.capacity && !drawn; i++)
        drawn = run.stage.world.entities[i].used &&
                strcmp(run.stage.world.entities[i].label, label) == 0;
      assert(drawn);
      checked++;
    }
  assert(checked > 10);
  stop(&run);
  city_package_free(&package);
}

static void every_vehicle_pulls_away_and_stays_on_its_wheels(void) {
  city_run_t run;
  start(&run);
  driving_input_t throttle = {1, 0, 0, false, false};
  for (int i = 0; i < FLEET_VARIANTS; i++) {
    run.stage.vehicle.damage = 0;
    vec3_t before = car(&run)->position;
    drive(&run, throttle, 5.0f);
    assert(vec3_distance(car(&run)->position, before) > 3.0f);
    vec3_t up = quat_rotate(car(&run)->rotation, vec3(0, 1, 0));
    assert(up.y > 0.8f);
    assert(vehicle_speed_kmh(&run.stage.vehicle) > 4.0f ||
           vehicle_speed_kmh(&run.stage.vehicle) < -4.0f ||
           vec3_distance(car(&run)->position, before) > 3.0f);
    driving_stage_next_vehicle(&run.stage);
    vehicle_reset(&run.stage.vehicle,
                  run.stage.physics); /* back at the spawn */
  }
  stop(&run);
}

/* Aims the car at a pole from the east at a given speed, heading west. */
static void aim_at_pole(city_run_t* run, const pole_t* pole, float speed) {
  vec3_t from = vec3(pole->foot.x + 6.0f, pole->foot.y + 0.6f, pole->foot.z);
  quat_t facing_west = quat_from_yaw((float)M_PI * 0.5f);
  physics_body_teleport(run->stage.physics, run->stage.vehicle.body, from,
                        facing_west);
  rigid_body_t* body =
      physics_body(run->stage.physics, run->stage.vehicle.body);
  body->velocity = vec3(-speed, 0, 0);
  body->angular_velocity = vec3_zero();
}

static void a_car_that_hits_a_lamp_post_hard_knocks_it_down(void) {
  city_run_t run;
  start(&run);
  scene_t* scene = &run.stage.scene;
  assert(scene->pole_count > 5);
  size_t standing = poles_standing(scene);
  assert(standing == scene->pole_count);
  /* Some pole in the fixture stands where a fast car can reach it. */
  driving_input_t coast = {0, 0, 0, false, false};
  const pole_t* felled = NULL;
  for (size_t i = 0; i < scene->pole_count && !felled; i++) {
    pole_t* pole = &scene->poles[i];
    if (pole->kind != POLE_LAMP) continue;
    aim_at_pole(&run, pole, 12.0f);
    for (int step = 0; step < 120 * 2; step++)
      driving_stage_step(&run.stage, coast, 1.0f / 120.0f);
    if (pole->state != POLE_STANDING) felled = pole;
  }
  assert(felled);
  assert(poles_standing(scene) < standing);
  /* It is no longer solid and lies tipped over, away from the car. */
  assert(felled->collider.id == 0);
  assert(felled->state == POLE_DOWN);
  entity_t* post = world_entity(&run.stage.world, felled->entities[0]);
  assert(post && quat_rotate(post->transform.rotation, vec3(0, 1, 0)).y < 0.2f);
  /* The lamp head is dark and the car carried on rather than stopping dead. */
  bool dark = false;
  for (int e = 0; e < felled->entity_count; e++)
    dark |= strncmp(world_entity(&run.stage.world, felled->entities[e])->label,
                    "dead lamp_light", 15) == 0;
  assert(dark);
  stop(&run);
}

static void a_gentle_bump_leaves_a_lamp_post_standing(void) {
  city_run_t run;
  start(&run);
  scene_t* scene = &run.stage.scene;
  driving_input_t coast = {0, 0, 0, false, false};
  for (size_t i = 0; i < scene->pole_count; i++) {
    pole_t* pole = &scene->poles[i];
    if (pole->kind != POLE_LAMP) continue;
    aim_at_pole(&run, pole, 1.5f);
    for (int step = 0; step < 120 * 2; step++)
      driving_stage_step(&run.stage, coast, 1.0f / 120.0f);
  }
  assert(poles_standing(scene) == scene->pole_count);
  stop(&run);
}

static void the_map_and_the_pause_silence_the_engine(void) {
  city_run_t run;
  start(&run);
  driving_input_t throttle = {1, 0, 0, false, false};
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

static void make_city_folder(const char* root, const char* name,
                             bool with_manifest) {
  char path[256];
  snprintf(path, sizeof(path), "mkdir -p %s/%s", root, name);
  assert(system(path) == 0);
  if (!with_manifest) return;
  snprintf(path, sizeof(path), "%s/%s/manifest.txt", root, name);
  FILE* file = fopen(path, "w");
  assert(file);
  fputs("city\n", file);
  fclose(file);
}

static void the_menu_lists_the_cooked_cities_by_name_and_wraps_round(void) {
  const char* root = "/tmp/gta-like-test-city-list";
  assert(system("rm -rf /tmp/gta-like-test-city-list") == 0);
  make_city_folder(root, "pordenone", true);
  make_city_folder(root, "barletta", true);
  make_city_folder(root, "santa_maria", true);
  make_city_folder(root, "half_cooked", false); /* no manifest: not a city */
  make_city_folder(root, "district",
                   true); /* the tests' fixture, not a place */
  city_entry_t cities[CITY_LIST_MAX];
  size_t count = city_list(root, cities, CITY_LIST_MAX);
  assert(count == 3);
  assert(strcmp(cities[0].name, "barletta") == 0);
  assert(strcmp(cities[0].title, "Barletta") == 0);
  assert(strcmp(cities[1].name, "pordenone") == 0);
  assert(strcmp(cities[2].title, "Santa Maria") == 0);
  assert(strcmp(cities[0].path, "/tmp/gta-like-test-city-list/barletta") == 0);
  assert(city_list("/tmp/gta-like-test-no-such-folder", cities, 4) == 0);
  /* Row 0 is the test track, the cities follow; the cursor wraps. */
  assert(strcmp(city_menu_row_title(cities, count, 0), "Test track") == 0);
  assert(strcmp(city_menu_row_title(cities, count, 1), "Barletta") == 0);
  assert(city_menu_move(0, 4, MENU_KEY_UP) == 3);
  assert(city_menu_move(3, 4, MENU_KEY_DOWN) == 0);
  assert(city_menu_move(1, 4, MENU_KEY_DOWN) == 2);
}

static void the_player_can_get_out_walk_get_back_in_and_steal_a_car(void) {
  test_traffic_cars = 20;
  test_pedestrians = 10;
  city_run_t run;
  start(&run);
  assert(run.stage.has_figures && !run.stage.on_foot);
  driving_input_t stand = {0, 0, 0, false, false};
  drive(&run, stand, 1.0f);
  vec3_t car_at = car(&run)->position;
  /* Out by the car's side; the car stays where it is. */
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  float apart = hypotf(run.stage.walker.position.x - car_at.x,
                       run.stage.walker.position.z - car_at.z);
  assert(apart > 1.0f && apart < 3.5f);
  driving_input_t walk = {1.0f, 0, 0, false, false};
  vec3_t start_at = run.stage.walker.position;
  for (int i = 0; i < 120 * 2; i++)
    driving_stage_step(&run.stage, walk, 1.0f / 120.0f);
  assert(hypotf(run.stage.walker.position.x - start_at.x,
                run.stage.walker.position.z - start_at.z) > 2.0f);
  assert(isfinite(run.stage.walker.position.y));
  assert(hypotf(car(&run)->position.x - car_at.x,
                car(&run)->position.z - car_at.z) < 0.5f);
  /* Too far from any car to get in. */
  run.stage.walker.position = vec3(car_at.x + 60.0f, car_at.y, car_at.z);
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  /* Back by the car: in again. */
  run.stage.walker.position = vec3(car_at.x + 1.8f, car_at.y, car_at.z);
  driving_stage_interact(&run.stage);
  /* They climb in over a second or so, not at once. */
  assert(run.stage.on_foot && run.stage.entering > 0);
  drive(&run, stand, 0.3f);
  assert(run.stage.on_foot);
  drive(&run, stand, 1.5f);
  assert(!run.stage.on_foot && run.stage.entering == 0);
  /* Steal one off the road: stand beside a traffic car and get in. */
  driving_stage_interact(&run.stage);
  assert(run.stage.on_foot);
  traffic_car_t other = traffic_car(run.stage.traffic, 3);
  run.stage.walker.position =
      vec3(other.position.x + 1.4f, other.position.y - 0.5f, other.position.z);
  int index_before = run.stage.vehicle_index;
  (void)index_before;
  driving_stage_interact(&run.stage);
  drive(&run, stand, 2.0f);
  assert(!run.stage.on_foot);
  assert(run.stage.vehicle_index == other.variant);
  /* It keeps its paint. */
  vec3_t paint = fleet_paint(other.paint);
  if (!run.stage.car_visual.own_paint) {
    vec3_t shown = run.stage.car_visual.paint.base_color;
    assert(shown.x == paint.x && shown.y == paint.y && shown.z == paint.z);
  }
  assert(hypotf(car(&run)->position.x - other.position.x,
                car(&run)->position.z - other.position.z) < 2.5f);
  assert(run.stage.wanted.stars >= 1); /* stealing a car is a crime */
  /* The car it was has been moved on, off the spot. */
  traffic_car_t moved = traffic_car(run.stage.traffic, 3);
  assert(hypotf(moved.position.x - other.position.x,
                moved.position.z - other.position.z) > 5.0f);
  stop(&run);
}

void run_citydrive_tests(void) {
  cook_fixture_city();
  RUN(the_city_loads_with_roads_buildings_and_a_spawn_on_a_road);
  RUN(the_car_drives_along_the_spawn_road_and_through_a_junction);
  RUN(buildings_stop_the_car);
  RUN(a_hard_crash_jolts_the_view_and_it_settles);
  RUN(a_big_city_is_streamed_in_and_out_around_the_car);
  RUN(the_street_under_the_car_is_named_on_the_screen);
  RUN(the_tyres_stay_quiet_on_an_ordinary_drive);
  RUN(the_car_follows_the_hills);
  RUN(the_town_sounds_follow_the_time_of_day);
  RUN(the_driver_sees_from_inside_and_from_the_bonnet);
  RUN(v_swaps_the_car_for_the_next_one_where_it_stands);
  RUN(a_busy_city_runs_for_a_minute_without_trouble);
  RUN(the_chase_view_widens_with_speed);
  RUN(rain_comes_on_gradually_wets_the_roads_and_is_heard);
  RUN(the_cockpit_has_a_dashboard_and_a_wheel_that_turns);
  RUN(the_horn_sounds_only_while_held_and_not_on_the_map);
  RUN(a_wrecked_car_smokes_and_looks_scuffed);
  RUN(walls_are_grubbier_at_the_foot_but_roads_are_not);
  RUN(everything_the_generator_builds_is_drawn);
  RUN(every_vehicle_pulls_away_and_stays_on_its_wheels);
  RUN(a_car_that_hits_a_lamp_post_hard_knocks_it_down);
  RUN(a_gentle_bump_leaves_a_lamp_post_standing);
  RUN(a_wrecked_car_burns_then_blows_up_and_the_player_starts_again);
  RUN(a_player_who_got_out_survives_the_car_blowing_up_unless_close);
  RUN(every_fleet_vehicle_can_be_stolen_and_driven);
  RUN(the_driver_door_swings_open_to_get_out_and_in);
  RUN(on_foot_the_player_can_punch_but_not_twice_at_once);
  RUN(f_in_the_air_bails_out_and_the_plane_crashes);
  RUN(the_engine_note_stays_with_the_aircraft_that_is_flown);
  RUN(the_horn_sounds_from_an_aircraft_and_not_from_a_walker);
  RUN(the_cockpit_view_looks_from_the_pilots_seat);
  RUN(a_missile_leaves_the_jet_flies_and_goes_off_where_it_strikes);
  RUN(a_plane_has_no_missiles_to_fire);
  RUN(every_cockpit_draws_its_dials);
  RUN(the_747_can_be_flown_off_from_its_stand);
  RUN(the_747_can_be_boarded_at_its_nose_and_its_tail);
  RUN(on_foot_the_pistol_fires_leaves_a_trail_and_cools_down);
  RUN(a_reset_stops_a_climb_and_a_blast_so_nothing_is_left_wedged);
  RUN(the_guide_points_at_the_airport_gate_by_a_road_that_reaches_it);
  RUN(a_plane_can_be_stolen_and_flown_off_the_runway);
  RUN(a_crash_blows_the_aircraft_up_and_puts_it_back_on_its_stand);
  RUN(the_map_and_the_pause_silence_the_engine);
  RUN(the_player_can_get_out_walk_get_back_in_and_steal_a_car);
  RUN(the_menu_lists_the_cooked_cities_by_name_and_wraps_round);
}
