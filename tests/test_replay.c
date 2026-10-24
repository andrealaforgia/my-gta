#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "city_generator.h"
#include "city_import.h"
#include "osm_reader.h"
#include "session_log.h"
#include "test_groups.h"
#include "test_support.h"

static stage_options_t track_options(void) {
  stage_options_t options = {
      SCENE_TEST_TRACK, NULL, NULL, false, true, 10, 30, 0, 0, 0, false};
  return options;
}

static void start(driving_stage_t* stage, renderer_t* renderer,
                  window_t* window, stage_options_t options, uint32_t seed) {
  assert(
      driving_stage_init_seeded(stage, window, renderer, NULL, options, seed));
}

static FILE* copy_text(FILE* original, const char* suffix) {
  FILE* result = tmpfile();
  assert(result);
  rewind(original);
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), original)) fputs(buffer, result);
  if (suffix) fputs(suffix, result);
  rewind(result);
  return result;
}

static void recorded_actions_replay_the_same_state_at_every_frame(void) {
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  driving_stage_t stage;
  stage_options_t options = track_options();
  start(&stage, renderer, &window, options, 71);
  FILE* log = tmpfile();
  assert(log && session_write_header(log, &stage));
  for (int i = 0; i < 150; i++) {
    window_events_t events = {0};
    events.toggle_weather = i == 5;
    events.toggle_pause = i == 20 || i == 23;
    events.toggle_map = i == 30 || i == 32;
    events.toggle_camera = i == 40;
    events.reset_vehicle = i == 55;
    events.interact = i == 70;
    events.fire = i == 100;
    events.punch = i == 110;
    events.next_vehicle = i == 120;
    events.next_station = i == 130;
    driving_input_t input = {i < 50 ? 0.8f : 0.0f, 0, 0.1f, false, false};
    double seconds = i == 10 ? 0.5 : 1.0 / 60.0;
    float before_seconds = stage.simulated_seconds;
    float before_hour = stage.hour, before_rain = stage.rain;
    assert(session_record_frame(log, &stage, events, input, i == 8, seconds));
    if (i == 20 || i == 30) {
      assert(stage.simulated_seconds == before_seconds);
      assert(stage.hour == before_hour && stage.rain == before_rain);
    }
    if (i == 10) {
      assert(near_to(stage.simulated_seconds - before_seconds, 0.25, 0.0001));
      assert(near_to(stage.hour - before_hour, 0.5 / 30, 0.00001));
      assert(near_to(stage.rain - before_rain, 0.5 / 6, 0.00001));
    }
    if (i == 55)
      assert(vec3_distance(
                 physics_body(stage.physics, stage.vehicle.body)->position,
                 stage.scene.spawn_position) < 1);
    if (i == 120) assert(stage.vehicle_index == 1);
  }
  uint64_t expected = session_state_hash(&stage);
  assert(stage.rain > 0 && stage.hour > 10);
  assert(session_finish(log, &stage, 150));
  driving_stage_destroy(&stage);
  start(&stage, renderer, &window, options, 71);
  session_result_t result;
  rewind(log);
  assert(session_replay(log, &stage, &result));
  assert(result.frames == 150 && result.state_hash == expected);
  assert(session_state_hash(&stage) == expected);
  driving_stage_destroy(&stage);

  start(&stage, renderer, &window, options, 72);
  rewind(log);
  assert(!session_replay(log, &stage, &result));
  assert(result.frames == 0 && strstr(result.error, "initial"));
  driving_stage_destroy(&stage);

  start(&stage, renderer, &window, options, 71);
  FILE* appended = copy_text(log, "junk\n");
  assert(!session_replay(appended, &stage, &result));
  assert(strstr(result.error, "trailing"));
  fclose(appended);
  driving_stage_destroy(&stage);
  start(&stage, renderer, &window, options, 71);
  FILE* tampered = tmpfile();
  assert(tampered);
  rewind(log);
  char row[1024];
  int line = 0;
  while (fgets(row, sizeof(row), log)) {
    if (line++ == 8) {
      char* checksum = strrchr(row, ' ');
      assert(checksum);
      snprintf(checksum, sizeof(row) - (size_t)(checksum - row), " 0\n");
    }
    fputs(row, tampered);
  }
  rewind(tampered);
  assert(!session_replay(tampered, &stage, &result));
  assert(result.frames == 2 && strstr(result.error, "diverged at frame 3"));
  fclose(tampered);
  driving_stage_destroy(&stage);
  fclose(log);
  renderer_destroy(renderer);
}

static void malformed_frames_and_tampering_fail_at_the_first_bad_frame(void) {
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  driving_stage_t stage;
  start(&stage, renderer, &window, track_options(), 2026);
  FILE* header = tmpfile();
  assert(header && session_write_header(header, &stage));
  const char* rows[] = {"input nan 0 0 0 0 0 0 0\n",
                        "input 0.02 2 0 0 0 0 0 0\n",
                        "input 0.02 0 0 0 0 0 0 4294967295\n",
                        "input 0.02 0 0 0 0 0 0 0 extra\n",
                        "",
                        "input 0.02 0 0 0 0 0 0 0\nstate 0\n"};
  for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    FILE* log = copy_text(header, rows[i]);
    session_result_t result;
    assert(!session_replay(log, &stage, &result));
    assert(result.frames == 0);
    if (i < 4) assert(strstr(result.error, "invalid frame 1"));
    if (i == 4) assert(strstr(result.error, "truncated"));
    if (i == 5) assert(strstr(result.error, "diverged at frame 1"));
    fclose(log);
  }
  FILE* wrong_version = tmpfile();
  assert(wrong_version);
  rewind(header);
  char valid_header[1024];
  assert(fgets(valid_header, sizeof(valid_header), header));
  assert(strncmp(valid_header, "gta-replay 1 ", 13) == 0);
  fprintf(wrong_version, "gta-replay 9 %s", valid_header + 13);
  while (fgets(valid_header, sizeof(valid_header), header))
    fputs(valid_header, wrong_version);
  rewind(wrong_version);
  session_header_t parsed;
  assert(!session_read_header(wrong_version, &parsed));
  fclose(wrong_version);
  fclose(header);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
}

static void seed_controls_the_city_and_actor_state_is_observed(void) {
  const char* directory = "/tmp/gta-like-test-replay-city";
  osm_data_t osm;
  city_model_t model;
  city_package_t package;
  city_generator_stats_t stats;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  assert(city_import(&model, &osm, city_import_default_options(),
                     "tests/fixtures/district.osm"));
  osm_free(&osm);
  city_generator_options_t settings = city_generator_default_options();
  snprintf(settings.airport_side, sizeof(settings.airport_side), "-");
  assert(city_generate(&package, &model, &settings, "district", &stats));
  assert(city_package_write(&package, directory));
  city_package_free(&package);
  city_model_free(&model);
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  stage_options_t options = track_options();
  options.scene = SCENE_CITY;
  options.city_package = directory;
  options.traffic_cars = 12;
  options.pedestrians = 20;
  driving_stage_t stage;
  start(&stage, renderer, &window, options, 9);
  uint64_t initial = session_state_hash(&stage);
  stage.map_texture.id++;
  stage.physics_ms = 999;
  assert(session_state_hash(&stage) == initial);
  car_visual_t original_visual = stage.car_visual;
  car_visual_set_paint(&stage.car_visual, vec3(0.2f, 0.7f, 0.5f));
  assert(session_state_hash(&stage) != initial);
  stage.car_visual = original_visual;
  stage.vehicle.damage += 0.1f;
  assert(session_state_hash(&stage) != initial);
  stage.vehicle.damage -= 0.1f;
  stage.walker.position.x += 1;
  assert(session_state_hash(&stage) != initial);
  stage.walker.position.x -= 1;
  stage.wanted.stars++;
  assert(session_state_hash(&stage) != initial);
  stage.wanted.stars--;
  stage.rain = 0.5f;
  assert(session_state_hash(&stage) != initial);
  stage.rain = 0;
  stage.aircraft_count = 1;
  stage.aircraft[0].power = 0.5f;
  assert(session_state_hash(&stage) != initial);
  stage.aircraft_count = 0;
  stage.missiles[0].active = true;
  assert(session_state_hash(&stage) != initial);
  stage.missiles[0].active = false;
  stage.chute.active = true;
  assert(session_state_hash(&stage) != initial);
  stage.chute.active = false;
  assert(stage.scene.pole_count > 0);
  stage.scene.poles[0].state = POLE_DOWN;
  assert(session_state_hash(&stage) != initial);
  stage.scene.poles[0].state = POLE_STANDING;
  assert(session_state_hash(&stage) == initial);
  uint64_t content = session_content_hash(&stage);
  float original_yaw = stage.scene.package->spawn_yaw;
  stage.scene.package->spawn_yaw += 0.1f;
  assert(session_content_hash(&stage) != content);
  stage.scene.package->spawn_yaw = original_yaw;
  stage.scene.package->aircraft_count = 1;
  assert(session_content_hash(&stage) != content);
  stage.scene.package->aircraft_count = 0;
  traffic_car_t first = traffic_car(stage.traffic, 0);
  physics_world_t* physics = physics_world_create();
  traffic_t* expected =
      traffic_create(stage.scene.routes, stage.scene.route_count, physics,
                     &stage.fleet, 12, 9);
  assert(expected);
  traffic_car_t seeded = traffic_car(expected, 0);
  assert(first.position.x == seeded.position.x &&
         first.position.z == seeded.position.z && first.paint == seeded.paint);
  pedestrians_t* expected_people = pedestrians_create(
      stage.scene.routes, stage.scene.route_count, physics, 20, 10);
  assert(expected_people);
  pedestrian_t actual_person = pedestrians_get(stage.people, 0);
  pedestrian_t seeded_person = pedestrians_get(expected_people, 0);
  assert(actual_person.outfit == seeded_person.outfit &&
         actual_person.position.x == seeded_person.position.x &&
         actual_person.position.z == seeded_person.position.z);
  pedestrians_destroy(expected_people);
  traffic_destroy(expected, physics);
  physics_world_destroy(physics);
  traffic_car_hold(stage.traffic, 0, true);
  traffic_car_place(stage.traffic, 0, first.route, 1, 15);
  assert(session_state_hash(&stage) != initial);
  uint64_t before = session_state_hash(&stage);
  pedestrians_place(stage.people, 0, 0, 1, 1, 10);
  assert(session_state_hash(&stage) != before);
  window_events_t events = {0};
  events.interact = true;
  driving_input_t idle = {0};
  driving_stage_apply_frame(&stage, events, idle, false, 0.01);
  assert(stage.on_foot);
  events.interact = false;
  events.fire = true;
  driving_stage_apply_frame(&stage, events, idle, false, 0.01);
  assert(stage.shot_cooldown > 0);
  events.fire = false;
  events.punch = true;
  driving_stage_apply_frame(&stage, events, idle, false, 0.01);
  assert(stage.punch_clock > 0);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
}

static void a_pending_crash_input_is_applied_but_never_reported_verified(void) {
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  driving_stage_t stage;
  start(&stage, renderer, &window, track_options(), 2026);
  FILE* prefix = tmpfile();
  assert(prefix && session_write_header(prefix, &stage));
  window_events_t events = {0};
  driving_input_t input = {0};
  assert(session_record_frame(prefix, &stage, events, input, false, 1.0 / 60));
  assert(session_record_frame(prefix, &stage, events, input, false, 1.0 / 60));
  driving_stage_destroy(&stage);
  start(&stage, renderer, &window, track_options(), 2026);
  rewind(prefix);
  session_result_t result;
  assert(!session_replay(prefix, &stage, &result));
  assert(result.frames == 2 && strstr(result.error, "missing footer"));
  driving_stage_destroy(&stage);
  start(&stage, renderer, &window, track_options(), 2026);
  FILE* pending = copy_text(prefix, "input 0x1p-2 1 0 0 0 0 0 0\n");
  assert(!session_replay(pending, &stage, &result));
  assert(stage.simulated_seconds > 0.28f);
  assert(result.frames == 2 && strstr(result.error, "unverified frame 3"));
  assert(strstr(result.error, "missing state"));
  fclose(pending);
  fclose(prefix);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
}

static void quitting_the_live_loop_writes_one_complete_session(void) {
  assert(SDL_InitSubSystem(SDL_INIT_EVENTS) == 0);
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  driving_stage_t stage;
  start(&stage, renderer, &window, track_options(), 2026);
  FILE* file = tmpfile();
  assert(file);
  SDL_Event quit = {0};
  quit.type = SDL_QUIT;
  assert(SDL_PushEvent(&quit) == 1);
  assert(session_run_live(&stage, file, 0));
  assert(stage.quit);
  driving_stage_destroy(&stage);
  start(&stage, renderer, &window, track_options(), 2026);
  rewind(file);
  session_result_t result;
  assert(session_replay(file, &stage, &result));
  assert(result.frames == 1 && stage.quit);
  fclose(file);
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
  SDL_QuitSubSystem(SDL_INIT_EVENTS);
}

void run_replay_tests(void) {
  RUN(a_pending_crash_input_is_applied_but_never_reported_verified);
  RUN(recorded_actions_replay_the_same_state_at_every_frame);
  RUN(malformed_frames_and_tampering_fail_at_the_first_bad_frame);
  RUN(seed_controls_the_city_and_actor_state_is_observed);
  RUN(quitting_the_live_loop_writes_one_complete_session);
}
