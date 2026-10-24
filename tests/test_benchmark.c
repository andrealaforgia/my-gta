#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "benchmark.h"
#include "city_generator.h"
#include "city_import.h"
#include "osm_reader.h"
#include "test_groups.h"
#include "test_support.h"

static const char* report_path = "/tmp/gta-like-test-benchmark.cfg";

static benchmark_report_t example_report(void) {
  benchmark_report_t report = {0};
  report.width = 1280;
  report.height = 720;
  report.frames = 100;
  report.warmup_frames = 60;
  report.traffic = 20;
  report.pedestrians = 40;
  snprintf(report.city, sizeof(report.city), "district");
  report.config_hash = 1234;
  report.workload_hash = 5678;
  report.frame_average_ms = 8;
  report.frame_p99_ms = 12;
  report.simulation_average_ms = 0.5;
  report.draw_calls_average = 1000;
  report.triangles_average = 100000;
  return report;
}

static void a_report_has_average_and_nearest_rank_p99(void) {
  benchmark_sample_t samples[100];
  for (int i = 0; i < 100; i++) {
    samples[i].frame_ms = 100 - i;
    samples[i].simulation_ms = 2;
    samples[i].draw_calls = 100;
    samples[i].triangles = 400;
  }
  benchmark_report_t report = {0};
  assert(benchmark_summarize(samples, 100, &report));
  assert(report.frames == 100);
  assert(near(report.frame_average_ms, 50.5));
  assert(near(report.frame_p99_ms, 99));
  assert(near(report.simulation_average_ms, 2));
  assert(near(report.draw_calls_average, 100));
  assert(near(report.triangles_average, 400));
  assert(!benchmark_summarize(samples, 0, &report));
  samples[8].frame_ms = NAN;
  assert(!benchmark_summarize(samples, 100, &report));
}

static void regression_checks_allow_exactly_twenty_five_percent(void) {
  benchmark_report_t baseline = example_report();
  for (int metric = 0; metric < 5; metric++) {
    benchmark_report_t measured = baseline;
    double* value = metric == 0   ? &measured.frame_average_ms
                    : metric == 1 ? &measured.frame_p99_ms
                    : metric == 2 ? &measured.simulation_average_ms
                    : metric == 3 ? &measured.draw_calls_average
                                  : &measured.triangles_average;
    *value *= 1.25;
    assert(benchmark_within_budget(&measured, &baseline));
    *value += 0.001;
    assert(!benchmark_within_budget(&measured, &baseline));
  }
  benchmark_report_t measured = baseline;
  measured.width = 1920;
  assert(!benchmark_within_budget(&measured, &baseline));
  measured = baseline;
  measured.config_hash++;
  assert(!benchmark_within_budget(&measured, &baseline));
  measured = baseline;
  measured.traffic++;
  assert(!benchmark_within_budget(&measured, &baseline));
  measured = baseline;
  measured.workload_hash++;
  assert(!benchmark_within_budget(&measured, &baseline));
  measured = baseline;
  snprintf(measured.city, sizeof(measured.city), "another_city");
  assert(!benchmark_within_budget(&measured, &baseline));
}

static void baseline_files_round_trip_and_reject_bad_metrics(void) {
  benchmark_report_t expected = example_report(), loaded = {0};
  FILE* file = fopen(report_path, "w");
  assert(file && benchmark_write(file, &expected));
  assert(fclose(file) == 0);
  assert(benchmark_read(report_path, &loaded));
  assert(loaded.width == expected.width && loaded.height == expected.height);
  assert(loaded.config_hash == expected.config_hash);
  assert(strcmp(loaded.city, expected.city) == 0);
  assert(near(loaded.frame_average_ms, expected.frame_average_ms));
  assert(benchmark_within_budget(&loaded, &expected));
  const char* bad[] = {"frame_average_ms=nan\n", "frame_p99_ms=0\n",
                       "simulation_average_ms=inf\n", "width=1280junk\n",
                       "triangles_average=-4\n"};
  char valid_text[2048];
  file = fopen(report_path, "r");
  assert(file);
  size_t size = fread(valid_text, 1, sizeof(valid_text) - 1, file);
  valid_text[size] = '\0';
  fclose(file);
  for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    file = fopen(report_path, "w");
    assert(file);
    size_t key_length = (size_t)(strchr(bad[i], '=') - bad[i]);
    char key[64];
    assert(key_length < sizeof(key));
    memcpy(key, bad[i], key_length);
    key[key_length] = '\0';
    const char* field = strstr(valid_text, key);
    assert(field && strncmp(field, bad[i], key_length) == 0);
    assert(fwrite(valid_text, 1, (size_t)(field - valid_text), file) ==
           (size_t)(field - valid_text));
    fputs(bad[i], file);
    fputs(strchr(field, '\n') + 1, file);
    assert(fclose(file) == 0);
    assert(!benchmark_read(report_path, &loaded));
  }
  file = fopen(report_path, "w");
  assert(file && benchmark_write(file, &expected));
  fputs("frame_average_ms=9\n", file);
  fclose(file);
  assert(!benchmark_read(report_path, &loaded));
  file = fopen(report_path, "w");
  assert(file);
  fputs("frame_average_ms=8\n", file);
  fclose(file);
  assert(!benchmark_read(report_path, &loaded));
  remove(report_path);
  assert(!benchmark_read(report_path, &loaded));
}

static void the_camera_route_is_repeatable_and_moves_over_the_city(void) {
  scene_t scene = {0};
  city_package_t package = {0};
  city_chunk_t chunk = {0};
  chunk.bounds = aabb_from_center(vec3(0, 100, 0), vec3(1000, 100, 1000));
  package.chunks = &chunk;
  package.chunk_count = 1;
  scene.package = &package;
  render_camera_t first = benchmark_camera(&scene, 0);
  render_camera_t again = benchmark_camera(&scene, 0);
  render_camera_t later = benchmark_camera(&scene, 7.5f);
  assert(vec3_distance(first.eye, again.eye) == 0);
  assert(vec3_distance(first.eye, later.eye) > 50);
  assert(first.eye.y > 20 && later.eye.y > 20);
  assert(first.target.y < aabb_center(chunk.bounds).y);
  assert(hypotf(first.target.x - first.eye.x, first.target.z - first.eye.z) <
         hypotf(first.eye.x, first.eye.z));
  assert(isfinite(later.target.x) && isfinite(later.target.z));
  assert(vec3_distance(later.eye, later.target) > 1);
}

static void a_short_benchmark_exercises_the_stage_and_writes_its_report(void) {
  const char* previous_debug = getenv("GTA_DEBUG_SHADOWS");
  char* saved_debug = NULL;
  if (previous_debug) {
    saved_debug = malloc(strlen(previous_debug) + 1);
    assert(saved_debug);
    memcpy(saved_debug, previous_debug, strlen(previous_debug) + 1);
  }
  assert(unsetenv("GTA_DEBUG_SHADOWS") == 0);
  const char* directory = "/tmp/gta-like-test-benchmark-city";
  osm_data_t osm;
  city_model_t model;
  city_package_t package;
  city_generator_stats_t stats;
  assert(osm_load(&osm, "tests/fixtures/district.osm"));
  assert(city_import(&model, &osm, city_import_default_options(),
                     "tests/fixtures/district.osm"));
  osm_free(&osm);
  city_generator_options_t options = city_generator_default_options();
  snprintf(options.airport_side, sizeof(options.airport_side), "-");
  assert(city_generate(&package, &model, &options, "district", &stats));
  assert(city_package_write(&package, directory));
  city_package_free(&package);
  city_model_free(&model);
  window_t window = {0};
  window.drawable_width = 1280;
  window.drawable_height = 720;
  renderer_t* renderer = renderer_create(1280, 720);
  stage_options_t stage_options = {SCENE_CITY, directory, NULL, false, true, 10,
                                   0,          20,        40,   0,     false};
  driving_stage_t stage;
  assert(driving_stage_init(&stage, &window, renderer, NULL, stage_options));
  benchmark_report_t report = {0}, loaded = {0};
  FILE* file = fopen(report_path, "w");
  assert(file);
  assert(benchmark_run(&stage, 3, 2, NULL, file, &report));
  assert(fclose(file) == 0);
  assert(report.frames == 3);
  assert(report.width == 1280 && report.height == 720);
  assert(report.traffic == 20 && report.pedestrians == 40);
  assert(near_to(stage.simulated_seconds, 5.0 / 60.0, 0.0001));
  assert(report.frame_average_ms > 0 && report.frame_p99_ms > 0);
  assert(report.simulation_average_ms > 0 && report.draw_calls_average > 0);
  render_camera_t expected_camera = benchmark_camera(&stage.scene, 2.0f / 60);
  assert(vec3_distance(stage.camera.position, expected_camera.eye) < 0.001f);
  assert(vec3_distance(stage.camera.look_target, expected_camera.target) <
         0.001f);
  render_camera_t actual_camera = driving_stage_camera(&stage);
  mat4_t expected_projection = mat4_multiply(
      mat4_perspective(actual_camera.fov_y, 1280.0f / 720.0f,
                       actual_camera.near_plane, actual_camera.far_plane),
      mat4_look_at(actual_camera.eye, actual_camera.target, actual_camera.up));
  mat4_t rendered = renderer_view_projection(renderer);
  for (int i = 0; i < 16; i++)
    assert(near_to(rendered.m[i], expected_projection.m[i], 0.00001));
  assert(benchmark_read(report_path, &loaded));
  assert(near_to(loaded.frame_average_ms, report.frame_average_ms, 0.000001));
  report.frame_average_ms = 1e-6;
  file = fopen(report_path, "w");
  assert(file && benchmark_write(file, &report));
  fclose(file);
  file = tmpfile();
  assert(file);
  assert(!benchmark_run(&stage, 3, 2, report_path, file, &loaded));
  assert(loaded.frame_average_ms > report.frame_average_ms * 1.25);
  assert(near_to(stage.simulated_seconds, 10.0 / 60.0, 0.0001));
  fclose(file);
  remove(report_path);
  driving_stage_destroy(&stage);
  stage_options.traffic_cars = 0;
  stage_options.pedestrians = 0;
  assert(driving_stage_init(&stage, &window, renderer, NULL, stage_options));
  file = tmpfile();
  assert(file && benchmark_run(&stage, 1, 0, NULL, file, &report));
  assert(report.traffic == 0 && report.pedestrians == 0);
  fclose(file);
  file = fopen(report_path, "w");
  assert(file && benchmark_write(file, &report));
  fclose(file);
  assert(setenv("GTA_DEBUG_SHADOWS", "1", 1) == 0);
  float seconds = stage.simulated_seconds;
  file = tmpfile();
  assert(file && !benchmark_run(&stage, 1, 0, report_path, file, &loaded));
  assert(loaded.workload_hash != report.workload_hash);
  assert(stage.simulated_seconds == seconds);
  fclose(file);
  assert(unsetenv("GTA_DEBUG_SHADOWS") == 0);
  float mass = stage.vehicle.params.mass;
  stage.vehicle.params.mass += 1;
  file = tmpfile();
  assert(file && !benchmark_run(&stage, 1, 0, report_path, file, &loaded));
  assert(loaded.workload_hash != report.workload_hash);
  assert(stage.simulated_seconds == seconds);
  fclose(file);
  stage.vehicle.params.mass = mass;
  remove(report_path);
  if (saved_debug) {
    assert(setenv("GTA_DEBUG_SHADOWS", saved_debug, 1) == 0);
    free(saved_debug);
  } else {
    assert(unsetenv("GTA_DEBUG_SHADOWS") == 0);
  }
  driving_stage_destroy(&stage);
  renderer_destroy(renderer);
}

void run_benchmark_tests(void) {
  RUN(a_report_has_average_and_nearest_rank_p99);
  RUN(regression_checks_allow_exactly_twenty_five_percent);
  RUN(baseline_files_round_trip_and_reject_bad_metrics);
  RUN(the_camera_route_is_repeatable_and_moves_over_the_city);
  RUN(a_short_benchmark_exercises_the_stage_and_writes_its_report);
}
