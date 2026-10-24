#include "benchmark.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "session_log.h"

#define ROUTE_VERSION 2
#define ROUTE_SECONDS 30.0f
#define ROUTE_MAX_RADIUS 750.0f
#define ROUTE_RADIUS_FRACTION 0.45f
#define ROUTE_CLEARANCE 80.0f
#define ROUTE_LOOK_FRACTION 0.3f
#define BENCHMARK_STEP (1.0 / 60.0)
#define BUDGET_FACTOR 1.25

static const char* metric_keys[] = {"frame_average_ms", "frame_p99_ms",
                                    "simulation_average_ms",
                                    "draw_calls_average", "triangles_average"};

static void report_metrics(const benchmark_report_t* report, double values[5]) {
  values[0] = report->frame_average_ms;
  values[1] = report->frame_p99_ms;
  values[2] = report->simulation_average_ms;
  values[3] = report->draw_calls_average;
  values[4] = report->triangles_average;
}

static bool positive_finite(double value) {
  return isfinite(value) && value > 0;
}

static bool valid_report(const benchmark_report_t* report) {
  if (report->width <= 0 || report->height <= 0 || report->frames <= 0 ||
      report->warmup_frames < 0 || report->traffic < 0 ||
      report->pedestrians < 0 || !report->city[0])
    return false;
  double values[5];
  report_metrics(report, values);
  for (int i = 0; i < 5; i++)
    if (!positive_finite(values[i])) return false;
  return true;
}

static int compare_times(const void* a, const void* b) {
  double left = *(const double*)a, right = *(const double*)b;
  return (left > right) - (left < right);
}

bool benchmark_summarize(const benchmark_sample_t* samples, size_t count,
                         benchmark_report_t* report) {
  if (!samples || !report || count == 0 || count > INT_MAX) return false;
  double* times = malloc(count * sizeof(double));
  if (!times) return false;
  double frame = 0, simulation = 0, draws = 0, triangles = 0;
  bool valid = true;
  for (size_t i = 0; i < count; i++) {
    const benchmark_sample_t* sample = &samples[i];
    if (!positive_finite(sample->frame_ms) ||
        !positive_finite(sample->simulation_ms) || sample->draw_calls < 0 ||
        sample->triangles < 0) {
      valid = false;
      break;
    }
    times[i] = sample->frame_ms;
    frame += sample->frame_ms;
    simulation += sample->simulation_ms;
    draws += sample->draw_calls;
    triangles += sample->triangles;
  }
  if (valid) {
    qsort(times, count, sizeof(double), compare_times);
    report->frames = (int)count;
    report->frame_average_ms = frame / count;
    report->frame_p99_ms = times[(size_t)ceil(0.99 * (double)count) - 1];
    report->simulation_average_ms = simulation / count;
    report->draw_calls_average = draws / count;
    report->triangles_average = triangles / count;
  }
  free(times);
  return valid;
}

static bool read_number(config_t* config, const char* key, double* out) {
  const char* text = config_get_string(config, key, "");
  char* end;
  errno = 0;
  double number = strtod(text, &end);
  if (end == text || *end || errno || !isfinite(number)) return false;
  *out = number;
  return true;
}

static bool read_integer(config_t* config, const char* key, int* out) {
  double number;
  if (!read_number(config, key, &number) || number < 0 || number > INT_MAX ||
      floor(number) != number)
    return false;
  *out = (int)number;
  return true;
}

static bool read_hash(config_t* config, const char* key, uint64_t* out) {
  const char* text = config_get_string(config, key, "");
  char* end;
  errno = 0;
  unsigned long long number = strtoull(text, &end, 16);
  if (end == text || *end || errno || text[0] == '-') return false;
  *out = (uint64_t)number;
  return true;
}

bool benchmark_read(const char* path, benchmark_report_t* report) {
  config_t config;
  if (!config_load(&config, path)) return false;
  benchmark_report_t parsed = {0};
  double values[5];
  bool valid = config_malformed_count(&config) == 0 &&
               config_count(&config) == 14 &&
               read_integer(&config, "width", &parsed.width) &&
               read_integer(&config, "height", &parsed.height) &&
               read_integer(&config, "frames", &parsed.frames) &&
               read_integer(&config, "warmup_frames", &parsed.warmup_frames) &&
               read_integer(&config, "traffic", &parsed.traffic) &&
               read_integer(&config, "pedestrians", &parsed.pedestrians) &&
               read_hash(&config, "config_hash", &parsed.config_hash) &&
               read_hash(&config, "workload_hash", &parsed.workload_hash);
  const char* city = config_get_string(&config, "city", "");
  if (strlen(city) >= sizeof(parsed.city)) valid = false;
  snprintf(parsed.city, sizeof(parsed.city), "%s", city);
  for (int i = 0; i < 5; i++)
    if (!read_number(&config, metric_keys[i], &values[i])) valid = false;
  if (valid) {
    parsed.frame_average_ms = values[0];
    parsed.frame_p99_ms = values[1];
    parsed.simulation_average_ms = values[2];
    parsed.draw_calls_average = values[3];
    parsed.triangles_average = values[4];
    valid = valid_report(&parsed);
  }
  if (valid) *report = parsed;
  config_free(&config);
  return valid;
}

bool benchmark_write(FILE* file, const benchmark_report_t* report) {
  if (!file || !valid_report(report)) return false;
  if (fprintf(file,
              "width=%d\nheight=%d\nframes=%d\nwarmup_frames=%d\n"
              "traffic=%d\npedestrians=%d\ncity=%s\n"
              "config_hash=%llx\nworkload_hash=%llx\n",
              report->width, report->height, report->frames,
              report->warmup_frames, report->traffic, report->pedestrians,
              report->city, (unsigned long long)report->config_hash,
              (unsigned long long)report->workload_hash) < 0)
    return false;
  double values[5];
  report_metrics(report, values);
  for (int i = 0; i < 5; i++)
    if (fprintf(file, "%s=%.9f\n", metric_keys[i], values[i]) < 0) return false;
  return !ferror(file);
}

static bool same_workload(const benchmark_report_t* a,
                          const benchmark_report_t* b) {
  return a->width == b->width && a->height == b->height &&
         a->frames == b->frames && a->warmup_frames == b->warmup_frames &&
         a->traffic == b->traffic && a->pedestrians == b->pedestrians &&
         a->config_hash == b->config_hash &&
         a->workload_hash == b->workload_hash && strcmp(a->city, b->city) == 0;
}

bool benchmark_within_budget(const benchmark_report_t* measured,
                             const benchmark_report_t* baseline) {
  if (!valid_report(measured) || !valid_report(baseline) ||
      !same_workload(measured, baseline))
    return false;
  double actual[5], previous[5];
  report_metrics(measured, actual);
  report_metrics(baseline, previous);
  for (int i = 0; i < 5; i++)
    if (actual[i] > previous[i] * BUDGET_FACTOR) return false;
  return true;
}

render_camera_t benchmark_camera(const scene_t* scene, float seconds) {
  aabb_t bounds = aabb_empty();
  for (size_t i = 0; i < scene->package->chunk_count; i++)
    bounds = aabb_merge(bounds, scene->package->chunks[i].bounds);
  vec3_t centre = aabb_center(bounds);
  vec3_t half = aabb_half_extents(bounds);
  float angle = seconds * (2.0f * (float)M_PI / ROUTE_SECONDS);
  float rx = fminf(ROUTE_MAX_RADIUS, half.x * ROUTE_RADIUS_FRACTION);
  float rz = fminf(ROUTE_MAX_RADIUS, half.z * ROUTE_RADIUS_FRACTION);
  vec3_t eye = vec3(centre.x + cosf(angle) * rx, bounds.max.y + ROUTE_CLEARANCE,
                    centre.z + sinf(angle) * rz);
  vec3_t target =
      vec3(eye.x + (centre.x - eye.x) * ROUTE_LOOK_FRACTION, bounds.min.y,
           eye.z + (centre.z - eye.z) * ROUTE_LOOK_FRACTION);
  render_camera_t camera = {eye, target, {0, 1, 0}, 1.05f, 1.0f, 4000.0f};
  return camera;
}

static uint64_t hash_bytes(uint64_t hash, const void* bytes, size_t count) {
  const unsigned char* data = bytes;
  for (size_t i = 0; i < count; i++) {
    hash ^= data[i];
    hash *= 1099511628211ull;
  }
  return hash;
}

static uint64_t workload_hash(const driving_stage_t* stage) {
  uint64_t hash = 14695981039346656037ull;
  char settings[256];
  int size =
      snprintf(settings, sizeof(settings), "%d %.9g %.9g %.9g %.9g %d %d %d",
               ROUTE_VERSION, stage->hour, stage->hour_seconds, stage->rain,
               stage->rain_target, getenv("GTA_NO_POST") != NULL,
               getenv("GTA_DEBUG_SHADOWS") != NULL,
               renderer_post_enabled(stage->renderer));
  hash = hash_bytes(hash, settings, (size_t)size);
  uint64_t vehicle = session_vehicle_hash(&stage->vehicle.params);
  hash = hash_bytes(hash, &vehicle, sizeof(vehicle));
  hash = hash_bytes(hash, &stage->seed, sizeof(stage->seed));
  uint64_t content = session_content_hash(stage);
  hash = hash_bytes(hash, &content, sizeof(content));
  return hash;
}

static benchmark_sample_t measure_frame(driving_stage_t* stage, float seconds) {
  double started = platform_seconds();
  render_camera_t camera = benchmark_camera(&stage->scene, seconds);
  stage->walker.position = vec3_add(camera.eye, vec3(0, -20, 0));
  double simulation = platform_seconds();
  driving_input_t idle = {0, 0, 0, true, false};
  driving_stage_advance(stage, idle, BENCHMARK_STEP);
  simulation = (platform_seconds() - simulation) * 1000.0;
  stage->camera.position = camera.eye;
  stage->camera.look_target = camera.target;
  stage->camera.params.fov_y = camera.fov_y;
  driving_stage_render(stage);
  renderer_wait_idle(stage->renderer);
  if (stage->window->window) window_swap(stage->window);
  render_stats_t stats = renderer_stats(stage->renderer);
  benchmark_sample_t sample = {(platform_seconds() - started) * 1000.0,
                               simulation, stats.draw_calls, stats.triangles};
  return sample;
}

static benchmark_report_t describe_workload(const driving_stage_t* stage,
                                            int frames, int warmup_frames) {
  benchmark_report_t report = {0};
  report.width = stage->window->drawable_width;
  report.height = stage->window->drawable_height;
  report.frames = frames;
  report.warmup_frames = warmup_frames;
  report.traffic = stage->traffic ? (int)traffic_car_count(stage->traffic) : 0;
  report.pedestrians =
      stage->people ? (int)pedestrians_count(stage->people) : 0;
  report.config_hash = stage->scene.package->config_hash;
  report.workload_hash = workload_hash(stage);
  snprintf(report.city, sizeof(report.city), "%s", stage->scene.name);
  return report;
}

bool benchmark_run(driving_stage_t* stage, int frames, int warmup_frames,
                   const char* baseline_path, FILE* output,
                   benchmark_report_t* report) {
  if (!stage->scene.package || frames <= 0 || warmup_frames < 0 || !output)
    return false;
  *report = describe_workload(stage, frames, warmup_frames);
  benchmark_report_t baseline;
  if (baseline_path && (!benchmark_read(baseline_path, &baseline) ||
                        !same_workload(report, &baseline))) {
    fprintf(stderr, "benchmark baseline is invalid or workload differs\n");
    return false;
  }
  benchmark_sample_t* samples = calloc((size_t)frames, sizeof(*samples));
  if (!samples) return false;
  stage->on_foot = true;
  stage->view_mode = 0;
  stage->show_debug = false;
  for (int i = 0; i < warmup_frames; i++) measure_frame(stage, 0);
  for (int i = 0; i < frames; i++)
    samples[i] = measure_frame(stage, (float)i / 60.0f);
  bool success = benchmark_summarize(samples, (size_t)frames, report) &&
                 benchmark_write(output, report);
  free(samples);
  if (success && baseline_path && !benchmark_within_budget(report, &baseline)) {
    fprintf(stderr, "benchmark exceeded its baseline by more than 25%%\n");
    success = false;
  }
  return success;
}
