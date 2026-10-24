#ifndef GAME_SRC_MAIN_BENCHMARK_H_
#define GAME_SRC_MAIN_BENCHMARK_H_

#include <stdio.h>

#include "driving_stage.h"

#define BENCHMARK_FRAMES 1800
#define BENCHMARK_WARMUP_FRAMES 60

typedef struct {
  double frame_ms, simulation_ms;
  int draw_calls, triangles;
} benchmark_sample_t;

typedef struct {
  int width, height, frames, warmup_frames, traffic, pedestrians;
  char city[CITY_PACKAGE_TEXT];
  uint64_t config_hash, workload_hash;
  double frame_average_ms, frame_p99_ms, simulation_average_ms;
  double draw_calls_average, triangles_average;
} benchmark_report_t;

bool benchmark_summarize(const benchmark_sample_t* samples, size_t count,
                         benchmark_report_t* report);
bool benchmark_read(const char* path, benchmark_report_t* report);
bool benchmark_write(FILE* file, const benchmark_report_t* report);
bool benchmark_within_budget(const benchmark_report_t* measured,
                             const benchmark_report_t* baseline);
render_camera_t benchmark_camera(const scene_t* scene, float seconds);
bool benchmark_run(driving_stage_t* stage, int frames, int warmup_frames,
                   const char* baseline_path, FILE* output,
                   benchmark_report_t* report);

#endif  // GAME_SRC_MAIN_BENCHMARK_H_
