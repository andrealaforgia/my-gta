#ifndef GAME_SRC_STAGE_SESSION_LOG_H_
#define GAME_SRC_STAGE_SESSION_LOG_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driving_stage.h"

typedef struct {
  uint32_t seed;
  scene_kind_t scene;
  float hour, hour_seconds, rain;
  int traffic, pedestrians;
  uint64_t initial_hash, content_hash, vehicle_hash;
  char scene_name[64];
  char vehicle_name[VEHICLE_NAME_LENGTH];
} session_header_t;

typedef struct {
  double seconds;
  driving_input_t input;
  uint32_t actions;
  bool horn;
} session_frame_t;

typedef struct {
  size_t frames;
  uint64_t state_hash;
  char error[160];
} session_result_t;

bool session_read_header(FILE* file, session_header_t* header);
bool session_write_header(FILE* file, const driving_stage_t* stage);
bool session_record_frame(FILE* file, driving_stage_t* stage,
                          window_events_t events, driving_input_t input,
                          bool horn, double seconds);
bool session_finish(FILE* file, const driving_stage_t* stage, size_t frames);
bool session_replay(FILE* file, driving_stage_t* stage,
                    session_result_t* result);
bool session_run_live(driving_stage_t* stage, FILE* recording,
                      size_t frame_limit);
uint64_t session_state_hash(const driving_stage_t* stage);
uint64_t session_content_hash(const driving_stage_t* stage);
uint64_t session_vehicle_hash(const vehicle_params_t* parameters);

#endif  // GAME_SRC_STAGE_SESSION_LOG_H_
