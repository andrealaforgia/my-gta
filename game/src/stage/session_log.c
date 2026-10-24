#include "session_log.h"

#include <errno.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SESSION_ACTIONS ((1u << 14) - 1u)
#define SESSION_LINE_SIZE 1024

static int read_tokens(FILE* file, char line[SESSION_LINE_SIZE],
                       char* tokens[16]) {
  if (!fgets(line, SESSION_LINE_SIZE, file)) return ferror(file) ? -1 : 0;
  if (!strchr(line, '\n')) return -1;
  int count = 0;
  char* token = strtok(line, " \t\r\n");
  while (token) {
    if (count == 16) return -1;
    tokens[count++] = token;
    token = strtok(NULL, " \t\r\n");
  }
  return count;
}

static bool unsigned_number(const char* text, int base, uint64_t maximum,
                            uint64_t* value) {
  if (!text[0] || text[0] == '-' || text[0] == '+') return false;
  char* end;
  errno = 0;
  unsigned long long parsed = strtoull(text, &end, base);
  if (end == text || *end || errno || parsed > maximum) return false;
  *value = (uint64_t)parsed;
  return true;
}

static bool finite_number(const char* text, double minimum, double maximum,
                          double* value) {
  char* end;
  errno = 0;
  double parsed = strtod(text, &end);
  if (end == text || *end || errno || !isfinite(parsed) || parsed < minimum ||
      parsed > maximum)
    return false;
  *value = parsed;
  return true;
}

static bool read_label(FILE* file, const char* prefix, char* label,
                       size_t capacity) {
  char line[SESSION_LINE_SIZE];
  if (!fgets(line, sizeof(line), file)) return false;
  size_t prefix_length = strlen(prefix), length = strlen(line);
  if (strncmp(line, prefix, prefix_length) || length <= prefix_length + 1 ||
      line[length - 1] != '\n' || length - prefix_length > capacity)
    return false;
  line[length - 1] = '\0';
  for (const char* p = line + prefix_length; *p; p++)
    if ((unsigned char)*p < 32 || (unsigned char)*p == 127) return false;
  snprintf(label, capacity, "%s", line + prefix_length);
  return true;
}

bool session_read_header(FILE* file, session_header_t* header) {
  char line[SESSION_LINE_SIZE], *tokens[16];
  if (read_tokens(file, line, tokens) != 12 ||
      strcmp(tokens[0], "gta-replay") || strcmp(tokens[1], "1"))
    return false;
  uint64_t seed, scene, cars, people, initial, content, vehicle;
  double hour, rate, rain;
  if (!unsigned_number(tokens[2], 10, UINT32_MAX, &seed) ||
      !unsigned_number(tokens[3], 10, SCENE_CITY, &scene) ||
      !finite_number(tokens[4], 0, 24, &hour) || hour >= 24 ||
      !finite_number(tokens[5], 0, FLT_MAX, &rate) ||
      !finite_number(tokens[6], 0, 1, &rain) ||
      !unsigned_number(tokens[7], 10, INT_MAX, &cars) ||
      !unsigned_number(tokens[8], 10, INT_MAX, &people) ||
      !unsigned_number(tokens[9], 16, UINT64_MAX, &initial) ||
      !unsigned_number(tokens[10], 16, UINT64_MAX, &content) ||
      !unsigned_number(tokens[11], 16, UINT64_MAX, &vehicle))
    return false;
  *header = (session_header_t){(uint32_t)seed,
                               (scene_kind_t)scene,
                               (float)hour,
                               (float)rate,
                               (float)rain,
                               (int)cars,
                               (int)people,
                               initial,
                               content,
                               vehicle,
                               {0},
                               {0}};
  return read_label(file, "scene ", header->scene_name,
                    sizeof(header->scene_name)) &&
         read_label(file, "vehicle ", header->vehicle_name,
                    sizeof(header->vehicle_name));
}

bool session_write_header(FILE* file, const driving_stage_t* stage) {
  return fprintf(
             file,
             "gta-replay 1 %u %d %a %a %a %zu %zu %llx %llx %llx\n"
             "scene %s\nvehicle %s\n",
             stage->seed, stage->scene_kind, (double)stage->hour,
             (double)stage->hour_seconds, (double)stage->rain,
             stage->traffic ? traffic_car_count(stage->traffic) : 0,
             stage->people ? pedestrians_count(stage->people) : 0,
             (unsigned long long)session_state_hash(stage),
             (unsigned long long)session_content_hash(stage),
             (unsigned long long)session_vehicle_hash(&stage->vehicle.params),
             stage->scene.name, stage->vehicle.params.name) > 0;
}

static uint32_t actions_from_events(window_events_t events) {
  const bool flags[] = {
      events.quit,          events.reset_vehicle,  events.toggle_pause,
      events.toggle_camera, events.toggle_debug,   events.toggle_map,
      events.next_vehicle,  events.toggle_weather, events.back_to_menu,
      events.fire,          events.punch,          events.next_station,
      events.interact,      events.toggle_gear};
  uint32_t actions = 0;
  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++)
    if (flags[i]) actions |= 1u << i;
  return actions;
}

static window_events_t events_from_actions(uint32_t actions) {
  window_events_t events = {0};
  bool* flags[] = {
      &events.quit,          &events.reset_vehicle,  &events.toggle_pause,
      &events.toggle_camera, &events.toggle_debug,   &events.toggle_map,
      &events.next_vehicle,  &events.toggle_weather, &events.back_to_menu,
      &events.fire,          &events.punch,          &events.next_station,
      &events.interact,      &events.toggle_gear};
  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++)
    *flags[i] = (actions & (1u << i)) != 0;
  return events;
}

static bool valid_input(driving_input_t input, double seconds) {
  return isfinite(seconds) && seconds >= 0 && seconds <= FLT_MAX &&
         isfinite(input.throttle) && input.throttle >= 0 &&
         input.throttle <= 1 && isfinite(input.brake) && input.brake >= 0 &&
         input.brake <= 1 && isfinite(input.steering) && input.steering >= -1 &&
         input.steering <= 1;
}

bool session_record_frame(FILE* file, driving_stage_t* stage,
                          window_events_t events, driving_input_t input,
                          bool horn, double seconds) {
  if (!valid_input(input, seconds)) return false;
  if (fprintf(file, "input %a %a %a %a %d %d %d %u\n", seconds,
              (double)input.throttle, (double)input.brake,
              (double)input.steering, input.handbrake, input.lower, horn,
              actions_from_events(events)) < 0 ||
      fflush(file) != 0)
    return false;
  driving_stage_apply_frame(stage, events, input, horn, seconds);
  return fprintf(file, "state %llx\n",
                 (unsigned long long)session_state_hash(stage)) > 0 &&
         fflush(file) == 0;
}

bool session_finish(FILE* file, const driving_stage_t* stage, size_t frames) {
  return fprintf(file, "end %zu %llx\n", frames,
                 (unsigned long long)session_state_hash(stage)) > 0 &&
         fflush(file) == 0;
}

static bool parse_frame(char* tokens[16], int count, session_frame_t* frame) {
  if (count != 9 || strcmp(tokens[0], "input")) return false;
  double seconds, throttle, brake, steering;
  uint64_t handbrake, lower, horn, actions;
  if (!finite_number(tokens[1], 0, FLT_MAX, &seconds) ||
      !finite_number(tokens[2], 0, 1, &throttle) ||
      !finite_number(tokens[3], 0, 1, &brake) ||
      !finite_number(tokens[4], -1, 1, &steering) ||
      !unsigned_number(tokens[5], 10, 1, &handbrake) ||
      !unsigned_number(tokens[6], 10, 1, &lower) ||
      !unsigned_number(tokens[7], 10, 1, &horn) ||
      !unsigned_number(tokens[8], 10, SESSION_ACTIONS, &actions))
    return false;
  *frame = (session_frame_t){seconds,
                             {(float)throttle, (float)brake, (float)steering,
                              handbrake != 0, lower != 0},
                             (uint32_t)actions,
                             horn != 0};
  return true;
}

static bool fail(session_result_t* result, const char* message) {
  snprintf(result->error, sizeof(result->error), "%s", message);
  return false;
}

static bool matching_header(const session_header_t* header,
                            const driving_stage_t* stage) {
  return strcmp(header->scene_name, stage->scene.name) == 0 &&
         strcmp(header->vehicle_name, stage->vehicle.params.name) == 0 &&
         header->seed == stage->seed && header->scene == stage->scene_kind &&
         header->hour == stage->hour &&
         header->hour_seconds == stage->hour_seconds &&
         header->rain == stage->rain &&
         header->traffic ==
             (int)(stage->traffic ? traffic_car_count(stage->traffic) : 0) &&
         header->pedestrians ==
             (int)(stage->people ? pedestrians_count(stage->people) : 0) &&
         header->initial_hash == session_state_hash(stage) &&
         header->content_hash == session_content_hash(stage) &&
         header->vehicle_hash == session_vehicle_hash(&stage->vehicle.params);
}

static bool finish_replay(FILE* file, char* tokens[16], int count,
                          session_result_t* result) {
  uint64_t frames, hash;
  if (count != 3 || !unsigned_number(tokens[1], 10, SIZE_MAX, &frames) ||
      !unsigned_number(tokens[2], 16, UINT64_MAX, &hash) ||
      frames != result->frames || hash != result->state_hash)
    return fail(result, "invalid replay footer");
  if (fgetc(file) != EOF || ferror(file))
    return fail(result, "trailing replay data");
  return true;
}

static bool poll_replay_window(driving_stage_t* stage,
                               session_result_t* result) {
  if (!stage->window->window) return true;
  window_events_t live = window_poll(stage->window);
  if (live.quit) return fail(result, "replay cancelled");
  if (live.resized)
    renderer_resize(stage->renderer, stage->window->drawable_width,
                    stage->window->drawable_height);
  return true;
}

static bool verify_frame_state(FILE* file, uint64_t actual,
                               session_result_t* result) {
  uint64_t expected;
  char line[SESSION_LINE_SIZE], *tokens[16];
  int count = read_tokens(file, line, tokens);
  if (count == 0) {
    snprintf(result->error, sizeof(result->error),
             "unverified frame %zu: missing state after pending input",
             result->frames + 1);
    return false;
  }
  if (count != 2 || strcmp(tokens[0], "state") ||
      !unsigned_number(tokens[1], 16, UINT64_MAX, &expected)) {
    snprintf(result->error, sizeof(result->error), "invalid state at frame %zu",
             result->frames + 1);
    return false;
  }
  if (actual != expected) {
    snprintf(result->error, sizeof(result->error),
             "replay diverged at frame %zu: expected %llx, got %llx",
             result->frames + 1, (unsigned long long)expected,
             (unsigned long long)actual);
    return false;
  }
  return true;
}

static bool replay_frame(FILE* file, driving_stage_t* stage,
                         const session_frame_t* frame,
                         session_result_t* result) {
  if (!poll_replay_window(stage, result)) return false;
  driving_stage_apply_frame(stage, events_from_actions(frame->actions),
                            frame->input, frame->horn, frame->seconds);
  uint64_t actual = session_state_hash(stage);
  if (!verify_frame_state(file, actual, result)) return false;
  result->state_hash = actual;
  result->frames++;
  if (stage->window->window) {
    driving_stage_render(stage);
    window_swap(stage->window);
  }
  return true;
}

bool session_replay(FILE* file, driving_stage_t* stage,
                    session_result_t* result) {
  *result = (session_result_t){0};
  session_header_t header;
  if (!session_read_header(file, &header))
    return fail(result, "invalid replay header");
  if (!matching_header(&header, stage))
    return fail(result, "initial replay state/content/vehicle differs");
  result->state_hash = header.initial_hash;
  char line[SESSION_LINE_SIZE], *tokens[16];
  int count;
  while ((count = read_tokens(file, line, tokens)) != 0) {
    if (count > 0 && strcmp(tokens[0], "end") == 0)
      return finish_replay(file, tokens, count, result);
    session_frame_t frame;
    if (!parse_frame(tokens, count, &frame)) {
      snprintf(result->error, sizeof(result->error), "invalid frame %zu",
               result->frames + 1);
      return false;
    }
    if (!replay_frame(file, stage, &frame, result)) return false;
  }
  return fail(result, "truncated replay: missing footer");
}

bool session_run_live(driving_stage_t* stage, FILE* recording,
                      size_t frame_limit) {
  if (recording && !session_write_header(recording, stage)) return false;
  double previous = platform_seconds();
  size_t frames = 0;
  while (!stage->quit && (frame_limit == 0 || frames < frame_limit)) {
    double now = platform_seconds();
    double seconds = now - previous;
    previous = now;
    window_events_t events = window_poll(stage->window);
    driving_input_t input = window_driving_input(stage->window);
    bool horn = window_horn_held(stage->window);
    if (recording) {
      if (!session_record_frame(recording, stage, events, input, horn, seconds))
        return false;
    } else {
      driving_stage_apply_frame(stage, events, input, horn, seconds);
    }
    frames++;
    driving_stage_render(stage);
    window_swap(stage->window);
    stage->frame_ms = (platform_seconds() - now) * 1000.0;
    stage->fps_accumulator += seconds;
    stage->fps_frames++;
    if (stage->fps_accumulator >= 0.5) {
      stage->fps = stage->fps_frames / stage->fps_accumulator;
      stage->fps_accumulator = 0;
      stage->fps_frames = 0;
    }
  }
  return !recording || session_finish(recording, stage, frames);
}
