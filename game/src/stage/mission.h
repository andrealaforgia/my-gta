/**
 * @file mission.h
 * @brief The delivery runs that give the drive a purpose: a destination on
 * a road some way off, a time to reach it in and a cash reward. When one
 * ends, another follows after a short pause.
 */
#ifndef GAME_SRC_STAGE_MISSION_H_
#define GAME_SRC_STAGE_MISSION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "city_package.h"

#define MISSION_ARRIVAL_RADIUS 9.0f

typedef enum {
  MISSION_NOTHING,
  MISSION_STARTED,
  MISSION_DELIVERED,
  MISSION_TIMED_OUT
} mission_event_t;

typedef struct {
  bool active;
  float x, z;       /* the destination */
  float time_left;  /* seconds */
  float time_limit; /* seconds the run was given */
  float distance;   /* metres from where it began to the destination */
  float pause;      /* seconds until the next run, while none is active */
  int cash;         /* earned so far */
  int delivered, failed;
  int last_reward;   /* of the run that has just ended, 0 if it failed */
  float result_time; /* seconds the result stays on the screen */
  uint32_t random;
} mission_t;

void mission_init(mission_t* mission, uint32_t seed);

/* Advances the run with the car at (px, pz): starts the next one when it is
 * time, delivers on arrival, times out when the clock runs down. */
mission_event_t mission_step(mission_t* mission, const city_route_t* routes,
                             size_t route_count, float px, float pz, float dt);

/* Straight-line metres to the destination. */
float mission_distance_to(const mission_t* mission, float px, float pz);

/* The reward for a run of `distance` metres finished with `time_left`
 * seconds to spare. */
int mission_reward(float distance, float time_left);

#endif  // GAME_SRC_STAGE_MISSION_H_
