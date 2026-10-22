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

/* What the run is, decided by the vehicle the player drives when it starts:
 * a taxi takes fares, an ambulance carries patients (quicker and better
 * paid), anything else delivers. */
typedef enum { MISSION_DELIVERY, MISSION_FARE, MISSION_PATIENT } mission_kind_t;

typedef struct {
  mission_kind_t kind; /* set by the caller while no run is active */
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

/* The words for a kind of run on the screen: what to do ("DELIVER", "FARE
 * TO", "PATIENT TO") and how it ends ("DELIVERED", "FARE PAID", "PATIENT
 * SAVED"), and how much better than a delivery it pays. */
const char* mission_verb(mission_kind_t kind);
const char* mission_done_text(mission_kind_t kind);
float mission_pay_rate(mission_kind_t kind);

/* The reward for a run of `distance` metres finished with `time_left`
 * seconds to spare. */
int mission_reward(float distance, float time_left);

#endif  // GAME_SRC_STAGE_MISSION_H_
