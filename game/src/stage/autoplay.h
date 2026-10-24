/**
 * @file autoplay.h
 * @brief A scripted player for demonstrations (--autoplay): it drives, hits a
 * car, steals a car, runs over a person, steals a truck, drives to the
 * airport, steals a plane, flies it across the city, lands, steals a
 * helicopter and flies that across the city. It presses what a player would
 * press: each frame it returns the driving input and calls the stage's own
 * interact function. Each act has a time limit, and a crash or an arrest
 * sends it back to a sensible act, so it carries on whatever happens.
 */
#ifndef GAME_SRC_STAGE_AUTOPLAY_H_
#define GAME_SRC_STAGE_AUTOPLAY_H_

#include <stdbool.h>
#include <stddef.h>

#include "input.h"

struct driving_stage;

typedef enum {
  ACT_START,
  ACT_DRIVE,
  ACT_HIT_CAR,
  ACT_STEAL_CAR,
  ACT_RUN_OVER,
  ACT_STEAL_TRUCK,
  ACT_TO_AIRPORT,
  ACT_BOARD_PLANE,
  ACT_TAKE_OFF,
  ACT_CROSS_CITY,
  ACT_LAND,
  ACT_STEAL_HELICOPTER,
  ACT_HELICOPTER,
  ACT_DONE
} autoplay_act_t;

typedef struct {
  bool enabled;
  autoplay_act_t act;
  float act_time; /* seconds in this act */
  float frame;    /* seconds since the last step, for the helpers */
  int step;       /* where in the act */
  int target;     /* the car, person or aircraft being gone after, -1 none */
  float path[2 * 128];
  size_t path_count;
  float path_clock;
  float stuck;                /* seconds the vehicle has been trying to move */
  float reverse_time;         /* seconds left backing out of a corner */
  int waypoint;               /* of a flight */
  int retries;                /* times a crash or an arrest sent it back */
  int downed_before;          /* people already down when the run-over began */
  int truck_tries;            /* lorries given up on */
  bool brought;               /* a target has been stood up the street */
  autoplay_act_t after_steal; /* where stealing a car leads */
  autoplay_act_t after_airport; /* where the airport leads */
  bool leaving;                 /* getting out of an aircraft on purpose */
  float yaw_hold;               /* the heading kept on a runway */
  bool town_found;              /* the middle of town has been worked out */
  float town_x, town_z;
  float walk_clock;     /* seconds since the walker's progress was checked */
  float walk_x, walk_z; /* where the walker was then */
  float wiggle;         /* seconds left of sidestepping an obstacle */
  float wiggle_side;
  float detour_x, detour_z; /* the way round an obstacle: a direction */
  int detour_sign;          /* which way round: -1 or +1, 0 not yet chosen */
  float detour_hold;        /* seconds left of keeping to it */
  float block_time;         /* seconds held up behind something */
  float pass_time;          /* seconds left of going round it */
  float pass_side;
  int pass_tries;         /* times it has had to go round something */
  bool reverse_side_hold; /* backing straight out, not round a corner */
  float watch_clock;      /* progress is checked every so often */
  float watch_x, watch_z;
  int watch_stuck;
  float law_clock; /* seconds since the police were called off */
  float ground_y;  /* the airfield's height */
} autoplay_t;

void autoplay_init(autoplay_t* autoplay);
/* The input for this frame; may press F or fire through the stage. */
driving_input_t autoplay_step(autoplay_t* autoplay, struct driving_stage* stage,
                              float dt);
/* Words for the screen, such as "STEAL A TRUCK". */
const char* autoplay_label(const autoplay_t* autoplay);

#endif  // GAME_SRC_STAGE_AUTOPLAY_H_
