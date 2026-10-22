/**
 * @file wanted.h
 * @brief The law: a hard crash is a crime and starts a chase; more crimes
 * mean more stars and more police cars. The player is busted if a police car
 * catches them standing, and gets away by staying out of reach for a while.
 */
#ifndef GAME_SRC_STAGE_WANTED_H_
#define GAME_SRC_STAGE_WANTED_H_

#define WANTED_MAX_STARS 5

typedef enum {
  WANTED_NOTHING,
  WANTED_STARTED,
  WANTED_BUSTED,
  WANTED_ESCAPED
} wanted_event_t;

typedef struct {
  int stars;          /* 0 when the law is not interested */
  float since_crime;  /* seconds since the last crime counted */
  float caught_for;   /* seconds a police car has had the player pinned */
  float out_of_reach; /* seconds with no police car near */
  float chase_time;   /* seconds since the stars began */
  int engaged;        /* a police car has been close: they found the player */
} wanted_t;

void wanted_init(wanted_t* wanted);

/* A crime: one more star, if the last was a moment ago or more. Returns
 * WANTED_STARTED if this is the first star. */
wanted_event_t wanted_crime(wanted_t* wanted);

/* How many police cars the law sends at this many stars: none at none, then
 * 1, 1, 2, 2, 3. */
int wanted_pursuers(const wanted_t* wanted);

/* Advances a chase: `nearest_police` is the distance to the nearest pursuing
 * car (a huge number with none) and `player_speed` the player's speed in
 * metres a second. Busted when a police car is within 9 m and the player
 * has been at a crawl for two seconds. Away when, once the police have come
 * within 150 m, none has been within 220 m for twelve seconds; or when they
 * never found the player in ninety. Either clears the stars. */
wanted_event_t wanted_step(wanted_t* wanted, float nearest_police,
                           float player_speed, float dt);

#endif  // GAME_SRC_STAGE_WANTED_H_
