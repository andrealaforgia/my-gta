/**
 * @file air_stage.h
 * @brief Flying in the driving stage: the aircraft standing at the airport,
 * getting into one (F, on foot), flying it, and getting out. While piloting
 * the player's walker is pinned to the aircraft, so the people, traffic and
 * streaming that follow the player follow it too.
 */
#ifndef GAME_SRC_STAGE_AIR_STAGE_H_
#define GAME_SRC_STAGE_AIR_STAGE_H_

#include "driving_stage.h"

void air_stage_init(driving_stage_t* stage);
void air_stage_destroy(driving_stage_t* stage);
/* F on foot: into the nearest aircraft within reach. True if boarded. */
bool air_stage_board(driving_stage_t* stage);
/* The aircraft within reach of the walker, or NULL. */
const aircraft_t* air_stage_nearest(const driving_stage_t* stage);
/* F while piloting: out beside the aircraft if it is down and nearly
 * stopped; in the air, a jump with a parachute, and the aircraft left to dive
 * and crash. True if the player got out either way. */
bool air_stage_leave(driving_stage_t* stage);
/* One step of the descent on the parachute: steering turns, the throttle
 * drifts forward; the ground ends it. */
void air_stage_step_chute(driving_stage_t* stage, driving_input_t input,
                          float dt);
/* Aircraft with nobody in them dive and crash. */
void air_stage_step_adrift(driving_stage_t* stage, float dt);
/* The aircraft that was being flown goes back to its stand, and the player
 * is no longer piloting. */
void air_stage_stop(driving_stage_t* stage);
/* One step while piloting. */
void air_stage_step(driving_stage_t* stage, driving_input_t input, float dt);
/* The chase camera follows the aircraft. */
void air_stage_follow(driving_stage_t* stage, float frame_seconds);
void air_stage_draw(driving_stage_t* stage, float night);
/* Speed, height and power in place of the car's dial. */
void air_stage_draw_gauge(driving_stage_t* stage);
/* The aircraft the player is in, or NULL. */
const aircraft_t* air_stage_piloted(const driving_stage_t* stage);

#endif  // GAME_SRC_STAGE_AIR_STAGE_H_
