/**
 * @file walker.h
 * @brief The player on foot: turns and walks or runs over the ground, is
 * stopped by walls and solid props, steps up kerbs but not walls, and falls
 * if the ground goes. There is no body in the physics world: the ground and
 * the obstacles are found with rays, which is plenty for one person.
 */
#ifndef GAME_SRC_STAGE_WALKER_H_
#define GAME_SRC_STAGE_WALKER_H_

#include <stdbool.h>

#include "physics_world.h"
#include "vec3.h"

typedef struct {
  vec3_t position; /* feet */
  float yaw;       /* radians; 0 faces -z, positive turns left */
  float speed;     /* metres a second forwards (negative backwards) */
  float fall;      /* metres a second downwards while off the ground */
  float phase;     /* stride, radians, for the swing of arms and legs */
  bool grounded;
} walker_t;

typedef struct {
  float forward; /* -1 back .. 1 forward */
  float turn;    /* -1 left .. 1 right */
  bool run;
} walker_input_t;

#define WALKER_RADIUS 0.35f
#define WALKER_WALK_SPEED 2.0f
#define WALKER_RUN_SPEED 5.5f
#define WALKER_STEP_HEIGHT 0.45f

void walker_init(walker_t* walker, vec3_t feet, float yaw);
void walker_step(walker_t* walker, const physics_world_t* physics,
                 walker_input_t input, float dt);
/* Pushes the walker out of an oriented box (a parked car), if inside it. */
void walker_push_out_of_box(walker_t* walker, vec3_t centre, float yaw,
                            vec3_t half_extents);
/* The unit vector the walker faces, flat. */
vec3_t walker_forward(const walker_t* walker);

#endif  // GAME_SRC_STAGE_WALKER_H_
